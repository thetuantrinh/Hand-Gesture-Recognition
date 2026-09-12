"""Tests for the gesture-to-motion vocabulary."""

import pytest

from radar_hgr.config.gestures import Gesture
from radar_hgr.config.robot import RobotConfig, WorkspaceLimits
from radar_hgr.robot.actions import GESTURE_ACTIONS, ToolState, action_for, translate
from radar_hgr.robot.controllers import GestureController, JogController
from radar_hgr.robot.safety import SafetySupervisor

HOME = [0.0, 0.0, 0.0, 1.0, 2.0, 3.0]
STEP = RobotConfig().step_distance


class FakeRobot:
    """Records the commands a controller issues, in place of a UR3."""

    def __init__(self):
        self.poses = []
        self.tool_states = []

    def set_pose(self, pose):
        self.poses.append(list(pose))

    def set_tool_state(self, state):
        self.tool_states.append(state)


def test_translate_leaves_orientation_untouched():
    moved = translate(HOME, (0.1, 0.2, 0.3))
    assert moved[3:] == HOME[3:]
    assert moved[:3] == pytest.approx([0.1, 0.2, 0.3])


def test_translate_does_not_mutate_its_input():
    original = list(HOME)
    translate(original, (1.0, 1.0, 1.0))
    assert original == HOME


def test_inert_gestures_command_nothing():
    for gesture in (Gesture.EMPTY, Gesture.CLOCKWISE, Gesture.COUNTER_CLOCKWISE, Gesture.UNKNOWN):
        assert action_for(gesture) is None


def test_every_mapped_action_is_a_move_or_a_tool_command():
    for gesture, action in GESTURE_ACTIONS.items():
        is_move = action.translation is not None
        is_tool = action.tool_state is not None
        assert is_move != is_tool, gesture


@pytest.mark.parametrize(
    ("gesture", "axis", "sign"),
    [
        (Gesture.LOWER_WORKPIECE, 2, -1),
        (Gesture.LIFT_WORKPIECE, 2, +1),
        (Gesture.MOVE_LEFT, 0, +1),
        (Gesture.MOVE_RIGHT, 0, -1),
    ],
)
def test_translating_gestures_move_one_axis_by_one_step(gesture, axis, sign):
    robot = FakeRobot()
    controller = GestureController(robot, HOME)

    assert controller.execute(gesture)

    expected = list(HOME)
    expected[axis] += sign * STEP
    assert robot.poses == [pytest.approx(expected)]


@pytest.mark.parametrize(
    ("gesture", "state"),
    [
        (Gesture.CLAMP_WORKPIECE, ToolState.CLAMP),
        (Gesture.RELEASE_WORKPIECE, ToolState.RELEASE),
    ],
)
def test_tool_gestures_actuate_the_gripper(gesture, state):
    robot = FakeRobot()
    controller = GestureController(robot, HOME)

    assert controller.execute(gesture)

    assert robot.tool_states == [state]
    assert robot.poses == []


def test_an_inert_gesture_sends_no_command():
    robot = FakeRobot()
    controller = GestureController(robot, HOME)

    assert not controller.execute(Gesture.UNKNOWN)

    assert robot.poses == []
    assert robot.tool_states == []


def test_the_supervisor_vetoes_a_move_out_of_the_workspace():
    robot = FakeRobot()
    # HOME sits at z = 0, so every commanded height is outside this band.
    supervisor = SafetySupervisor(WorkspaceLimits(z_min=-0.40, z_max=-0.36))
    controller = GestureController(robot, HOME, supervisor=supervisor)

    assert not controller.execute(Gesture.LIFT_WORKPIECE)
    assert robot.poses == []


def test_the_supervisor_permits_a_move_inside_the_workspace():
    robot = FakeRobot()
    supervisor = SafetySupervisor(WorkspaceLimits(z_min=-0.40, z_max=-0.36))
    start = [0.0, 0.0, -0.37, 0.0, 0.0, 0.0]
    controller = GestureController(robot, start, supervisor=supervisor)

    assert controller.execute(Gesture.LOWER_WORKPIECE)
    assert robot.poses[0][2] == pytest.approx(-0.37 - STEP)


@pytest.mark.parametrize(
    ("command", "axis", "sign"),
    [
        ("up", 2, +1),
        ("down", 2, -1),
        ("left", 0, -1),
        ("right", 0, +1),
        ("inward", 1, -1),
        ("outward", 1, +1),
    ],
)
def test_jog_commands_move_the_expected_axis(command, axis, sign):
    robot = FakeRobot()
    controller = JogController(robot, HOME)

    getattr(controller, command)()

    expected = list(HOME)
    expected[axis] += sign * STEP
    assert robot.poses == [pytest.approx(expected)]


def test_jog_gripper_commands():
    robot = FakeRobot()
    controller = JogController(robot, HOME)

    controller.clamp()
    controller.release()

    assert robot.tool_states == [ToolState.CLAMP, ToolState.RELEASE]
