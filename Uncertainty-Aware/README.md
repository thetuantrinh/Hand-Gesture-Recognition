# radar-hgr — Real-Time Uncertainty-Aware Radar HGR & UR3 Control

Real-time application accompanying *"Hand Gesture Recognition With Uncertainty
Awareness via FMCW Radar Sensing and Deep Learning"* (IEEE Sensors Journal,
2025). It streams raw ADC data from a TI AWR1243BOOST / DCA1000EVM pair,
classifies micro-Doppler signatures, and drives a Universal Robots UR3
manipulator from the recognised gestures.

<div align="center">
<img src="https://drive.google.com/uc?export=view&id=123tEgcBX-lTC2jh5HfkZ2sWziaGvC7AN" alt="Real-time control GUI" width="650">
</div>

## Installation

```bash
python3 -m venv .venv && source .venv/bin/activate

pip install -e .                 # acquisition, DSP and robot control
pip install -e '.[inference]'    # additionally installs TensorFlow
pip install -e '.[dev]'          # additionally installs pytest and ruff
```

TensorFlow is an optional extra. Without it the GUI still starts and supports
radar acquisition, the live micro-Doppler display and manual robot control;
only gesture classification is disabled.

## Running

```bash
radar-hgr                        # console script
python -m radar_hgr              # equivalent
python -m radar_hgr --model models/model_1.h5 --verbose
```

Before launching, confirm the host is on the capture card's network (default
`192.168.33.30`) and can reach the UR3 controller or a URSim instance.

In the GUI: start the radar to see the live micro-Doppler map, tick
**Predict Hand Gesture** to classify, then start the UR3 and tick
**Control UR3 by Hand Gesture** to close the loop. The jog pad and gripper
selector work independently of the classifier.

### Configuration

Defaults live in [`src/radar_hgr/config/`](src/radar_hgr/config) as frozen
dataclasses. Two environment variables override model discovery:

| Variable | Effect |
| :--- | :--- |
| `RADAR_HGR_MODEL` | Checkpoint to load when `--model` is not given |
| `RADAR_HGR_MODELS_DIR` | Directory searched for `model_2.h5`, then `model_1.h5` |

## Layout

```text
Uncertainty-Aware/
├── pyproject.toml            # Packaging, dependencies, pytest and ruff config
├── models/                   # Pre-trained checkpoints
├── Datasets/                 # Dataset specification (data available on request)
├── tests/                    # Hardware-free unit tests
└── src/radar_hgr/
    ├── cli.py                # Argument parsing and the console-script entry point
    ├── logging_config.py     # Console logging setup
    ├── config/               # Frozen dataclasses: radar, network, robot, features, paths
    ├── dsp/                  # windows → filters → transforms → pipeline; framing
    ├── radar/                # protocol (pure codec) + dca1000 (sockets)
    ├── inference/            # smoothing (framework-free) + keras_model + predictor
    ├── robot/                # actions, controllers, safety, ur3, and vendored sdk/
    ├── workers/              # Qt threads for acquisition and telemetry
    ├── ui/                   # layout/ (widgets), views/ (presentation), assets/
    └── app/                  # sessions and main_window: wiring only
```

Dependencies point strictly downwards: `config` depends on nothing, the four
domain packages depend only on `config`, and `app` is the only layer that knows
about both the widgets and the hardware.

### Where things are

| Concern | Module |
| :--- | :--- |
| Range-Doppler chain | [`dsp/pipeline.py`](src/radar_hgr/dsp/pipeline.py) |
| Packet-to-frame reassembly | [`dsp/framing.py`](src/radar_hgr/dsp/framing.py) |
| Micro-Doppler feature window | [`app/gesture_pipeline.py`](src/radar_hgr/app/gesture_pipeline.py) |
| Capture-card wire format | [`radar/protocol.py`](src/radar_hgr/radar/protocol.py) |
| Prediction stabilisation | [`inference/smoothing.py`](src/radar_hgr/inference/smoothing.py) |
| Gesture → motion vocabulary | [`robot/actions.py`](src/radar_hgr/robot/actions.py) |
| Workspace limits | [`robot/safety.py`](src/radar_hgr/robot/safety.py) |
| Window layout | [`ui/layout/`](src/radar_hgr/ui/layout) |

## Development

```bash
pytest                 # 68 tests, no radar or robot required
ruff check src tests
```

The tests cover the DSP chain, frame reassembly, the wire protocol, vote
smoothing, the gesture vocabulary, feature windowing and path resolution. The
hardware clients and Qt widgets are not unit-tested; they are exercised by
running the application.

### Notes for maintainers

- **The window layout is hand-maintained, not generated.**
  [`ui/layout/`](src/radar_hgr/ui/layout) began as `pyuic5` output from
  [`ui/designer/UI.ui`](src/radar_hgr/ui/designer/UI.ui) and was then edited by
  hand. Regenerating from the `.ui` file would discard those edits. Widgets are
  positioned with absolute geometry, so **creation order sets stacking order** —
  see the note on `PANELS` in
  [`ui/layout/main_window.py`](src/radar_hgr/ui/layout/main_window.py).
- **`robot/sdk/` is vendored third-party code** derived from `URBasic`. It keeps
  upstream's naming and style, is excluded from linting, and should be reached
  only through `robot/ur3.py`.
- **The Doppler FFT shifts every axis, not just the Doppler axis.** This is not
  textbook behaviour, but the shipped checkpoints were trained on features
  produced this way. See `center_zero_velocity` in
  [`dsp/transforms.py`](src/radar_hgr/dsp/transforms.py) before changing it.

## License

MIT — see [LICENSE](LICENSE).
