"""Network endpoints for the capture card and the robot controller."""

from dataclasses import dataclass


@dataclass(frozen=True)
class RadarNetworkConfig:
    """UDP endpoints of the TI DCA1000EVM capture card.

    ``fpga_*`` addresses the card; ``host_*`` addresses the local interface the
    card streams to.
    """

    fpga_ip: str = "192.168.33.180"
    fpga_port: int = 4096
    host_ip: str = "192.168.33.30"
    host_config_port: int = 4096
    host_data_port: int = 4098

    @property
    def fpga_endpoint(self) -> tuple:
        """``(ip, port)`` of the card's configuration service."""
        return (self.fpga_ip, self.fpga_port)

    @property
    def host_config_endpoint(self) -> tuple:
        """Local ``(ip, port)`` bound for configuration replies."""
        return (self.host_ip, self.host_config_port)

    @property
    def host_data_endpoint(self) -> tuple:
        """Local ``(ip, port)`` bound for the raw ADC stream."""
        return (self.host_ip, self.host_data_port)


@dataclass(frozen=True)
class RobotNetworkConfig:
    """TCP endpoints of the Universal Robots UR3 controller."""

    ip: str = "169.254.200.239"
    rtde_port: int = 30004
    realtime_port: int = 30003
