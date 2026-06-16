"""Load camera_params.yaml for the camera node.

Resolution priority (see spb_node_common.config_base.locate_config_file):
  1. CAMERA_NODE_CONFIG / CAMERA_NODE_SECRETS environment variables
  2. the installed package share dir: share/camera_node_1/config/camera_params.yaml
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from spb_node_common.config_base import (
    BrokerConfig, NodeConfigBase, deep_get, load_yaml_with_secrets,
)


def _share_dir() -> Path:
    return Path(get_package_share_directory("camera_node_1"))


class HsvRange:
    def __init__(self, lower: list, upper: list):
        self.lower = lower
        self.upper = upper


class CameraConfig(NodeConfigBase):
    """Typed view over camera_params.yaml."""

    # ── Camera / detection ────────────────────────────────────────────────

    @property
    def frame_width(self) -> int:
        return int(deep_get(self._d, "camera", "frame_width", default=640))

    @property
    def frame_height(self) -> int:
        return int(deep_get(self._d, "camera", "frame_height", default=400))

    @property
    def detection_timeout_s(self) -> float:
        return float(deep_get(self._d, "camera", "detection_timeout_s", default=30.0))

    @property
    def frame_counter(self) -> int:
        return int(deep_get(self._d, "camera", "frame_counter", default=30))

    @property
    def pellet_color(self) -> HsvRange:
        s = deep_get(self._d, "camera", "pellet_color") or {}
        return HsvRange(
            lower=list(s.get("lower_hsv", [28, 140, 160])),
            upper=list(s.get("upper_hsv", [38, 255, 255])),
        )

    @property
    def foreign_color(self) -> HsvRange:
        s = deep_get(self._d, "camera", "foreign_color") or {}
        return HsvRange(
            lower=list(s.get("lower_hsv", [0, 0, 0])),
            upper=list(s.get("upper_hsv", [0, 0, 0])),
        )

    @property
    def pellet_pixel_threshold(self) -> int:
        return int(deep_get(self._d, "camera", "pellet_pixel_threshold", default=100))

    @property
    def foreign_pixel_threshold(self) -> int:
        return int(deep_get(self._d, "camera", "foreign_pixel_threshold", default=50))


_cached: CameraConfig | None = None


def load_config(reload: bool = False) -> CameraConfig:
    global _cached
    if _cached is None or reload:
        data = load_yaml_with_secrets(
            config_env_var="CAMERA_NODE_CONFIG",
            secrets_env_var="CAMERA_NODE_SECRETS",
            default_config_filename="camera_params.yaml",
            default_secrets_filename="camera_secrets.yaml",
            anchor_dir=_share_dir(),
        )
        _cached = CameraConfig(data)
    return _cached
