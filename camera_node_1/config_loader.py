"""Load camera_params.yaml, resolving the file via (in priority order):
1. CAMERA_NODE_CONFIG environment variable
2. Sibling config/ directory (in-source)
"""

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

_DEFAULT_FILENAME = "camera_params.yaml"
_SECRETS_FILENAME = "camera_secrets.yaml"


def _locate_config() -> Path:
    env_override = os.environ.get("CAMERA_NODE_CONFIG")
    if env_override:
        p = Path(env_override).expanduser()
        if p.is_file():
            return p
        raise FileNotFoundError(f"CAMERA_NODE_CONFIG points to missing file: {p}")

    here = Path(__file__).parent
    candidate = here / "config" / _DEFAULT_FILENAME
    if candidate.is_file():
        return candidate

    raise FileNotFoundError(
        f"Could not locate {_DEFAULT_FILENAME}. "
        "Set CAMERA_NODE_CONFIG or place the file at config/camera_params.yaml."
    )


def _locate_secrets(config_path: Path) -> Path | None:
    env_override = os.environ.get("CAMERA_NODE_SECRETS")
    if env_override:
        p = Path(env_override).expanduser()
        if p.is_file():
            return p
        raise FileNotFoundError(f"CAMERA_NODE_SECRETS points to missing file: {p}")

    candidate = config_path.parent / _SECRETS_FILENAME
    return candidate if candidate.is_file() else None


def _deep_merge(base: dict, overlay: dict) -> dict:
    for k, v in overlay.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _deep_merge(base[k], v)
        else:
            base[k] = v
    return base


def _deep_get(d: dict, *keys, default=None) -> Any:
    for k in keys:
        if not isinstance(d, dict):
            return default
        d = d.get(k, {})
    return d if d != {} else default


@dataclass
class BrokerConfig:
    host: str
    port: int
    use_tls: bool
    username: str
    password: str


@dataclass
class HsvRange:
    lower: list  # [H, S, V]
    upper: list  # [H, S, V]


class CameraConfig:
    """Typed view over camera_params.yaml."""

    def __init__(self, data: dict):
        self._d = data

    # ── ISA-95 / Sparkplug B identity ────────────────────────────────────

    @property
    def spb_group_id(self) -> str:
        return _deep_get(self._d, "spb_bridge", "group_id", default="DMATDTS_DLSU_LS_MiniFactory")

    @property
    def spb_edge_node_id(self) -> str:
        return _deep_get(self._d, "spb_bridge", "edge_node_id", default="camera_node")

    @property
    def spb_device_id(self) -> str:
        return _deep_get(self._d, "spb_bridge", "device_id", default="depthai_camera")

    @property
    def metric_prefix(self) -> str:
        return _deep_get(
            self._d, "spb_bridge", "metric_prefix",
            default=""
        )

    @property
    def primary_host_id(self) -> str:
        return _deep_get(self._d, "spb_bridge", "primary_host_id", default="IgnitionPrimary")

    @property
    def heartbeat_interval_s(self) -> float:
        return float(_deep_get(self._d, "spb_bridge", "heartbeat_interval_s", default=0.5))

    # ── Broker selection ──────────────────────────────────────────────────

    @property
    def broker_type(self) -> str:
        return _deep_get(self._d, "spb_bridge", "broker_type", default="local")

    @property
    def local_broker(self) -> BrokerConfig:
        s = _deep_get(self._d, "spb_bridge", "local") or {}
        return BrokerConfig(
            host=s.get("host", "localhost"),
            port=int(s.get("port", 1883)),
            use_tls=bool(s.get("use_tls", False)),
            username=s.get("username", "") or "",
            password=s.get("password", "") or "",
        )

    @property
    def hivemq_broker(self) -> BrokerConfig:
        s = _deep_get(self._d, "spb_bridge", "hivemq") or {}
        return BrokerConfig(
            host=s.get("host", ""),
            port=int(s.get("port", 8883)),
            use_tls=bool(s.get("use_tls", True)),
            username=s.get("username", "") or "",
            password=s.get("password", "") or "",
        )

    def active_broker(self) -> BrokerConfig:
        if self.broker_type == "hivemq":
            return self.hivemq_broker
        return self.local_broker

    # ── Camera / detection ────────────────────────────────────────────────

    @property
    def frame_width(self) -> int:
        return int(_deep_get(self._d, "camera", "frame_width", default=640))

    @property
    def frame_height(self) -> int:
        return int(_deep_get(self._d, "camera", "frame_height", default=400))

    @property
    def detection_timeout_s(self) -> float:
        return float(_deep_get(self._d, "camera", "detection_timeout_s", default=30.0))

    @property
    def pellet_color(self) -> HsvRange:
        s = _deep_get(self._d, "camera", "pellet_color") or {}
        return HsvRange(
            lower=list(s.get("lower_hsv", [28, 140, 160])),
            upper=list(s.get("upper_hsv", [38, 255, 255])),
        )

    @property
    def foreign_color(self) -> HsvRange:
        s = _deep_get(self._d, "camera", "foreign_color") or {}
        return HsvRange(
            lower=list(s.get("lower_hsv", [0, 0, 0])),
            upper=list(s.get("upper_hsv", [0, 0, 0])),
        )

    @property
    def pellet_pixel_threshold(self) -> int:
        return int(_deep_get(self._d, "camera", "pellet_pixel_threshold", default=100))

    @property
    def foreign_pixel_threshold(self) -> int:
        return int(_deep_get(self._d, "camera", "foreign_pixel_threshold", default=50))


_cached: CameraConfig | None = None


def load_config(reload: bool = False) -> CameraConfig:
    global _cached
    if _cached is None or reload:
        path = _locate_config()
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        secrets_path = _locate_secrets(path)
        if secrets_path is not None:
            with open(secrets_path) as f:
                secrets = yaml.safe_load(f) or {}
            _deep_merge(data, secrets)
        _cached = CameraConfig(data)
    return _cached
