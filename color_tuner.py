"""Interactive HSV colour threshold tuner for pellet detection.

Shows the live camera feed alongside the current pellet mask in real-time.
HSV sliders update the preview instantly.

Presets are saved to  config/color_presets.yaml  (per-name entries).
"Apply & Restart" writes the active values into  config/camera_secrets.yaml
(the gitignored overlay) and asks the parent to restart the camera worker.
"""

from pathlib import Path

import cv2
import numpy as np
import yaml

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QFont, QImage, QPixmap
from PyQt5.QtWidgets import (
    QComboBox, QDialog, QGroupBox, QHBoxLayout, QLabel,
    QLineEdit, QMessageBox, QPushButton, QSizePolicy,
    QSlider, QVBoxLayout, QWidget,
)

_CONFIG_DIR    = Path(__file__).parent / "config"
_PRESETS_FILE  = _CONFIG_DIR / "color_presets.yaml"
_SECRETS_FILE  = _CONFIG_DIR / "camera_secrets.yaml"


# ---------------------------------------------------------------------------
# YAML helpers
# ---------------------------------------------------------------------------

def _load_presets() -> dict:
    if _PRESETS_FILE.is_file():
        data = yaml.safe_load(_PRESETS_FILE.read_text()) or {}
        return data.get("presets", {})
    return {}


def _save_presets(presets: dict):
    _CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    _PRESETS_FILE.write_text(
        yaml.dump({"presets": presets}, default_flow_style=None, sort_keys=False)
    )


def _apply_to_secrets(lower: list, upper: list):
    """Merge pellet colour values into camera_secrets.yaml without touching other keys."""
    data: dict = {}
    if _SECRETS_FILE.is_file():
        data = yaml.safe_load(_SECRETS_FILE.read_text()) or {}
    data.setdefault("camera", {})["pellet_color"] = {
        "lower_hsv": lower,
        "upper_hsv": upper,
    }
    _SECRETS_FILE.write_text(
        yaml.dump(data, default_flow_style=None, sort_keys=False)
    )


# ---------------------------------------------------------------------------
# Slider row widget
# ---------------------------------------------------------------------------

class _SliderRow(QWidget):
    """Label + horizontal slider + numeric readout in one row."""

    changed = pyqtSignal(int)

    def __init__(self, label: str, max_val: int, init_val: int, parent=None):
        super().__init__(parent)
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)

        lbl = QLabel(label)
        lbl.setFixedWidth(50)
        lbl.setFont(QFont("Monospace", 9))
        row.addWidget(lbl)

        self._slider = QSlider(Qt.Horizontal)
        self._slider.setRange(0, max_val)
        self._slider.setValue(init_val)
        row.addWidget(self._slider)

        self._readout = QLabel(str(init_val))
        self._readout.setFixedWidth(34)
        self._readout.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self._readout.setFont(QFont("Monospace", 9))
        row.addWidget(self._readout)

        self._slider.valueChanged.connect(self._on_change)

    def _on_change(self, v: int):
        self._readout.setText(str(v))
        self.changed.emit(v)

    def value(self) -> int:
        return self._slider.value()

    def set_value(self, v: int):
        self._slider.setValue(v)


# ---------------------------------------------------------------------------
# Main dialog
# ---------------------------------------------------------------------------

class ColorTunerDialog(QDialog):
    """Interactive HSV colour threshold tuner with preset save / load."""

    apply_requested = pyqtSignal()  # parent connects this to _start_camera()

    def __init__(self, worker, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Colour Threshold Tuner — Pellet Detection")
        self.setMinimumSize(920, 640)

        self._worker = worker
        self._latest_raw: np.ndarray | None = None
        self._presets: dict = _load_presets()

        # Load initial HSV bounds from config
        from config_loader import load_config
        cfg = load_config()
        lower = list(cfg.pellet_color.lower)
        upper = list(cfg.pellet_color.upper)

        self._build_ui(lower, upper)
        self._refresh_preset_combo()

        if self._worker is not None:
            self._worker.raw_frame_ready.connect(self._on_raw_frame)

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self, lower: list, upper: list):
        root = QVBoxLayout(self)
        root.setSpacing(8)
        root.setContentsMargins(10, 10, 10, 10)

        # ── Live preview row ──────────────────────────────────────────
        preview = QHBoxLayout()
        preview.setSpacing(8)

        for attr, title in [("_raw_label", "Live Feed"), ("_mask_label", "Pellet Mask")]:
            col = QVBoxLayout()
            col.setSpacing(2)
            hdr = QLabel(title)
            hdr.setFont(QFont("Sans Serif", 10, QFont.Bold))
            col.addWidget(hdr)
            lbl = QLabel("No feed")
            lbl.setFixedSize(430, 290)
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet(
                "background: #111111; color: #555555;"
                "border: 1px solid #333333; border-radius: 3px;"
            )
            col.addWidget(lbl)
            setattr(self, attr, lbl)
            preview.addLayout(col)

        root.addLayout(preview)

        # ── HSV sliders ───────────────────────────────────────────────
        slider_box = QGroupBox("Pellet Colour — HSV Range  (OpenCV scale: H 0–179, S/V 0–255)")
        slider_layout = QVBoxLayout(slider_box)
        slider_layout.setSpacing(3)

        # Order: H min, H max, S min, S max, V min, V max
        defs = [
            ("H  min", 179, lower[0]),
            ("H  max", 179, upper[0]),
            ("S  min", 255, lower[1]),
            ("S  max", 255, upper[1]),
            ("V  min", 255, lower[2]),
            ("V  max", 255, upper[2]),
        ]
        self._bands: list[_SliderRow] = []
        for lbl, max_val, init in defs:
            row = _SliderRow(lbl, max_val, init)
            row.changed.connect(self._on_slider_change)
            slider_layout.addWidget(row)
            self._bands.append(row)

        root.addWidget(slider_box)

        # ── Preset management ─────────────────────────────────────────
        preset_box = QGroupBox("Presets")
        preset_layout = QHBoxLayout(preset_box)
        preset_layout.setSpacing(8)

        preset_layout.addWidget(QLabel("Load:"))
        self._preset_combo = QComboBox()
        self._preset_combo.setMinimumWidth(170)
        self._preset_combo.currentTextChanged.connect(self._on_preset_selected)
        preset_layout.addWidget(self._preset_combo)

        del_btn = QPushButton("Delete")
        del_btn.clicked.connect(self._on_delete_preset)
        preset_layout.addWidget(del_btn)

        preset_layout.addSpacing(24)
        preset_layout.addWidget(QLabel("Save as:"))

        self._preset_name = QLineEdit()
        self._preset_name.setPlaceholderText("preset name…")
        self._preset_name.setMinimumWidth(140)
        preset_layout.addWidget(self._preset_name)

        save_btn = QPushButton("Save Preset")
        save_btn.clicked.connect(self._on_save_preset)
        preset_layout.addWidget(save_btn)

        preset_layout.addStretch()
        root.addWidget(preset_box)

        # ── Action buttons ────────────────────────────────────────────
        btn_row = QHBoxLayout()
        btn_row.addStretch()

        apply_btn = QPushButton("Apply & Restart Camera")
        apply_btn.setMinimumHeight(32)
        apply_btn.setStyleSheet("QPushButton { font-weight: bold; }")
        apply_btn.clicked.connect(self._on_apply)
        btn_row.addWidget(apply_btn)

        close_btn = QPushButton("Close")
        close_btn.setMinimumHeight(32)
        close_btn.clicked.connect(self.close)
        btn_row.addWidget(close_btn)

        root.addLayout(btn_row)

    # ------------------------------------------------------------------
    # HSV helpers
    # ------------------------------------------------------------------

    def _lower_hsv(self) -> list:
        return [self._bands[0].value(), self._bands[2].value(), self._bands[4].value()]

    def _upper_hsv(self) -> list:
        return [self._bands[1].value(), self._bands[3].value(), self._bands[5].value()]

    def _on_slider_change(self, _=None):
        if self._latest_raw is not None:
            self._update_mask(self._latest_raw)

    # ------------------------------------------------------------------
    # Frame display
    # ------------------------------------------------------------------

    def _on_raw_frame(self, bgr: np.ndarray):
        self._latest_raw = bgr
        self._show_bgr(self._raw_label, bgr)
        self._update_mask(bgr)

    def _update_mask(self, bgr: np.ndarray):
        lower = np.array(self._lower_hsv())
        upper = np.array(self._upper_hsv())
        hsv   = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask  = cv2.inRange(hsv, lower, upper)

        vis = np.zeros_like(bgr)
        vis[mask > 0] = [0, 255, 0]

        count = cv2.countNonZero(mask)
        cv2.putText(vis, f"{count:,} px", (8, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (200, 200, 200), 1, cv2.LINE_AA)

        self._show_bgr(self._mask_label, vis)

    def _show_bgr(self, label: QLabel, bgr: np.ndarray):
        h, w, ch = bgr.shape
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        img = QImage(rgb.data.tobytes(), w, h, ch * w, QImage.Format_RGB888)
        label.setPixmap(
            QPixmap.fromImage(img).scaled(
                label.width(), label.height(),
                Qt.KeepAspectRatio, Qt.SmoothTransformation,
            )
        )

    # ------------------------------------------------------------------
    # Preset management
    # ------------------------------------------------------------------

    def _refresh_preset_combo(self):
        self._preset_combo.blockSignals(True)
        self._preset_combo.clear()
        self._preset_combo.addItem("— select preset —")
        for name in sorted(self._presets):
            self._preset_combo.addItem(name)
        self._preset_combo.blockSignals(False)

    def _on_preset_selected(self, name: str):
        if name.startswith("—") or name not in self._presets:
            return
        p = self._presets[name]
        lower = p.get("lower_hsv", [0, 0, 0])
        upper = p.get("upper_hsv", [179, 255, 255])
        vals = [lower[0], upper[0], lower[1], upper[1], lower[2], upper[2]]
        for band, val in zip(self._bands, vals):
            band.set_value(val)
        self._preset_name.setText(name)

    def _on_save_preset(self):
        name = self._preset_name.text().strip()
        if not name:
            QMessageBox.warning(self, "Save Preset", "Enter a name for this preset.")
            return
        self._presets[name] = {
            "lower_hsv": self._lower_hsv(),
            "upper_hsv": self._upper_hsv(),
        }
        _save_presets(self._presets)
        self._refresh_preset_combo()
        idx = self._preset_combo.findText(name)
        if idx >= 0:
            self._preset_combo.setCurrentIndex(idx)

    def _on_delete_preset(self):
        name = self._preset_combo.currentText()
        if name.startswith("—") or name not in self._presets:
            return
        if QMessageBox.question(self, "Delete Preset",
                                f"Delete preset '{name}'?") != QMessageBox.Yes:
            return
        del self._presets[name]
        _save_presets(self._presets)
        self._refresh_preset_combo()

    # ------------------------------------------------------------------
    # Apply to config
    # ------------------------------------------------------------------

    def _on_apply(self):
        lower = self._lower_hsv()
        upper = self._upper_hsv()
        _apply_to_secrets(lower, upper)

        from config_loader import load_config
        load_config(reload=True)

        self.apply_requested.emit()

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def closeEvent(self, event):
        if self._worker is not None:
            try:
                self._worker.raw_frame_ready.disconnect(self._on_raw_frame)
            except Exception:
                pass
        event.accept()
