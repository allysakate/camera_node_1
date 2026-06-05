"""Standalone camera preview and detection monitor — visualization and debugging.

Runs the DepthAI camera, shows the live annotated feed, and displays the
pass / reject result with pellet and foreign-object pixel counts.

This is a development/debugging tool only.  For SCADA integration run
camera_spb_node.py separately (headless, no GUI dependency).

NOTE: Only one process can hold the DepthAI device at a time.
      Do not run camera_gui.py and camera_spb_node.py simultaneously.

Usage:
    python camera_gui.py
    CAMERA_NODE_CONFIG=/path/to/camera_params.yaml python camera_gui.py
"""

import os
import sys
import threading

# camera.py imports cv2, which sets QT_QPA_PLATFORM_PLUGIN_PATH to its own
# bundled Qt plugins directory — this breaks PyQt5's xcb platform plugin.
# Import camera first so cv2 runs, then hard-override the path before
# QApplication() is instantiated (that is when Qt reads the plugin path).
from camera import VideoWorker, enumerate_webcams
from camera_spb_node import CameraSpbBridge
from color_tuner import ColorTunerDialog
from config_loader import load_config as _load_cam_cfg
os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = "/usr/lib/x86_64-linux-gnu/qt5/plugins"

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QHBoxLayout, QFrame,
    QLabel, QPushButton, QComboBox, QGroupBox, QSizePolicy, QProgressBar,
    QButtonGroup,
)

_DOT_IDLE    = "color: #444444;"
_DOT_PASS    = "color: #22CC55;"
_DOT_REJECT  = "color: #CC3333;"
_DOT_ONLINE  = "color: #22CC55;"
_DOT_OFFLINE = "color: #888888;"


class CameraWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Camera Node — Detection Monitor")
        self.setMinimumSize(680, 580)

        self._worker = VideoWorker()
        self._worker.frame_ready.connect(self._update_frame)
        self._worker.detection_ready.connect(self._update_detection)
        self._cam_thread: threading.Thread | None = None

        self._bridge: CameraSpbBridge | None = None
        self._bridge_thread: threading.Thread | None = None
        self._tuner: ColorTunerDialog | None = None

        # (label, type, index) — DepthAI first, then any detected webcams
        self._cameras: list[tuple[str, str, int]] = (
            [("DepthAI", "depthai", -1)]
            + [(f"Webcam {i}", "webcam", i) for i in enumerate_webcams()]
        )

        self._build_ui()

        self._poll = QTimer(self)
        self._poll.timeout.connect(self._refresh_spb_status)
        self._poll.start(1000)

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setSpacing(8)
        root.setContentsMargins(12, 12, 12, 12)

        root.addWidget(self._build_feed_panel())
        root.addWidget(self._build_detection_bar())
        root.addWidget(self._build_controls())

    def _build_feed_panel(self) -> QWidget:
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        self._video_label = QLabel()
        self._video_label.setAlignment(Qt.AlignCenter)
        self._video_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._video_label.setMinimumSize(640, 400)
        self._video_label.setText("Waiting for camera…")
        self._video_label.setStyleSheet(
            "background-color: #1a1a1a; color: #888888;"
            "border: 1px solid #333333;"
        )
        layout.addWidget(self._video_label)

        # View mode toggle
        mode_row = QHBoxLayout()
        mode_row.addStretch()
        mode_row.addWidget(QLabel("View:"))

        self._view_btn_group = QButtonGroup(self)
        self._view_btn_group.setExclusive(True)
        for label, mode in [("Annotated", "annotated"), ("Pellets", "pellet"), ("Foreign", "foreign")]:
            btn = QPushButton(label)
            btn.setCheckable(True)
            btn.setChecked(mode == "annotated")
            btn.clicked.connect(lambda _, m=mode: self._set_view_mode(m))
            self._view_btn_group.addButton(btn)
            mode_row.addWidget(btn)

        layout.addLayout(mode_row)
        return container

    def _build_detection_bar(self) -> QGroupBox:
        cfg = _load_cam_cfg()
        self._pellet_threshold = cfg.pellet_pixel_threshold
        self._foreign_threshold = cfg.foreign_pixel_threshold

        box = QGroupBox()
        layout = QVBoxLayout(box)
        layout.setContentsMargins(8, 4, 8, 4)
        layout.setSpacing(4)

        # ── Result row ────────────────────────────────────────────────
        row = QHBoxLayout()

        self._result_dot = QLabel("●")
        self._result_dot.setFont(QFont("monospace", 18))
        self._result_dot.setStyleSheet(_DOT_IDLE)
        row.addWidget(self._result_dot)

        self._result_label = QLabel("—")
        self._result_label.setFont(QFont("Sans Serif", 13, QFont.Bold))
        self._result_label.setMinimumWidth(80)
        row.addWidget(self._result_label)

        row.addStretch()

        self._circles_label = QLabel("Circles: —")
        self._circles_label.setFont(QFont("Monospace", 10))
        row.addWidget(self._circles_label)

        row.addWidget(self._sep())

        self._pellet_label = QLabel("Pellet px: —")
        self._pellet_label.setFont(QFont("Monospace", 10))
        row.addWidget(self._pellet_label)

        row.addWidget(self._sep())

        self._foreign_label = QLabel("Foreign px: —")
        self._foreign_label.setFont(QFont("Monospace", 10))
        row.addWidget(self._foreign_label)

        layout.addLayout(row)

        # ── Pellet pixel bar ──────────────────────────────────────────
        pellet_row = QHBoxLayout()
        lbl = QLabel("Pellets")
        lbl.setFixedWidth(55)
        pellet_row.addWidget(lbl)
        self._pellet_bar = QProgressBar()
        self._pellet_bar.setMaximum(max(self._pellet_threshold * 10, 10000))
        self._pellet_bar.setValue(0)
        self._pellet_bar.setTextVisible(False)
        self._pellet_bar.setMaximumHeight(12)
        self._pellet_bar.setStyleSheet(
            "QProgressBar { border: 1px solid #444; border-radius: 3px; background: #1a1a1a; }"
            "QProgressBar::chunk { background-color: #22CC55; border-radius: 3px; }"
        )
        pellet_row.addWidget(self._pellet_bar)
        layout.addLayout(pellet_row)

        # ── Foreign pixel bar ─────────────────────────────────────────
        foreign_row = QHBoxLayout()
        lbl2 = QLabel("Foreign")
        lbl2.setFixedWidth(55)
        foreign_row.addWidget(lbl2)
        self._foreign_bar = QProgressBar()
        self._foreign_bar.setMaximum(max(self._foreign_threshold * 10, 1000))
        self._foreign_bar.setValue(0)
        self._foreign_bar.setTextVisible(False)
        self._foreign_bar.setMaximumHeight(12)
        self._foreign_bar.setStyleSheet(
            "QProgressBar { border: 1px solid #444; border-radius: 3px; background: #1a1a1a; }"
            "QProgressBar::chunk { background-color: #444444; border-radius: 3px; }"
        )
        foreign_row.addWidget(self._foreign_bar)
        layout.addLayout(foreign_row)

        return box

    @staticmethod
    def _sep() -> QLabel:
        s = QLabel("|")
        s.setStyleSheet("color: #888888;")
        return s

    def _build_controls(self) -> QGroupBox:
        box = QGroupBox("Controls")
        layout = QVBoxLayout(box)
        layout.setSpacing(6)

        # ── Camera row ────────────────────────────────────────────────
        cam_row = QHBoxLayout()
        cam_lbl = QLabel("Camera")
        cam_lbl.setMinimumWidth(90)
        cam_row.addWidget(cam_lbl)

        self._cam_combo = QComboBox()
        for label, _, _ in self._cameras:
            self._cam_combo.addItem(label)
        self._cam_combo.setMinimumWidth(120)
        cam_row.addWidget(self._cam_combo)

        self._start_btn = QPushButton("Start")
        self._start_btn.setMinimumHeight(30)
        self._start_btn.clicked.connect(self._on_start)

        self._stop_btn = QPushButton("Stop")
        self._stop_btn.setMinimumHeight(30)
        self._stop_btn.clicked.connect(self._on_stop)

        self._tuner_btn = QPushButton("Colour Tuner…")
        self._tuner_btn.setMinimumHeight(30)
        self._tuner_btn.clicked.connect(self._on_tuner)

        cam_row.addWidget(self._start_btn)
        cam_row.addWidget(self._stop_btn)
        cam_row.addStretch()
        cam_row.addWidget(self._tuner_btn)
        layout.addLayout(cam_row)

        # ── Divider ───────────────────────────────────────────────────
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        layout.addWidget(line)

        # ── SPB Bridge row ────────────────────────────────────────────
        spb_row = QHBoxLayout()

        self._spb_dot = QLabel("●")
        self._spb_dot.setFont(QFont("monospace", 14))
        self._spb_dot.setStyleSheet(_DOT_OFFLINE)
        spb_row.addWidget(self._spb_dot)

        spb_lbl = QLabel("SPB Bridge")
        spb_lbl.setMinimumWidth(78)
        spb_row.addWidget(spb_lbl)

        self._broker_combo = QComboBox()
        self._broker_combo.addItem("Local MQTT",   "local")
        self._broker_combo.addItem("HiveMQ Cloud", "hivemq")
        self._broker_combo.setMinimumWidth(130)
        spb_row.addWidget(self._broker_combo)

        self._spb_launch_btn = QPushButton("Launch")
        self._spb_launch_btn.setMinimumHeight(30)
        self._spb_launch_btn.clicked.connect(self._on_spb_launch)

        self._spb_stop_btn = QPushButton("Stop")
        self._spb_stop_btn.setMinimumHeight(30)
        self._spb_stop_btn.setEnabled(False)
        self._spb_stop_btn.clicked.connect(self._on_spb_stop)

        spb_row.addWidget(self._spb_launch_btn)
        spb_row.addWidget(self._spb_stop_btn)
        spb_row.addStretch()
        layout.addLayout(spb_row)

        return box

    # ------------------------------------------------------------------
    # Camera lifecycle
    # ------------------------------------------------------------------

    def _start_camera(self):
        self._worker.stop()
        if self._cam_thread and self._cam_thread.is_alive():
            self._cam_thread.join(timeout=2.0)
        _, cam_type, cam_idx = self._cameras[self._cam_combo.currentIndex()]
        self._cam_combo.setEnabled(False)
        self._cam_thread = threading.Thread(
            target=self._worker.start,
            kwargs={"camera_type": cam_type, "webcam_index": cam_idx},
            daemon=True,
        )
        self._cam_thread.start()
        self._start_btn.setEnabled(False)
        self._stop_btn.setEnabled(True)

    def _on_start(self):
        self._start_camera()

    def _on_stop(self):
        self._worker.stop()
        self._cam_combo.setEnabled(True)
        self._start_btn.setEnabled(True)
        self._stop_btn.setEnabled(False)
        self._result_dot.setStyleSheet(_DOT_IDLE)
        self._result_label.setText("—")
        self._result_label.setStyleSheet("")

    # ------------------------------------------------------------------
    # SPB Bridge lifecycle
    # ------------------------------------------------------------------

    def _on_spb_launch(self):
        if self._bridge_thread and self._bridge_thread.is_alive():
            return
        broker_type = self._broker_combo.currentData()
        self._broker_combo.setEnabled(False)
        # Share the live VideoWorker pipeline — no second DepthAI connection.
        self._bridge = CameraSpbBridge(
            frame_provider=self._worker.next_result,
            broker_type=broker_type,
        )
        self._bridge_thread = threading.Thread(target=self._bridge.run, daemon=True)
        self._bridge_thread.start()
        self._spb_launch_btn.setEnabled(False)
        self._spb_stop_btn.setEnabled(True)
        self._spb_dot.setStyleSheet(_DOT_ONLINE)

    def _on_spb_stop(self):
        if self._bridge:
            self._bridge.stop()
            self._bridge = None
        self._broker_combo.setEnabled(True)
        self._spb_launch_btn.setEnabled(True)
        self._spb_stop_btn.setEnabled(False)
        self._spb_dot.setStyleSheet(_DOT_OFFLINE)

    def _refresh_spb_status(self):
        if self._bridge_thread and not self._bridge_thread.is_alive():
            if self._spb_stop_btn.isEnabled():
                # Thread ended unexpectedly — reset buttons.
                self._bridge = None
                self._broker_combo.setEnabled(True)
                self._spb_launch_btn.setEnabled(True)
                self._spb_stop_btn.setEnabled(False)
                self._spb_dot.setStyleSheet(_DOT_OFFLINE)

    # ------------------------------------------------------------------
    # Signal handlers
    # ------------------------------------------------------------------

    def _on_tuner(self):
        if self._tuner is None:
            self._tuner = ColorTunerDialog(self._worker, parent=self)
            self._tuner.apply_requested.connect(self._on_tuner_apply)
            self._tuner.finished.connect(self._on_tuner_closed)
        self._tuner.show()
        self._tuner.raise_()

    def _on_tuner_apply(self):
        self._start_camera()

    def _on_tuner_closed(self):
        self._tuner = None

    def _set_view_mode(self, mode: str):
        self._worker.set_view_mode(mode)

    def _update_frame(self, image):
        pix = QPixmap.fromImage(image)
        self._video_label.setPixmap(
            pix.scaled(
                self._video_label.width(),
                self._video_label.height(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
        )

    def _update_detection(self, pass_: bool, pellet_px: int, foreign_px: int, pellet_count: int):
        if pass_:
            self._result_dot.setStyleSheet(_DOT_PASS)
            self._result_label.setText("PASS")
            self._result_label.setStyleSheet(_DOT_PASS)
        else:
            self._result_dot.setStyleSheet(_DOT_REJECT)
            self._result_label.setText("REJECT")
            self._result_label.setStyleSheet(_DOT_REJECT)

        self._circles_label.setText(f"Circles: {pellet_count}")
        self._pellet_label.setText(f"Pellet px: {pellet_px:,}")
        self._foreign_label.setText(f"Foreign px: {foreign_px:,}")

        # Pellet bar
        self._pellet_bar.setValue(min(pellet_px, self._pellet_bar.maximum()))

        # Foreign bar — grey when safe, red when threshold exceeded
        self._foreign_bar.setValue(min(foreign_px, self._foreign_bar.maximum()))
        if foreign_px >= self._foreign_threshold:
            chunk_color = "#CC3333"
        else:
            chunk_color = "#444444"
        self._foreign_bar.setStyleSheet(
            "QProgressBar { border: 1px solid #444; border-radius: 3px; background: #1a1a1a; }"
            f"QProgressBar::chunk {{ background-color: {chunk_color}; border-radius: 3px; }}"
        )

        if self._bridge is not None:
            self._bridge.push_result(pass_, pellet_px, foreign_px, pellet_count)

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def closeEvent(self, event):
        self._poll.stop()
        if self._tuner:
            self._tuner.close()
        if self._bridge:
            self._bridge.stop()
        self._worker.stop()
        if self._bridge_thread and self._bridge_thread.is_alive():
            self._bridge_thread.join(timeout=2.0)
        if self._cam_thread and self._cam_thread.is_alive():
            self._cam_thread.join(timeout=2.0)
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("Camera Node")
    win = CameraWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
