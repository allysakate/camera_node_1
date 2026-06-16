"""Production touchscreen panel for the camera detection node (PyQt5).

Combines the live DepthAI video feed, detection results, Sparkplug B bridge,
and operator controls in a single 800×480 window optimised for a 7" HDMI touch
display. Replaces camera_gui.py for production deployment.

Hardware required: DepthAI USB camera + HDMI/DSI touchscreen.
No GPIO needed — all controls are on-screen.

Usage:
    python3 camera_panel.py
    python3 camera_panel.py --broker hivemq
    CAMERA_NODE_CONFIG=/path/to/camera_params.yaml python3 camera_panel.py

Broker override precedence:
    1. --broker flag
    2. CAMERA_BROKER_TYPE environment variable
    3. camera_params.yaml broker_type field
"""

import argparse
import os
import platform
import sys
import threading

# ── Import cv2-dependent modules FIRST so cv2 can set its env vars, then we ──
# ── override QT_QPA_PLATFORM_PLUGIN_PATH with the system Qt5 path before    ──
# ── QApplication() reads it.                                                 ──
from camera import VideoWorker
from camera_spb_node import (
    CameraSpbBridge,
    CMD_START, CMD_RESET, CMD_STOP, CMD_CLEAR,
)

_arch = platform.machine()
_qt_plugin_dir = f"/usr/lib/{_arch}-linux-gnu/qt5/plugins"
if os.path.isdir(_qt_plugin_dir):
    os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = _qt_plugin_dir

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton,
    QHBoxLayout, QVBoxLayout, QFrame, QSizePolicy,
)
from PyQt5.QtCore import Qt, QTimer, QObject, pyqtSignal
from PyQt5.QtGui import QFont, QPixmap

# ── Colour palette ─────────────────────────────────────────────────────────

_C = {
    'bg':        '#0d1117',
    'card':      '#161b22',
    'border':    '#30363d',
    'text':      '#e6edf3',
    'dim':       '#8b949e',
    # PackML states
    'idle':      '#1a7f3c',
    'execute':   '#9a6700',
    'complete':  '#1f6feb',
    'aborted':   '#da3633',
    # Detection result
    'pass_':     '#15803d',
    'reject':    '#b91c1c',
    # Buttons
    'btn_start': '#1a7f3c',
    'btn_reset': '#1f6feb',
    'btn_clear': '#6d28d9',
    'btn_dis':   '#21262d',
}

_STATE_COLOR = {
    'Idle':     _C['idle'],
    'Execute':  _C['execute'],
    'Complete': _C['complete'],
    'Aborted':  _C['aborted'],
}


def _style_btn(bg: str, fg: str = _C['text'], radius: int = 8) -> str:
    pressed = _darken(bg)
    return f"""
        QPushButton {{
            background-color: {bg};
            color: {fg};
            border: none;
            border-radius: {radius}px;
        }}
        QPushButton:pressed {{
            background-color: {pressed};
        }}
        QPushButton:disabled {{
            background-color: {_C['btn_dis']};
            color: #4d5763;
        }}
    """


def _darken(hex_color: str, factor: float = 0.75) -> str:
    h = hex_color.lstrip('#')
    return '#{:02x}{:02x}{:02x}'.format(
        int(int(h[0:2], 16) * factor),
        int(int(h[2:4], 16) * factor),
        int(int(h[4:6], 16) * factor),
    )


# ── Qt signals (cross-thread: bridge/worker → UI thread) ──────────────────

class _Signals(QObject):
    state_changed  = pyqtSignal(str)   # PackML state name
    broker_changed = pyqtSignal(bool)  # True = MQTT connected
    alarm_changed  = pyqtSignal(int)   # active alarm count


# ── Observable bridge subclass ────────────────────────────────────────────

class _ObservableBridge(CameraSpbBridge):
    """CameraSpbBridge that emits Qt signals on state/connection changes."""

    def __init__(self, signals: _Signals, **kwargs):
        self._panel_signals = signals
        super().__init__(**kwargs)

    # Overrides — call super first, then emit signal

    def _set_state(self, new_state: str):
        super()._set_state(new_state)
        self._panel_signals.state_changed.emit(new_state)

    def _raise_alarm(self, code: int):
        super()._raise_alarm(code)
        self._panel_signals.alarm_changed.emit(self._active_alarm_count())

    def _clear_alarm(self, code: int):
        super()._clear_alarm(code)
        self._panel_signals.alarm_changed.emit(self._active_alarm_count())

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        super()._on_mqtt_connect(client, userdata, flags, rc)
        self._panel_signals.broker_changed.emit(rc == 0)

    def _on_mqtt_disconnect(self, client, userdata, rc):
        super()._on_mqtt_disconnect(client, userdata, rc)
        self._panel_signals.broker_changed.emit(False)

    # Public command API (called from UI thread)
    def cmd_start(self): self._execute_cntrl_cmd(CMD_START)
    def cmd_reset(self): self._execute_cntrl_cmd(CMD_RESET)
    def cmd_clear(self): self._execute_cntrl_cmd(CMD_CLEAR)
    def cmd_stop(self):  self._execute_cntrl_cmd(CMD_STOP)


# ── Main window ───────────────────────────────────────────────────────────

class PanelWindow(QMainWindow):

    def __init__(self, broker_type: str = '', fullscreen: bool = True,
                 width: int = 800, height: int = 480):
        super().__init__()
        self._fullscreen  = fullscreen
        self._win_w       = width
        self._win_h       = height

        # Runtime state (written from signal handlers — UI thread only)
        self._state        = 'Idle'
        self._broker_ok    = False
        self._alarm_count  = 0

        # Qt signals
        self._sig = _Signals()

        # VideoWorker — live DepthAI pipeline
        self._worker = VideoWorker()
        self._worker.frame_ready.connect(self._on_frame)
        self._worker.detection_ready.connect(self._on_detection)

        # Bridge — shares the live pipeline via worker.next_result
        self._bridge = _ObservableBridge(
            signals=self._sig,
            frame_provider=self._worker.next_result,
            broker_type=broker_type or None,
        )

        # Connect bridge signals → UI
        self._sig.state_changed.connect(self._on_state_change)
        self._sig.broker_changed.connect(self._on_broker_change)
        self._sig.alarm_changed.connect(self._on_alarm_change)

        self._build_window()

        # Start background threads
        self._worker_thread = threading.Thread(
            target=self._worker.start,
            kwargs={'camera_type': 'depthai'},
            daemon=True,
        )
        self._bridge_thread = threading.Thread(
            target=self._bridge.run,
            daemon=True,
        )
        self._worker_thread.start()
        self._bridge_thread.start()

        # Clock
        self._clock_timer = QTimer(self)
        self._clock_timer.setInterval(1000)
        self._clock_timer.timeout.connect(self._tick_clock)
        self._clock_timer.start()

        self._refresh_buttons()

    # ── Window construction ───────────────────────────────────────────────

    def _build_window(self):
        self.setWindowTitle('Camera Detection Panel')
        self.setFixedSize(self._win_w, self._win_h)
        if self._fullscreen:
            self.showFullScreen()

        self.setStyleSheet(f"""
            QMainWindow, QWidget {{
                background-color: {_C['bg']};
                color: {_C['text']};
            }}
        """)

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        root.addWidget(self._build_header())
        root.addWidget(self._hline())
        root.addWidget(self._build_main(), stretch=1)

    def _hline(self) -> QFrame:
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFixedHeight(1)
        line.setStyleSheet(f'background-color: {_C["border"]};')
        return line

    def _vline(self) -> QFrame:
        line = QFrame()
        line.setFrameShape(QFrame.VLine)
        line.setFixedWidth(1)
        line.setStyleSheet(f'background-color: {_C["border"]};')
        return line

    # ── Header ────────────────────────────────────────────────────────────

    def _build_header(self) -> QWidget:
        hdr = QWidget()
        hdr.setFixedHeight(52)
        hdr.setStyleSheet(f'background-color: {_C["card"]};')
        lay = QHBoxLayout(hdr)
        lay.setContentsMargins(16, 0, 12, 0)
        lay.setSpacing(10)

        title = QLabel('CAMERA DETECTION')
        title.setFont(QFont('Arial', 13, QFont.Bold))
        lay.addWidget(title)

        lay.addStretch()

        # Broker status dot
        self._lbl_broker = QLabel('● OFFLINE')
        self._lbl_broker.setFont(QFont('Arial', 10))
        self._lbl_broker.setStyleSheet(f'color: {_C["dim"]};')
        lay.addWidget(self._lbl_broker)

        sep = QLabel('|')
        sep.setStyleSheet(f'color: {_C["border"]};')
        lay.addWidget(sep)

        # Alarm count
        self._lbl_alarms = QLabel('0 alarms')
        self._lbl_alarms.setFont(QFont('Arial', 10))
        self._lbl_alarms.setStyleSheet(f'color: {_C["dim"]};')
        lay.addWidget(self._lbl_alarms)

        sep2 = QLabel('|')
        sep2.setStyleSheet(f'color: {_C["border"]};')
        lay.addWidget(sep2)

        # State pill
        self._lbl_pill = QLabel('● IDLE')
        self._lbl_pill.setFont(QFont('Arial', 10, QFont.Bold))
        self._lbl_pill.setAlignment(Qt.AlignCenter)
        self._lbl_pill.setFixedHeight(28)
        self._lbl_pill.setMinimumWidth(90)
        self._lbl_pill.setStyleSheet(
            f'background-color: {_C["idle"]}; color: {_C["text"]}; '
            f'border-radius: 5px; padding: 0 8px;'
        )
        lay.addWidget(self._lbl_pill)

        sep3 = QLabel('|')
        sep3.setStyleSheet(f'color: {_C["border"]};')
        lay.addWidget(sep3)

        # Clock
        self._lbl_clock = QLabel('00:00:00')
        self._lbl_clock.setFont(QFont('Arial', 10))
        self._lbl_clock.setStyleSheet(f'color: {_C["dim"]};')
        self._lbl_clock.setFixedWidth(68)
        self._lbl_clock.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        lay.addWidget(self._lbl_clock)

        return hdr

    # ── Main area ─────────────────────────────────────────────────────────

    def _build_main(self) -> QWidget:
        main = QWidget()
        lay = QHBoxLayout(main)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setSpacing(8)

        lay.addWidget(self._build_video(), stretch=1)
        lay.addWidget(self._vline())
        lay.addWidget(self._build_controls())
        return main

    def _build_video(self) -> QWidget:
        container = QWidget()
        container.setStyleSheet('background: transparent;')
        lay = QVBoxLayout(container)
        lay.setContentsMargins(0, 0, 0, 0)

        self._video_label = QLabel('Waiting for camera…')
        self._video_label.setAlignment(Qt.AlignCenter)
        self._video_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._video_label.setStyleSheet(
            f'background-color: {_C["card"]}; color: {_C["dim"]}; '
            f'border-radius: 6px;'
        )
        lay.addWidget(self._video_label)
        return container

    def _build_controls(self) -> QWidget:
        ctrl = QWidget()
        ctrl.setFixedWidth(310)
        ctrl.setStyleSheet('background: transparent;')
        lay = QVBoxLayout(ctrl)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(8)

        lay.addWidget(self._build_result_card(), stretch=1)
        lay.addWidget(self._hline())
        lay.addWidget(self._build_buttons())
        return ctrl

    def _build_result_card(self) -> QWidget:
        card = QWidget()
        card.setStyleSheet(
            f'background-color: {_C["card"]}; border-radius: 8px;'
        )
        lay = QVBoxLayout(card)
        lay.setContentsMargins(12, 10, 12, 10)
        lay.setSpacing(4)

        lbl_title = QLabel('LAST RESULT')
        lbl_title.setFont(QFont('Arial', 9))
        lbl_title.setStyleSheet(f'color: {_C["dim"]}; background: transparent;')
        lbl_title.setAlignment(Qt.AlignCenter)
        lay.addWidget(lbl_title)

        self._lbl_result = QLabel('—')
        self._lbl_result.setFont(QFont('Arial', 44, QFont.Bold))
        self._lbl_result.setAlignment(Qt.AlignCenter)
        self._lbl_result.setStyleSheet(f'color: {_C["dim"]}; background: transparent;')
        lay.addWidget(self._lbl_result, stretch=1)

        self._lbl_pellet = QLabel('Pellet: — px  ·  — circles')
        self._lbl_pellet.setFont(QFont('Arial', 10))
        self._lbl_pellet.setAlignment(Qt.AlignCenter)
        self._lbl_pellet.setStyleSheet(f'color: {_C["dim"]}; background: transparent;')
        lay.addWidget(self._lbl_pellet)

        self._lbl_foreign = QLabel('Foreign: — px')
        self._lbl_foreign.setFont(QFont('Arial', 10))
        self._lbl_foreign.setAlignment(Qt.AlignCenter)
        self._lbl_foreign.setStyleSheet(f'color: {_C["dim"]}; background: transparent;')
        lay.addWidget(self._lbl_foreign)

        return card

    def _build_buttons(self) -> QWidget:
        btns = QWidget()
        btns.setStyleSheet('background: transparent;')
        lay = QVBoxLayout(btns)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(8)

        self._btn_start = QPushButton('CYCLE START')
        self._btn_start.setFont(QFont('Arial', 16, QFont.Bold))
        self._btn_start.setFixedHeight(80)
        self._btn_start.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self._btn_start.clicked.connect(self._on_start)
        lay.addWidget(self._btn_start)

        lower = QWidget()
        lower.setStyleSheet('background: transparent;')
        lower_lay = QHBoxLayout(lower)
        lower_lay.setContentsMargins(0, 0, 0, 0)
        lower_lay.setSpacing(8)

        self._btn_reset = QPushButton('RESET')
        self._btn_reset.setFont(QFont('Arial', 14, QFont.Bold))
        self._btn_reset.setFixedHeight(80)
        self._btn_reset.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self._btn_reset.clicked.connect(self._on_reset)

        self._btn_clear = QPushButton('CLEAR\nALARMS')
        self._btn_clear.setFont(QFont('Arial', 13, QFont.Bold))
        self._btn_clear.setFixedHeight(80)
        self._btn_clear.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self._btn_clear.clicked.connect(self._on_clear)

        lower_lay.addWidget(self._btn_reset)
        lower_lay.addWidget(self._btn_clear)
        lay.addWidget(lower)

        return btns

    # ── Clock ─────────────────────────────────────────────────────────────

    def _tick_clock(self):
        import datetime
        self._lbl_clock.setText(datetime.datetime.now().strftime('%H:%M:%S'))

    # ── Signal handlers (UI thread) ───────────────────────────────────────

    def _on_state_change(self, state: str):
        self._state = state
        color = _STATE_COLOR.get(state, _C['dim'])
        self._lbl_pill.setText(f'● {state.upper()}')
        self._lbl_pill.setStyleSheet(
            f'background-color: {color}; color: {_C["text"]}; '
            f'border-radius: 5px; padding: 0 8px;'
        )
        self._refresh_buttons()

    def _on_broker_change(self, connected: bool):
        self._broker_ok = connected
        if connected:
            self._lbl_broker.setText('● ONLINE')
            self._lbl_broker.setStyleSheet(f'color: {_C["idle"]};')
        else:
            self._lbl_broker.setText('● OFFLINE')
            self._lbl_broker.setStyleSheet(f'color: {_C["dim"]};')

    def _on_alarm_change(self, count: int):
        self._alarm_count = count
        if count == 0:
            self._lbl_alarms.setText('0 alarms')
            self._lbl_alarms.setStyleSheet(f'color: {_C["dim"]};')
        else:
            self._lbl_alarms.setText(f'⚠ {count} alarm{"s" if count != 1 else ""}')
            self._lbl_alarms.setStyleSheet(f'color: {_C["aborted"]};')
        self._refresh_buttons()

    def _on_frame(self, image):
        pix = QPixmap.fromImage(image)
        self._video_label.setPixmap(
            pix.scaled(
                self._video_label.width(),
                self._video_label.height(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
        )

    def _on_detection(self, pass_: bool, pellet_px: int, foreign_px: int, pellet_count: int):
        if pass_:
            self._lbl_result.setText('PASS')
            self._lbl_result.setStyleSheet(f'color: {_C["pass_"]}; background: transparent;')
        else:
            self._lbl_result.setText('REJECT')
            self._lbl_result.setStyleSheet(f'color: {_C["reject"]}; background: transparent;')

        self._lbl_pellet.setText(
            f'Pellet: {pellet_px:,} px  ·  {pellet_count} circles'
        )
        self._lbl_pellet.setStyleSheet(f'color: {_C["text"]}; background: transparent;')

        self._lbl_foreign.setText(f'Foreign: {foreign_px:,} px')
        fcolor = _C['reject'] if not pass_ else _C['text']
        self._lbl_foreign.setStyleSheet(f'color: {fcolor}; background: transparent;')

        # Forward to bridge so SCADA sees a continuous Result/Last/* stream
        self._bridge.push_result(pass_, pellet_px, foreign_px, pellet_count)

    # ── Button refresh ────────────────────────────────────────────────────

    def _refresh_buttons(self):
        s = self._state

        start_ok = (s == 'Idle' and self._alarm_count == 0)
        self._btn_start.setEnabled(start_ok)
        self._btn_start.setStyleSheet(
            _style_btn(_C['btn_start']) if start_ok
            else _style_btn(_C['btn_dis'], _C['dim'])
        )

        reset_ok = (s == 'Complete')
        self._btn_reset.setEnabled(reset_ok)
        self._btn_reset.setStyleSheet(
            _style_btn(_C['btn_reset']) if reset_ok
            else _style_btn(_C['btn_dis'], _C['dim'])
        )

        clear_ok = (s == 'Aborted')
        self._btn_clear.setEnabled(clear_ok)
        self._btn_clear.setStyleSheet(
            _style_btn(_C['btn_clear']) if clear_ok
            else _style_btn(_C['btn_dis'], _C['dim'])
        )

    # ── Button handlers ───────────────────────────────────────────────────

    def _on_start(self):
        self._bridge.cmd_start()

    def _on_reset(self):
        self._bridge.cmd_reset()

    def _on_clear(self):
        self._bridge.cmd_clear()

    # ── Cleanup ───────────────────────────────────────────────────────────

    def closeEvent(self, event):
        self._clock_timer.stop()
        self._worker.stop()
        self._bridge.stop()
        if self._worker_thread.is_alive():
            self._worker_thread.join(timeout=2.0)
        if self._bridge_thread.is_alive():
            self._bridge_thread.join(timeout=2.0)
        event.accept()


# ── Entry point ───────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='Camera detection touchscreen panel')
    parser.add_argument(
        '--broker', choices=['local', 'hivemq'], default='',
        help='Override MQTT broker (default: camera_params.yaml broker_type)',
    )
    parser.add_argument('--no-fullscreen', action='store_true',
                        help='Run in a window instead of full-screen')
    parser.add_argument('--width',  type=int, default=800)
    parser.add_argument('--height', type=int, default=480)
    args, _ = parser.parse_known_args()

    broker_type = (
        args.broker
        or os.environ.get('CAMERA_BROKER_TYPE', '')
    )
    fullscreen = not args.no_fullscreen

    app = QApplication(sys.argv)
    app.setAttribute(Qt.AA_SynthesizeTouchForUnhandledMouseEvents, True)

    window = PanelWindow(
        broker_type=broker_type,
        fullscreen=fullscreen,
        width=args.width,
        height=args.height,
    )
    if not fullscreen:
        window.show()

    ret = app.exec_()
    sys.exit(ret)


if __name__ == '__main__':
    main()
