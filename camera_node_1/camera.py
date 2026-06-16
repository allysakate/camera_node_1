"""Camera detection core for the camera node.

Two classes:
  CameraDetector  — DepthAI / webcam capture + HSV detection + circle counting.
                    No Qt dependency. Used by both camera_gui.py and camera_spb_node.py.
  VideoWorker     — Thin Qt wrapper around CameraDetector for live preview.
                    Used by camera_gui.py only.
"""

import queue as _queue
import threading
from typing import Callable, NamedTuple

import cv2
import numpy as np
import depthai as dai

from config_loader import load_config


def enumerate_webcams(max_index: int = 6) -> list[int]:
    """Return available webcam device indices (0-based) via cv2 probe.

    Uses an actual read() rather than isOpened() — on Linux, V4L2 metadata
    devices (e.g. /dev/video1, /dev/video3) open successfully but produce no
    frames, so isOpened() alone gives false positives.
    Suppresses OpenCV's V4L2 WARN/ERROR output during the probe.
    """
    available = []
    prev_level = cv2.getLogLevel()
    cv2.setLogLevel(0)
    try:
        for i in range(max_index):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                ret, _ = cap.read()
                if ret:
                    available.append(i)
            cap.release()
    finally:
        cv2.setLogLevel(prev_level)
    return available


class DetectionResult(NamedTuple):
    """Full output of one detection frame."""
    pass_: bool
    pellet_px: int
    foreign_px: int
    pellet_count: int        # discrete circular pellets found via HoughCircles
    annotated: np.ndarray    # BGR: pellets green, foreign red, circles outlined cyan
    pellet_vis: np.ndarray   # BGR: pellet mask on black (for Pellet view mode)
    foreign_vis: np.ndarray  # BGR: foreign mask on black (for Foreign view mode)
    raw: np.ndarray          # original BGR frame (for colour tuner)


class CameraDetector:
    """DepthAI / webcam capture + HSV detection + circle counting. No Qt dependency."""

    def __init__(self, cfg=None, camera_type: str = "depthai", webcam_index: int = 0):
        if cfg is None:
            cfg = load_config()
        self._camera_type  = camera_type
        self._webcam_index = webcam_index
        self._width   = cfg.frame_width
        self._height  = cfg.frame_height
        self._pellet_lower = np.array(cfg.pellet_color.lower)
        self._pellet_upper = np.array(cfg.pellet_color.upper)
        self._foreign_lower = np.array(cfg.foreign_color.lower)
        self._foreign_upper = np.array(cfg.foreign_color.upper)
        self._pellet_threshold  = cfg.pellet_pixel_threshold
        self._foreign_threshold = cfg.foreign_pixel_threshold

        # Foreign detection is disabled when both bounds are zero
        self._foreign_active = not (
            np.all(self._foreign_lower == 0) and np.all(self._foreign_upper == 0)
        )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def capture_one_frame(self) -> DetectionResult:
        """Open pipeline, grab one frame, run detection, close pipeline.

        Raises:
            RuntimeError   If the device is unavailable.
        """
        if self._camera_type == "webcam":
            cap = cv2.VideoCapture(self._webcam_index)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
            ret, bgr = cap.read()
            cap.release()
            if not ret:
                raise RuntimeError(f"Webcam {self._webcam_index} failed to capture frame")
            return self._detect(bgr)

        try:
            with dai.Pipeline() as pipeline:
                cam   = pipeline.create(dai.node.Camera).build()
                queue = cam.requestOutput(
                    (self._width, self._height)
                ).createOutputQueue()
                pipeline.start()
                frame_in = queue.get()
        except Exception as exc:
            raise RuntimeError(f"DepthAI pipeline error: {exc}") from exc

        assert isinstance(frame_in, dai.ImgFrame)
        bgr = frame_in.getCvFrame()
        return self._detect(bgr)

    def capture_stream(
        self,
        callback: Callable[["DetectionResult"], None],
        stop_event: threading.Event,
    ) -> None:
        """Continuous capture loop until stop_event is set.

        Calls callback(DetectionResult) for every frame.

        Raises:
            RuntimeError   If the device is unavailable.
        """
        if self._camera_type == "webcam":
            cap = cv2.VideoCapture(self._webcam_index)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
            if not cap.isOpened():
                raise RuntimeError(f"Webcam {self._webcam_index} unavailable")
            try:
                while not stop_event.is_set():
                    ret, bgr = cap.read()
                    if ret:
                        callback(self._detect(bgr))
            finally:
                cap.release()
            return

        try:
            with dai.Pipeline() as pipeline:
                cam   = pipeline.create(dai.node.Camera).build()
                queue = cam.requestOutput(
                    (self._width, self._height)
                ).createOutputQueue()
                pipeline.start()
                while pipeline.isRunning() and not stop_event.is_set():
                    frame_in = queue.get()
                    assert isinstance(frame_in, dai.ImgFrame)
                    bgr = frame_in.getCvFrame()
                    callback(self._detect(bgr))
        except Exception as exc:
            raise RuntimeError(f"DepthAI pipeline error: {exc}") from exc

    # ------------------------------------------------------------------
    # Detection logic
    # ------------------------------------------------------------------

    def _detect(self, bgr: np.ndarray) -> DetectionResult:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        pellet_mask = cv2.inRange(hsv, self._pellet_lower, self._pellet_upper)
        pellet_px   = cv2.countNonZero(pellet_mask)

        foreign_px   = 0
        foreign_mask = np.zeros(bgr.shape[:2], dtype=np.uint8)
        if self._foreign_active:
            foreign_mask = cv2.inRange(hsv, self._foreign_lower, self._foreign_upper)
            foreign_px   = cv2.countNonZero(foreign_mask)

        pass_ = foreign_px < self._foreign_threshold

        # Count discrete circular pellets
        pellet_count = 0
        circles = cv2.HoughCircles(
            pellet_mask,
            cv2.HOUGH_GRADIENT,
            dp=1,
            minDist=10,
            param1=50,
            param2=15,
            minRadius=3,
            maxRadius=50,
        )
        if circles is not None:
            pellet_count = len(circles[0])

        # Annotated frame: pellets → green, foreign → red, circle outlines → cyan
        annotated = bgr.copy()
        annotated[pellet_mask > 0] = [0, 255, 0]
        if self._foreign_active and foreign_px > 0:
            annotated[foreign_mask > 0] = [0, 0, 255]
        if circles is not None:
            for x, y, r in np.uint16(np.around(circles[0])):
                cv2.circle(annotated, (x, y), r, (0, 255, 255), 2)
                cv2.circle(annotated, (x, y), 2, (0, 255, 255), -1)

        # Mask visualisation frames for GUI view modes
        pellet_vis = np.zeros_like(bgr)
        pellet_vis[pellet_mask > 0] = [0, 255, 0]

        foreign_vis = np.zeros_like(bgr)
        if self._foreign_active:
            foreign_vis[foreign_mask > 0] = [0, 0, 255]

        return DetectionResult(
            pass_, pellet_px, foreign_px, pellet_count,
            annotated, pellet_vis, foreign_vis, bgr,
        )


# ---------------------------------------------------------------------------
# Qt wrapper — only import PyQt5 when this class is actually used
# ---------------------------------------------------------------------------

try:
    from PyQt5.QtCore import pyqtSignal, QObject
    from PyQt5.QtGui import QImage

    class VideoWorker(QObject):
        """Thin Qt wrapper around CameraDetector for live preview GUIs.

        Also exposes next_result() so CameraSpbBridge can share the live
        pipeline (GUI mode) instead of opening DepthAI independently.
        """

        frame_ready     = pyqtSignal(QImage)
        detection_ready = pyqtSignal(bool, int, int, int)  # pass_, pellet_px, foreign_px, pellet_count
        raw_frame_ready = pyqtSignal(object)               # np.ndarray — raw BGR for colour tuner

        def __init__(self, cfg=None):
            super().__init__()
            self._cfg = cfg
            self._stop_event = threading.Event()
            self._result_queue: _queue.Queue = _queue.Queue(maxsize=1)
            self._view_mode: str = "annotated"

        def set_view_mode(self, mode: str):
            """Switch the visualisation frame emitted on frame_ready.

            mode: "annotated" | "pellet" | "foreign"
            Thread-safe: str assignment is atomic in CPython.
            """
            self._view_mode = mode

        def start(self, camera_type: str = "depthai", webcam_index: int = 0):
            """Run the capture stream. Blocking — call from a daemon thread."""
            self._stop_event.clear()
            try:
                detector = CameraDetector(
                    self._cfg, camera_type=camera_type, webcam_index=webcam_index
                )
                detector.capture_stream(self._on_frame, self._stop_event)
            except RuntimeError as exc:
                print(f"[Camera] {exc}")

        def stop(self):
            self._stop_event.set()

        def next_result(self, timeout: float = 35.0) -> tuple[bool, int, int, int]:
            """Block until the next detection result arrives.

            Called by CameraSpbBridge when running inside camera_gui.py so the
            bridge can share the live DepthAI pipeline instead of opening its own.
            Returns (pass_, pellet_px, foreign_px, pellet_count).
            Raises queue.Empty if no frame arrives within timeout seconds.
            """
            try:
                self._result_queue.get_nowait()   # discard stale result
            except _queue.Empty:
                pass
            return self._result_queue.get(timeout=timeout)

        def _on_frame(self, result: "DetectionResult"):
            self.detection_ready.emit(
                result.pass_, result.pellet_px, result.foreign_px, result.pellet_count
            )
            try:
                self._result_queue.put_nowait((
                    result.pass_, result.pellet_px, result.foreign_px, result.pellet_count
                ))
            except _queue.Full:
                pass

            mode = self._view_mode
            if mode == "pellet":
                frame_bgr = result.pellet_vis
            elif mode == "foreign":
                frame_bgr = result.foreign_vis
            else:
                frame_bgr = result.annotated

            h, w, ch = frame_bgr.shape
            rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            img = QImage(rgb.data.tobytes(), w, h, ch * w, QImage.Format_RGB888)
            self.frame_ready.emit(img)
            self.raw_frame_ready.emit(result.raw)

except ImportError:
    # PyQt5 not available — VideoWorker is unused in the SPB node context
    pass
