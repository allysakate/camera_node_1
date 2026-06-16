"""Camera driver node — thin ROS wrapper around CameraDetector (hardware only).

No PackML, no MQTT. On a `camera/trigger` (Empty) it captures and aggregates
`frame_counter` frames from one DepthAI pipeline and publishes the outcome on
`camera/detection` (String JSON):

    {"pass": true, "pellets": 12, "pellet_px": 3400, "foreign_px": 0}   on success
    {"error": "CameraOffline"}                                          device unavailable

The bridge node owns the state machine and maps these to PackML/alarms.
"""

import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String

from .config_loader import load_config
from .camera import CameraDetector


class CameraDriverNode(Node):
    def __init__(self):
        super().__init__("camera_driver_node")
        self._cfg = load_config()
        self._detector = CameraDetector(self._cfg)
        self._n = self._cfg.frame_counter
        self._busy = False

        self._result_pub = self.create_publisher(String, "camera/detection", 10)
        self.create_subscription(Empty, "camera/trigger", self._on_trigger, 10)

        self.get_logger().info(
            f"camera_driver_node up — {self._cfg.frame_width}x{self._cfg.frame_height}, "
            f"frame_counter={self._n}")

    def _on_trigger(self, _msg: Empty):
        if self._busy:
            self.get_logger().warn("Capture already in progress — trigger ignored")
            return
        threading.Thread(target=self._do_capture, daemon=True).start()

    def _do_capture(self):
        self._busy = True
        try:
            try:
                result = self._detector.capture_n_frames(self._n)
            except RuntimeError as exc:
                self.get_logger().warn(f"Capture failed: {exc}")
                self._publish({"error": "CameraOffline"})
                return
            self._publish(result)
        finally:
            self._busy = False

    def _publish(self, data: dict):
        m = String(); m.data = json.dumps(data)
        self._result_pub.publish(m)
        self.get_logger().info(f"detection: {m.data}")


def main(args=None):
    rclpy.init(args=args)
    node = CameraDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
