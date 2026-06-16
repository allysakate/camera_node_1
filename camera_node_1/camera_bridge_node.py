"""Camera bridge node — PackML + OMAC authority + Sparkplug B.

Subclasses RosSpbBridgeBase. Owns no hardware: a cycle is run by triggering
the camera_driver_node over ROS (`camera/trigger`) and awaiting its
`camera/detection`. Maps driver errors to alarms, publishes the result on
`camera/result` for the panel, and bridges to Sparkplug B for SCADA/AUTO mode.
"""

import json
import time

import rclpy
from std_msgs.msg import String

from spb_node_common.ros_bridge_base import (
    RosSpbBridgeBase, MetricDataType, addMetric,
)
from .config_loader import load_config

ALARM_DEFINITIONS = {
    8001: (1, "CameraOffline"),
    8002: (2, "DetectionTimeout"),
    8003: (1, "PrimaryHostOffline"),
}


class CameraBridgeNode(RosSpbBridgeBase):
    ALARM_DEFINITIONS = ALARM_DEFINITIONS
    ROS_NS          = "camera"
    RESULT_TOPIC    = "camera/result"
    DETECTION_TOPIC = "camera/detection"
    TIMEOUT_ALARM   = 8002                        # DetectionTimeout
    ERROR_ALARMS    = {"CameraOffline": 8001, "DetectionTimeout": 8002}

    def __init__(self):
        cfg = load_config()
        super().__init__("camera_bridge_node", cfg)

    def _cycle_timeout_s(self) -> float:
        return float(self._cfg.detection_timeout_s)

    def _handle_result(self, data: dict):
        pass_   = bool(data.get("pass", False))
        pellets = int(data.get("pellets", 0))
        pellet_px  = int(data.get("pellet_px", 0))
        foreign_px = int(data.get("foreign_px", 0))

        # Result for the panel (compact: matches the panel's camera card fields).
        m = String()
        m.data = json.dumps({"pass": pass_, "pellets": pellets, "foreign_px": foreign_px})
        self._result_pub.publish(m)

        self._publish_ddata({
            self._m("Result/Last/Pass"):              (MetricDataType.Boolean, pass_),
            self._m("Result/Last/PelletCount"):       (MetricDataType.Int32,   pellets),
            self._m("Result/Last/PelletPixelCount"):  (MetricDataType.Int32,   pellet_px),
            self._m("Result/Last/ForeignPixelCount"): (MetricDataType.Int32,   foreign_px),
            self._m("Result/Last/TimestampMs"):       (MetricDataType.Int64,   int(time.time() * 1000)),
        })

    def _publish_extra_birth_metrics(self, payload):
        addMetric(payload, self._m("Result/Last/Pass"),              None, MetricDataType.Boolean, False)
        addMetric(payload, self._m("Result/Last/PelletCount"),       None, MetricDataType.Int32,   0)
        addMetric(payload, self._m("Result/Last/PelletPixelCount"),  None, MetricDataType.Int32,   0)
        addMetric(payload, self._m("Result/Last/ForeignPixelCount"), None, MetricDataType.Int32,   0)
        addMetric(payload, self._m("Result/Last/TimestampMs"),       None, MetricDataType.Int64,   0)


def main(args=None):
    rclpy.init(args=args)
    node = CameraBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
