"""Standalone camera launch — driver + bridge.

Makes camera_node_1 individually usable:
    ros2 launch camera_node_1 camera.launch.py
    ros2 launch camera_node_1 camera.launch.py broker_type:=hivemq

Broker connectivity is non-blocking; with no broker reachable the device is
still fully controllable over ROS (camera/cmd, camera/mode).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    broker_override  = context.launch_configurations.get("broker_type",  "").strip()
    camera_type      = context.launch_configurations.get("camera_type",  "webcam").strip()
    webcam_index_str = context.launch_configurations.get("webcam_index", "0").strip()

    from camera_node_1.config_loader import load_config
    cfg = load_config()
    if broker_override == "hivemq":
        broker = cfg.hivemq_broker
    elif broker_override == "local":
        broker = cfg.local_broker
    else:
        broker = cfg.active_broker()

    bridge_params = {
        "mqtt_host": broker.host,
        "mqtt_port": broker.port,
        "use_tls":   broker.use_tls,
    }
    if broker.username:
        bridge_params["mqtt_username"] = broker.username
        bridge_params["mqtt_password"] = broker.password

    driver_params = {"camera_type": camera_type}
    if camera_type == "webcam":
        driver_params["webcam_index"] = int(webcam_index_str)

    return [
        Node(package="camera_node_1", executable="camera_driver_node", output="screen",
             parameters=[driver_params]),
        Node(package="camera_node_1", executable="camera_bridge_node", output="screen",
             parameters=[bridge_params]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "broker_type", default_value="",
            description='MQTT broker: hivemq | local | "" (use camera_params.yaml)'),
        DeclareLaunchArgument(
            "camera_type", default_value="webcam",
            description='Camera type: webcam | depthai'),
        DeclareLaunchArgument(
            "webcam_index", default_value="0",
            description='Webcam device index (for camera_type:=webcam)'),
        OpaqueFunction(function=_setup),
    ])
