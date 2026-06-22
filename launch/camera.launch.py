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
    broker_override = context.launch_configurations.get("broker_type", "").strip()

    from config_loader import load_config
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

    return [
        Node(package="camera_node_1", executable="camera_driver_node", output="screen"),
        Node(package="camera_node_1", executable="camera_bridge_node", output="screen",
             parameters=[bridge_params]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "broker_type", default_value="",
            description='MQTT broker: hivemq | local | "" (use camera_params.yaml)'),
        OpaqueFunction(function=_setup),
    ])
