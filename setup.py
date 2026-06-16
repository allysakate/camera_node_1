from setuptools import setup

package_name = 'camera_node_1'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
         ['config/camera_params.yaml', 'config/camera_secrets.example.yaml',
          'config/color_presets.yaml']),
        ('share/' + package_name + '/launch', ['launch/camera.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='DepthAI camera ROS nodes: detection driver + Sparkplug B / PackML bridge.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_driver_node = camera_node_1.camera_driver_node:main',
            'camera_bridge_node = camera_node_1.camera_bridge_node:main',
        ],
    },
)
