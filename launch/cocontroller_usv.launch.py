#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  # 无人船协控制器节点
  cocontroller_node = Node(
      package='usv_controller_ros2_driver',
      executable='usv_controller_ros2_driver_node',
      name='cocontroller_usv',
      output='screen',
      parameters=[
        {'port_name': '/dev/ttyUSB0'},
        {'port_baudrate': 115200}
      ]
  )

  # Define LaunchDescription variable
  ld = LaunchDescription()

  ld.add_action(cocontroller_node)

  return ld