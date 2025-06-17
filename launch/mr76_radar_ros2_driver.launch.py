#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  serial2can_node = Node(
      package='mr76_radar_ros2_driver',
      executable='serial2can_node',
      name='serial2can',
      output='screen',
      parameters=[
        {'port_name': '/dev/ttyUSB0'},
        {'port_baudrate': 230400}
      ]
  )
  
  mr76_radar_node = Node(
      package='mr76_radar_ros2_driver',
      executable='mr76_radar_ros2_driver_node',
      name='mr76_radar',
      output='screen',
      parameters=[
        {'can_messages_topic_name': 'can_frame_messages'}
      ]
  )

  # Define LaunchDescription variable
  ld = LaunchDescription()

  ld.add_action(serial2can_node)
  ld.add_action(mr76_radar_node)

  return ld