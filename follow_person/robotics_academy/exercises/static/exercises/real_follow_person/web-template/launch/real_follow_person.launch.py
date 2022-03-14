import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node



def generate_launch_description():
	
	v4l2_camera_node = ExecuteProcess(
	    cmd=['ros2', 'run', 'v4l2_camera', 'v4l2_camera_node', '--ros-args', '-p', 'video_device:="/dev/video4"'], output='screen')

	ld = LaunchDescription()
	
	ld.add_action(v4l2_camera_node)
	
	return ld
