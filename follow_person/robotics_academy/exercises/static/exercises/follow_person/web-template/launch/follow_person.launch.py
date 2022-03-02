import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
	
	# -- Loading Hospital World Launcher that includes the autonomous person model
	pkg_hospital_world = FindPackageShare(package='hospital_world').find('hospital_world')
	
	hospital_world_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(os.path.join(pkg_hospital_world, 'launch', 'hospital_follow_person.launch.py'))
	)
	
	# -- Loading Darknet ROS
	pkg_darknet_ros = FindPackageShare(package='darknet_ros').find('darknet_ros')
	
	darknet_ros_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(os.path.join(pkg_darknet_ros, 'launch', 'darknet_ros.launch.py'))
	)

	ld = LaunchDescription()
	
	ld.add_action(hospital_world_launch)
	#ld.add_action(darknet_ros_launch)
	
	return ld
