#!/usr/bin/env python
import rospy
from object_pose_random import *

if __name__ == "__main__":
	rospy.init_node('gazebo_pose_randomizer')
	r = rospy.Rate(1) # 60hz
	pose_randomizer = ObjectPoseRandom()

	pose_randomizer.setCameraName('kinect')
	# object_name_list = ['drill_blue_small', 'driller_small']
	object_labels = dict()
	# object_name_list = ['node_uniform']
	# object_labels['node_uniform'] = 1
	# object_labels['link_uniform'] = 2
	object_name_list = ['drill_blue_small', 'driller_point_metal', 'driller_small', 'sander','mallet_ball_pein',
		'mallet_black_white', 'mallet_drilling', 'mallet_fiber', 'old_hammer' ]
	object_labels['drill_blue_small'] = 1
	object_labels['driller_small'] = 2
	object_labels['driller_point_metal'] = 3
	object_labels['sander'] = 4
	object_labels['mallet_ball_pein'] = 5
	object_labels['mallet_black_white'] = 6
	object_labels['mallet_drilling'] = 7
	object_labels['mallet_fiber'] = 8
	object_labels['old_hammer'] = 9

	pose_randomizer.setObjectNames(object_name_list, object_labels)
	object_imposters_list = ['c_clamp', 'keyboard', 'mug','pepsi_can','handspot']
	pose_randomizer.setObjectImposters(object_imposters_list)
	pose_randomizer.startSubscribers()

	use_camera_trajectory = False
	spawn_objects = True

	if spawn_objects:
		for obj_name in object_name_list:
			os.system("rosrun gazebo_ros spawn_model -sdf -database %s -model %s"%(obj_name,obj_name))
		for obj_name in object_imposters_list:
			os.system("rosrun gazebo_ros spawn_model -sdf -database %s -model %s"%(obj_name,obj_name))

	if use_camera_trajectory:
		while pose_randomizer.trajectory_counter_ < 50:
			pose_randomizer.rearrangeObjectAfterCameraTrajectory(trajectory_step = 100)
	else:
		counter = 0
		while counter < 1000:
			pose_randomizer.rearrangeObjectWithRandomCameraPose(frame_to_move_obj = 10)
			counter += 1
	# while not rospy.is_shutdown() and pose_randomizer.trajectory_counter_ < 3:
	# 	# pose_randomizer.saveRawData()
	# 	if use_camera_trajectory:
	# 		pose_randomizer.rearrangeObjectAfterCameraTrajectory()
	# 	else:
	# 		pose_randomizer.rearrangeObjectWithRandomCameraPose(frame_to_move_obj = 5)
	# 	# r.sleep()


