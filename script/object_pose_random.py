import random
import os
import time
import copy
import csv

import numpy as np
import PyKDL as kdl
import cv2

import rospy
import rospkg

import tf_conversions.posemath as pm
from gazebo_msgs.srv import *
from gazebo_msgs.msg import *
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Empty
from cv_bridge import CvBridge, CvBridgeError

import python_pcd

from gazebo_scene_customizer import *
from utility import *
from threading import Lock

class ObjectPoseRandom(object):
	"""Class that randomly change objects and camera pose"""
	def __init__(self):
		self.target_position_ = np.array([0., 0., 0.])

		self.obj_xyz_max_ = np.array([0.1 , 0.125,0.3])
		self.obj_xyz_min_ = np.array([-0.1,-0.125,0.1])
		self.obj_delta_xyz_ = self.obj_xyz_max_ - self.obj_xyz_min_

		self.camera_xyz_max_ = np.array([2.0, 0.7,1.0])
		self.camera_xyz_min_ = np.array([0.75,-0.7,0.05])
		self.camera_delta_xyz_ = self.camera_xyz_max_ - self.camera_xyz_min_
		self.camera_view_axis_ = np.array([1,0,0])

		# offset between camera_rgb_frame and camera_depth_optical_frame 
		# (which is where the camera pointcloud origin is)
		self.cam_rgb_to_depth_offset_ = kdl.Frame()
		self.cam_rgb_to_depth_offset_.p = kdl.Vector(0.000, 0.025, 0.000)
		self.cam_rgb_to_depth_offset_.M = kdl.Rotation.RPY(-np.pi/2, 0, -np.pi/2)

		self.camera_start_trajectory_ = np.array([0.6,-0.5,0.7])
		self.camera_end_trajectory_ = np.array([0.5,0.75,0.4])

		# self.trajectory_step_ = 300

		self.frame_counter_ = 0
		self.trajectory_counter_ = 0

		rospy.wait_for_service('/gazebo/set_model_state')
		self.set_model_pose_ = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

		rospy.wait_for_service('/gazebo/get_model_state')
		self.get_model_pose_ = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

		rospy.wait_for_service('/gazebo/set_physics_properties')
		self.set_world_physics = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)

		self.object_names_ = list()
		self.object_poses_ = dict()
		self.object_labels_ = dict()
		self.camera_name_ = None
		self.object_imposters_ = list()
		self.light_setup = dict()
		self.light_setup[0] = ['my_light1' ,'point',0.5 ,0   ,0.5, 0, 0, 0, 1]
		self.light_setup[1] = ['my_light2','point',-0.5,0   ,0.5, 0, 0, 0, 1]
		self.light_setup[2] = ['my_light3','point',0   ,-0.5,0.5, 0, 0, 0, 1]
		self.light_setup[3] = ['my_light4','point',0   ,0.5 ,0.5, 0, 0, 0, 1]

		self.bridge = CvBridge()
		self.cached_pcl_ = None
		self.cached_rgb_ = None
		self.cached_depth_ = None
		self.cached_depth_seq_ = None
		self.depth_mutex_ = Lock()
		self.rgb_mutex_ = Lock()
		self.pcl_mutex_ = Lock()

		# for setting gazebo disable collision
		self.gazebo_physics_setting_ = GazeboPhysicsSetting()
		self.gazebo_light_setting = GazeboLightSetting()
		self.make_faraway_distance_ = [15.] * 3
		self.original_depth_img_ = None


		# package_dir = rospkg.RosPack().get_path('gazebo_kinect_training')
		# data_folder_full_path = os.path.join(package_dir,'data')
		data_folder_full_path = os.path.join('/media/hanxiao/DATA','simulated')
		if not os.path.exists(data_folder_full_path):
			os.makedirs(data_folder_full_path)

		target_folder_full_path = os.path.join(data_folder_full_path,time.strftime("%d_%m_%y_%H_%M_%S"))
		
		os.makedirs(target_folder_full_path)

		self.data_pcl_dir = os.path.join(target_folder_full_path,'raw_cloud')
		self.data_rgb_dir = os.path.join(target_folder_full_path,'raw_rgb')
		# visual depth can be viewed directly as image, while raw depth is float32 image
		# self.data_depth_dir = os.path.join(target_folder_full_path,'visual_depth')
		self.data_raw_depth_dir = os.path.join(target_folder_full_path,'raw_depth')

		self.data_mask_dir = os.path.join(target_folder_full_path,'seg_mask')
		self.data_visual_mask_dir = os.path.join(target_folder_full_path,'visual_seg_mask')
		
		os.makedirs(self.data_pcl_dir)
		os.makedirs(self.data_rgb_dir)
		# os.makedirs(self.data_depth_dir)0.025, 0.000, 0.000
		os.makedirs(self.data_raw_depth_dir)
		os.makedirs(self.data_mask_dir)
		os.makedirs(self.data_visual_mask_dir)

		self.gazebo_physics_setting_.setEnablePhysics(enable = True)

		self.filewriter = open(os.path.join(target_folder_full_path,'object_poses_rel_to_camera.csv'), 'w')
		self.writer = csv.writer(self.filewriter, delimiter='\t', quoting=csv.QUOTE_NONE)
		self.writer.writerow(['Frame','Object Name','x','y','z','q.x','q.y','q.z','q.w'])

	def __del__(self):
		self.filewriter.close()

	def callbackPcl(self,data):
		self.pcl_mutex_.acquire()
		self.cached_pcl_ = data
		self.pcl_mutex_.release()

	def callbackImageRgb(self,data):
		self.rgb_mutex_.acquire()
		self.cached_rgb_ = data
		self.rgb_mutex_.release()

	def callbackImageDepth(self,data):
		self.depth_mutex_.acquire()
		self.cached_depth_ = data
		self.cached_depth_seq_ = data.header.seq
		self.depth_mutex_.release()

	def startSubscribers(self):
		rospy.Subscriber('/camera/depth/points', PointCloud2, self.callbackPcl)
		rospy.Subscriber('/camera/rgb/image_raw', Image, self.callbackImageRgb)
		rospy.Subscriber('/camera/depth/image_raw', Image, self.callbackImageDepth)
	
	def saveRawDepth(self, depth_data, filename):
		try:
			depth_array = self.msgToDepthArray(depth_data)
			self.original_depth_img_ = depth_array
			# scale from meters to 10000
			scaling_factor = 10000
			depth_array = depth_array*scaling_factor #scaling the image to [0;65535]
			cv2_img = np.array(depth_array,dtype=np.uint16)
		except CvBridgeError, e:
			rospy.logerr(e)
		else:
			# np.savetxt('%s.txt'%filename,depth_array,delimiter=',')
			# Save your OpenCV2 image as a png
			cv2.imwrite('%s.png'%(filename), cv2_img)

	def msgToDepthArray(self, msg, min_distance = 0.0, max_distance = 8.0):
		image = self.bridge.imgmsg_to_cv2(msg,"passthrough")
		depth_array = np.array(image, dtype=np.float32)
		depth_array = np.nan_to_num(depth_array)
		depth_array = np.clip(depth_array, 0. ,8.)
		return depth_array

	def getSaveableImageFromMessage(self,msg,img_type):
		# Convert ROS Image message to OpenCV2
		cv2_img = None
		if img_type == '32FC1': 
			depth_array = self.msgToDepthArray(msg)
			
			# scale from meters to 10000
			scaling_factor = 255/8.
			# scaling_factor = 10000
			depth_array = depth_array*scaling_factor
			cv2_img = np.array(depth_array,dtype=np.uint8)
		else:
			cv2_img = self.bridge.imgmsg_to_cv2(msg, img_type)

		return cv2_img

	def saveImgFromNumpy(self,data,filename):
		cv2.imwrite('%s.png'%(filename), data)

	def saveImgFromMessage(self,msg,filename,img_type):
		try:
			cv2_img = self.getSaveableImageFromMessage(msg,img_type)
			# if img_type == '32FC1':
			# 	self.original_depth_img_ = cv2_img.astype(dtype = np.uint16)
		except CvBridgeError, e:
			rospy.logerr(e)
		else:
			# Save your OpenCV2 image as a png
			cv2.imwrite('%s.png'%(filename), cv2_img)

	def setCameraPoseRandom(self):
		random_xyz_position = np.random.rand(3)*self.camera_delta_xyz_ + self.camera_xyz_min_

		kdl_frame = generateCameraPose(self.target_position_, random_xyz_position, self.camera_view_axis_)
		self.setCameraPosition(kdl_frame)

	def setCameraPosition(self, kdl_frame):
		if self.translateObject(self.camera_name_,kdl_frame):
			return True
		else:
			rospy.logerr("Fail to set the %s pose"%self.camera_name_)

	def setObjectPoseRandom(self,object_name):
		random_xyz_position = np.random.rand(3)*self.obj_delta_xyz_ + self.obj_xyz_min_
		random_rpy = 2 * np.pi * np.random.rand(3)
		kdl_frame = kdl.Frame()
		kdl_frame.p = kdl.Vector(*random_xyz_position.tolist())
		kdl_frame.M = kdl.Rotation.RPY(*random_rpy.tolist())

		if self.translateObject(object_name,kdl_frame, recheck_pose = False):
			return True
		else:
			rospy.logerr("Fail to set the %s pose"%object_name)

	def setLightPoseRandom(self):
		for key in self.light_setup:
			light_config = self.light_setup[key]
			light_config[4] = np.random.rand()
			self.gazebo_light_setting.setPointSourcePose(*light_config)

	def getObjectPoses(self, relative_frame = ''):
		success = True
		res = self.get_model_pose_(model_name = self.camera_name_)
		if res.success:
			self.object_poses_[self.camera_name_] = pm.fromMsg(res.pose)
		else:
			rospy.logerr(res.status_message)
			success = False

		for object_name in self.object_names_:
			res = self.get_model_pose_(model_name = object_name)
			if res.success:
				self.object_poses_[object_name] = pm.fromMsg(res.pose)
			else:
				rospy.logerr(res.status_message)
				success = False
		return success

	def checkObjectPoseInTarget(self, object_name, target_pose):
		rospy.loginfo('Checking if %s pose has reached the target pose'%object_name)
		res = self.get_model_pose_(model_name = object_name)
		res_pose = pm.fromMsg(res.pose)
		if res.success:
			return kdl.Equal(target_pose, res_pose , 0.001)
		else:
			rospy.loginfo('%s is not the same as %s'%(str(res_pose.p),str(target_pose.p)))
		return False

	def waitForData(self, data_type, data_name = '', min_seq = 0, get_msg = False):
		data_content = None
		have_data = False

		while not have_data:
			if data_type == 0:
				self.pcl_mutex_.acquire()
				if self.cached_pcl_ is not None:
					data_content = self.cached_pcl_
					have_data = True
				self.pcl_mutex_.release()
			elif data_type == 1:
				self.rgb_mutex_.acquire()
				if self.cached_rgb_ is not None:
					msg_data = self.cached_rgb_
					data_content = self.getSaveableImageFromMessage(msg_data,'bgr8')
					# make sure the image is not empty/blank
					have_data = np.any(data_content)
				self.rgb_mutex_.release()
			else:
				self.depth_mutex_.acquire()
				if self.cached_depth_ is not None and self.cached_depth_ > min_seq:
					msg_data = self.cached_depth_
					data_content = self.getSaveableImageFromMessage(msg_data,'32FC1')
					# make sure the image is not empty/blank
					have_data = np.any(data_content)
				self.depth_mutex_.release()
			rospy.loginfo('Waiting for %s data...'%data_name)
			rospy.sleep(0.005)
		if get_msg:
			return msg_data, data_content
		else:
			return data_content

	def saveRawData(self):
		# This will save point cloud, depth, and rgb image

		filename = '{:05d}'.format(self.frame_counter_)
		# flush the cache to make sure the data is new

		self.pcl_mutex_.acquire()
		self.cached_pcl_ = None
		self.pcl_mutex_.release()
		self.depth_mutex_.acquire()
		self.cached_depth_ = None
		self.original_depth_img_ = None
		self.depth_mutex_.release()
		self.rgb_mutex_.acquire()
		self.cached_rgb_ = None
		self.rgb_mutex_.release()

		pcl_data = self.waitForData(0, data_name = 'point cloud')
		self.pcl_mutex_.acquire()
		# save point cloud
		pcd_location = os.path.join(self.data_pcl_dir,'cloud_%s.pcd'%filename)
		python_pcd.write_pcd(pcd_location, pcl_data)
		self.pcl_mutex_.release()


		depth_msg, _ = self.waitForData(2, data_name = 'depth', get_msg = True)
		# depth_location = os.path.join(self.data_depth_dir,'depth_%s'%(filename))
		# self.saveImgFromNumpy(depth_data,depth_location)
		raw_depth_location = os.path.join(self.data_raw_depth_dir,'seq_depth%s'%(filename))
		self.saveRawDepth(depth_msg,raw_depth_location)

		rgb_data = self.waitForData(1, data_name = 'rgb')
		rgb_location = os.path.join(self.data_rgb_dir,'seq_color%s'%(filename))
		self.saveImgFromNumpy(rgb_data,rgb_location)
		
		rospy.loginfo('Saved pcd, depth, and rgb images at frame %d'%self.frame_counter_)
		return True

	def setCameraName(self,camera_name):
		self.camera_name_ = camera_name

	def setObjectNames(self,object_list, object_labels):
		self.object_names_ = object_list
		self.object_labels_ = object_labels

	def setObjectImposters(self, object_imposters):
		self.object_imposters_ = object_imposters

	def captureFrame(self):
		# Disable physics engine to ensure that the object pose will be stable
		self.gazebo_physics_setting_.setEnablePhysics(enable = False)
		# if getting updated object and camera poses successful
		if self.getObjectPoses():
			# if save data fail, do not compute segmented mask
			if self.saveRawData():
				self.saveObjectPoses()
				self.getSegmentedMask()
				self.frame_counter_ += 1

		self.gazebo_physics_setting_.setEnablePhysics(enable = True)

	def translateObjectFarAway(self,object_name, to_original_pose = False):
		# deep copy?
		kdl_frame = copy.deepcopy(self.object_poses_[object_name])
		if not to_original_pose:
			kdl_frame.p = kdl_frame.p + kdl.Vector(*self.make_faraway_distance_)

		if self.translateObject(object_name,kdl_frame):
			return True
		else:
			rospy.logerr("Fail to set the %s pose"%object_name)
			return False

	def translateObject(self,object_name, target_pose, recheck_pose = True):
		new_model_state = ModelState()
		new_model_state.model_name = object_name
		new_model_state.pose = pm.toMsg(target_pose)
		resp = self.set_model_pose_(model_state=new_model_state)

		counter = 0
		while not resp.success and counter < 10:
			resp = self.set_model_pose_(model_state=new_model_state)
			counter += 1
			rospy.sleep(0.015)
			return False

		# Revalidate the object pose after moving it
		if recheck_pose:
			counter = 0
			rospy.sleep(0.02)
			while not self.checkObjectPoseInTarget(object_name,target_pose):
				rospy.sleep(0.015)
				counter += 1
				if counter > 10:
					return False
		return True

	def getSegmentedMask(self):

		if self.original_depth_img_ is None:
			return

		# filename = 'frame_%d'%(self.frame_counter_)
		# save_mask_location = os.path.join(self.data_mask_dir,'seg_mask_%s_original_depth'%(filename))
		# cv2.imwrite('%s.png'%(save_mask_location), self.original_depth_img_.astype(dtype = np.uint8))
		
		# make zero depth to a value outside of 255 to help ignore it
		# original depth image is float in meters. Make points out of the range to be high value 
		self.original_depth_img_[self.original_depth_img_ == 0] = 100

		segmented_image = np.zeros(self.original_depth_img_.shape, dtype = np.uint8)
		scaling_factor = 255/len(self.object_labels_)

		# dictionary of python
		self.translateObjectFarAway(self.camera_name_)
		rospy.sleep(0.05)

		for object_name in self.object_names_:
			# MAKE SURE THE OBJECT IS TRANSLATED FAR AWAY
			translated = self.translateObjectFarAway(object_name)

			# make sure the cached depth image is fresh
			self.depth_mutex_.acquire()
			target_seq = self.cached_depth_seq_ + 5
			self.cached_depth_ = None
			self.depth_mutex_.release()
			rospy.sleep(0.03)
			
			depth_msg, _ = self.waitForData(2, 'isolated %s depth image'%object_name, min_seq = target_seq, get_msg = True)
			obj_depth_data = self.msgToDepthArray(depth_msg)
			# obj_depth_mask = self.getSaveableImageFromMessage(depth_data,"32FC1")

			# obj_seg_mask = np.equal(self.original_depth_img_, obj_depth_mask)
			# within 5 pixel depth difference is okay since we only have the object in obj_depth_mask

			# depth data is float in meters. Make max error to be 0.6cm == 0.006 meters
			obj_seg_mask = np.isclose(self.original_depth_img_, obj_depth_data, atol = 0.02)
			segmented_image += obj_seg_mask * self.object_labels_[object_name]

			self.translateObjectFarAway(object_name, to_original_pose = True)
			# rospy.sleep(0.015)
			self.depth_mutex_.acquire()
			self.cached_depth_ = None
			self.depth_mutex_.release()
			
			# obj_depth_mask_tmp = obj_depth_mask
			# obj_depth_mask_tmp = obj_seg_mask * scaling_factor * self.object_labels_[object_name]
			# filename = 'frame_%d'%(self.frame_counter_)
			# save_mask_location = os.path.join(self.data_mask_dir,'seg_mask_%s_%s'%(filename,object_name))
			# cv2.imwrite('%s.png'%(save_mask_location), obj_depth_mask_tmp.astype(dtype = np.uint8))
			
		self.translateObjectFarAway(self.camera_name_, to_original_pose = True)

		visual_segmented_image = segmented_image*scaling_factor
		
		filename = '{:05d}'.format(self.frame_counter_)
		mask_location = os.path.join(self.data_mask_dir,'seq_mask%s'%filename)
		visual_mask_location = os.path.join(self.data_visual_mask_dir,'visual_seq_mask%s'%filename)
		
		self.saveImgFromNumpy(visual_segmented_image.astype(dtype = np.uint8), visual_mask_location)
		self.saveImgFromNumpy(segmented_image.astype(dtype = np.uint8), mask_location)

		rospy.loginfo('Saved segmented mask at frame %d'%self.frame_counter_)
		rospy.sleep(0.05)
	
	def saveObjectPoses(self):

		camera_depth_optical_pose = self.object_poses_[self.camera_name_] * self.cam_rgb_to_depth_offset_
		camera_depth_opt_to_world_pose = camera_depth_optical_pose.Inverse()
		for object_name in self.object_names_:
			obj_pose = self.object_poses_[object_name]
			obj_pose_rel_to_camera = camera_depth_opt_to_world_pose * obj_pose

			q = [i for i in obj_pose_rel_to_camera.M.GetQuaternion()]
			p = [i for i in obj_pose_rel_to_camera.p]
			self.writer.writerow([self.frame_counter_, object_name] + p + q)

	def rearrangeObjects(self):
		invalid_pose = True
		while invalid_pose:
			for obj_name in self.object_names_:
				self.setObjectPoseRandom(obj_name)

			for imposter_name in self.object_imposters_:
				self.setObjectPoseRandom(imposter_name)

			# let the objects settle
			rospy.loginfo('Let objects settle on its resting position')
			rospy.sleep(2.0)
			self.getObjectPoses()
			all_pose_valid = True
			for object_name in self.object_names_:
				obj_pose = self.object_poses_[object_name]
				# check if any objects settles below the ground level (z = 0)
				if obj_pose.p[2] < -0.1:
					all_pose_valid = False
					break;
			invalid_pose = not all_pose_valid

	def rearrangeObjectWithRandomCameraPose(self, frame_to_move_obj = 5):
		# self.setCameraPoseRandom()

		if self.frame_counter_ % frame_to_move_obj == 0:
			# self.rearrangeObjects()
			pass
		
		self.captureFrame()

	def rearrangeObjectAfterCameraTrajectory(self, trajectory_step = 300):
		self.camera_trajectory_ = generateTrajectoryFrame(self.target_position_, self.camera_start_trajectory_,
			self.camera_end_trajectory_, self.camera_view_axis_, step_size = trajectory_step)
		
		self.setLightPoseRandom()
		self.setCameraPosition(self.camera_trajectory_[0])
		self.rearrangeObjects()

		# move camera with predefined trajectory
		for camera_frame in self.camera_trajectory_:
			self.setCameraPosition(camera_frame)
			self.captureFrame()
		self.trajectory_counter_ += 1
