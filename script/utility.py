import numpy as np
import PyKDL as kdl
import tf_conversions.posemath as pm 

def getPolarCoordinate(vector3_origin, vector3_point):
	dV = vector3_point - vector3_origin
	x, y, z = dV
	# x = r sin theta cos psi
	# y = r sin theta sin psi
	# z = r cos theta
	r = np.linalg.norm(dV)
	theta = np.arccos(z/r) 
	psi = np.arctan2(y,x)

	return r, theta, psi

def getXYZfromPolar(vector3_origin,vector3_polar_coordinate):
	r, theta, psi = vector3_polar_coordinate
	x = r * np.sin(theta) * np.cos(psi)
	y = r * np.sin(theta) * np.sin(psi)
	z = r * np.cos(theta)

	vector3_point = vector3_origin + np.array([x,y,z])
	return vector3_point

def getTrajectoryCartesianCoordinate(vector3_origin, vector3_point1, vector3_point2, 
		step_size = 30, simulate_movement_noise = True):
	r1, theta1, psi1 = getPolarCoordinate(vector3_origin, vector3_point1)
	r2, theta2, psi2 = getPolarCoordinate(vector3_origin, vector3_point2)

	dr = r2 - r1
	dtheta = theta2 - theta1
	dpsi = psi2 - psi1

	polar_origin = np.array([r1, theta1, psi1])
	step_increment = np.array([dr, dtheta, dpsi]) / step_size

	xyz_trajectory_list = list()
	for i in xrange(step_size):
		current_polar_coordinate = polar_origin + i * step_increment
		if simulate_movement_noise:
			noise = [np.random.normal(scale = abs(step_increment[i]) / 2) for i in range(3)]
			current_polar_coordinate += np.array(noise)
		xyz_trajectory_list.append(getXYZfromPolar(vector3_origin,current_polar_coordinate))

	return xyz_trajectory_list

def generateCameraPose(vector3_origin, vector3_point, camera_view_axis, 
		camera_z_axis = np.array([0,0,1]), set_constraint_z = True):
	camera_view_vector = vector3_origin - vector3_point
	camera_view_vector /= np.linalg.norm(camera_view_vector)
	v = np.cross(camera_view_axis, camera_view_vector)

	v_x = np.array([[0    ,-v[2],v[1]],[v[2] ,0    ,-v[0]],[-v[1],v[0] ,0]])
	cosine = camera_view_axis.dot(camera_view_vector)
	if (cosine == -1): cosine = -0.99999
	rot_matrix = np.identity(3) + v_x + v_x.dot(v_x) * (1/(1+cosine))

	if set_constraint_z:
		new_x_vector = rot_matrix[:3,0]
		new_y_vector = np.cross(camera_z_axis, new_x_vector)
		new_z_vector = np.cross(new_x_vector, new_y_vector)
		rot_matrix[:3,1] = new_y_vector / np.linalg.norm(new_y_vector)
		rot_matrix[:3,2] = new_z_vector / np.linalg.norm(new_z_vector)

	np_frame = np.eye(4)
	np_frame [:3,:3] = rot_matrix
	np_frame[:3,3] = vector3_point
	kdl_frame = pm.fromMatrix(np_frame)

	return kdl_frame

def generateTrajectoryFrame(vector3_origin, vector3_point1, vector3_point2, 
		camera_view_axis, camera_z_axis = np.array([0,0,1]), set_constraint_z = True,
		step_size = 30, simulate_movement_noise = True):
	xyz_trajectory_list = getTrajectoryCartesianCoordinate(vector3_origin, vector3_point1, 
		vector3_point2, step_size , simulate_movement_noise)

	trajectory_frames = list()
	for vector3_point in xyz_trajectory_list:
		trajectory_frames.append(generateCameraPose(vector3_origin, vector3_point, camera_view_axis, 
		camera_z_axis, set_constraint_z) )
	return trajectory_frames