import numpy as np

import rospy
import ActionHandler
import project_point_cloud
import convertOpen3d
import transformations

from robostats_proj.srv import *
from std_srvs.srv import Empty

class env_utils():

  def __init__(self):
    self.curr_points = 0 # Used to calculate reward
    self.get_pc = rospy.ServiceProxy("/pc_utils/get_model_cloud", get_map)
    self.get_cam_transf = rospy.ServiceProxy("/pc_utils/get_camera_transf", get_camera_transform)
    self.reset_pc = rospy.ServiceProxy("/pc_utils/reset_pc", Empty)
    self.last_cloud = None
    self.max_no_reward = 5 # How many steps no reward can be found
    self.termination_count = 0

  def get_obs(self, im_size, max_range):
    resp_cloud = self.get_pc()
    resp_transf = self.get_cam_transf()
    open3d_cloud = convertOpen3d.convertCloudFromRosToOpen3d(resp_cloud.cloud)
    if(open3d_cloud==None or np.asarray(open3d_cloud.points).size==0):
      return self.get_obs(im_size, max_range)
    self.last_cloud = open3d_cloud
    trans =np.zeros(3)
    rot = np.zeros(4)
    trans[0] = resp_transf.tf.transform.translation.x
    trans[1] = resp_transf.tf.transform.translation.y
    trans[2] = resp_transf.tf.transform.translation.z
    rot[1] = resp_transf.tf.transform.rotation.x
    rot[2] = resp_transf.tf.transform.rotation.y
    rot[3] = resp_transf.tf.transform.rotation.z
    rot[0] = resp_transf.tf.transform.rotation.w
    angles = transformations.euler_from_quaternion(rot)
    transf_mat = transformations.compose_matrix(angles=angles, translate=trans)
    depth_image = project_point_cloud.project_pointcloud(open3d_cloud, im_size, transf_mat, max_range)
    #from IPython import embed; embed()
    return depth_image.reshape(im_size[0], im_size[1], 1)
  
  def get_obs_voxels(self):
    pass


  def get_reward(self):
    curr_points = np.asarray(self.last_cloud.points)
    num_points = curr_points.size
    reward = num_points - self.curr_points
    self.curr_points = num_points
    if(reward<=50):
      self.termination_count +=1
    else:
      self.termination_count=0
    return reward
  
  def reset(self, im_size, max_range):
    self.curr_points=0
    self.termination_count = 0
    self.last_cloud=None
    self.reset_pc()
    rospy.sleep(0.1)
    next_obs = self.get_obs(im_size, max_range) #To reset the reward to the current reset observation
    self.get_reward()
    return next_obs
  
  def terminate(self):
    return self.termination_count>=self.max_no_reward