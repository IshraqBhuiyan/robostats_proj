import numpy as np

import rospy
import ActionHandler
import project_point_cloud
import convertOpen3d
import transformations
import open3d as o3d

from robostats_proj.srv import *
from std_srvs.srv import Empty

class env_utils():

  def __init__(self):
    self.curr_points = 0 # Used to calculate reward
    self.get_pc = rospy.ServiceProxy("/pc_utils/get_model_cloud", get_map)
    self.get_cam_transf = rospy.ServiceProxy("/pc_utils/get_camera_transf", get_camera_transform)
    self.reset_pc = rospy.ServiceProxy("/pc_utils/reset_pc", Empty)
    self.last_cloud = None
    self.max_no_reward = 40 # How many steps no reward can be found
    self.termination_count = 0
    self.rewards = np.zeros(10000)
    self.num_steps = 0

  def get_obs(self, im_size, max_range):
    self.num_steps +=1
    resp_cloud = self.get_pc()
    resp_transf = self.get_cam_transf()
    open3d_cloud = convertOpen3d.convertCloudFromRosToOpen3d(resp_cloud.cloud)
    if(open3d_cloud==None or np.asarray(open3d_cloud.points).size==0):
      #return self.get_obs(im_size, max_range)
      if(self.last_cloud==None):
        self.last_cloud = o3d.geometry.PointCloud()
      return np.ones((im_size[0], im_size[1], 1))*255
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
    if(np.asarray(open3d_cloud.points).size==0):
      depth_image = np.ones((im_size[0], im_size[1], 1))*255
    else:
      depth_image = project_point_cloud.project_pointcloud(open3d_cloud, im_size, transf_mat, max_range)
    #from IPython import embed; embed()
    return depth_image.reshape(im_size[0], im_size[1], 1)
  
  def get_obs_voxels(self):
    pass


  def get_reward(self):
    curr_cloud_points = np.asarray(self.last_cloud.voxel_down_sample(0.005).points)
    num_points = curr_cloud_points.shape[0]
    reward = num_points - self.curr_points
    self.rewards[self.num_steps-1] = reward
    if(self.num_steps==1): #or self.rewards[1:self.num_steps].std()<=0.0001):
      #print("First reward ", reward)
      reward=0
    else:
      #reward = (reward - self.rewards[1:self.num_steps].mean())/self.rewards[1:self.num_steps].std()
      r_max = self.rewards[1:self.num_steps].max()
      r_min = self.rewards[1:self.num_steps].min()
      reward = (reward-r_min)/(r_max - r_min) - 0.5
      if(not np.isfinite(reward)):
        reward=0
    self.curr_points = num_points
    if(reward<=-0.4):
      self.termination_count +=1
    else:
      self.termination_count=0
    return reward
  
  def reset(self, im_size, max_range):
    self.last_cloud=None
    self.reset_pc()
    rospy.sleep(0.1)
    next_obs = self.get_obs(im_size, max_range) #To reset the reward to the current reset observation
    self.rewards[:] = 0
    self.num_steps = 0
    self.curr_points=0
    self.termination_count = 0
    #self.get_reward()
    return next_obs
  
  def terminate(self):
    return self.termination_count>=self.max_no_reward