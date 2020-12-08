#!/usr/bin/env python3

import numpy as np
import os
import gym
from gym import spaces
import tensorflow as tf

import rospy
import ActionHandler
import project_point_cloud
import convertOpen3d
import envutils
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'

from stable_baselines.common.policies import CnnPolicy, MlpPolicy
from stable_baselines import PPO1
from stable_baselines.common.tf_layers import conv, linear, conv_to_fc, lstm
import open3d as o3d

class ScanEnv(gym.Env):

  def __init__(self):
    self.take_action = ActionHandler.ActionHandler()
    self.env_functions = envutils.env_utils()

    self.n_actions = 6
    self.action_space = spaces.Discrete(self.n_actions)
    self.sensor_max_range = 0.5
    self.h = 50
    self.w = 50
    self.imsize = (self.h, self.w)
    self.observation_space = spaces.Box(low=0, high=255, shape=(self.h, self.w,1))
    self.num_steps = 0

  def reset(self):
    self.num_steps = 0
    return self.env_functions.reset(self.imsize, self.sensor_max_range)
  
  def step(self, action):
    self.num_steps+=1
    self.take_action.move(action+1)
    obs = self.env_functions.get_obs(self.imsize, self.sensor_max_range)
    reward = self.env_functions.get_reward()
    #done = self.env_functions.terminate()
    done = self.num_steps>50 or self.env_functions.terminate()
    return obs, reward, done, {}
  
  def render(self, mode='human'):
    print("Number of steps ", self.num_steps)
    o3d.visualization.draw_geometries([self.env_functions.last_cloud])

def cnn(scaled_images, **kwargs):
    """
    CNN from Nature paper.

    :param scaled_images: (TensorFlow Tensor) Image input placeholder
    :param kwargs: (dict) Extra keywords parameters for the convolutional layers of the CNN
    :return: (TensorFlow Tensor) The CNN output layer
    """
    activ = tf.nn.relu
    layer_1 = activ(conv(scaled_images, 'c1', n_filters=4, filter_size=3, stride=2, init_scale=np.sqrt(2), **kwargs))
    layer_2 = activ(conv(layer_1, 'c2', n_filters=4, filter_size=3, stride=2, init_scale=np.sqrt(2), **kwargs))
    layer_3 = activ(conv(layer_2, 'c3', n_filters=4, filter_size=3, stride=2, init_scale=np.sqrt(2), **kwargs))
    layer_3 = conv_to_fc(layer_3)
    return activ(linear(layer_3, 'fc1', n_hidden=64, init_scale=np.sqrt(2)))


if __name__=="__main__":
  rospy.init_node("Scan_Environment_Training")

  env = ScanEnv()
  obs = env.reset()
  print(obs.min())

  #from IPython import embed; embed(colors="neutral")
  #pass
  model = PPO1(CnnPolicy, env, verbose=1)
  model.learn(total_timesteps=10000)
  model.save("ScanEnv_5_cnn")

  #model = PPO1.load("ScanEnv_1", env=env)

  obs = env.reset()

  while True:
    action, _states = model.predict(obs)
    obs, rewards, done, info = env.step(action)
    if done:
      np.save("obs_cnn.npy", obs)
      env.render()
      break
