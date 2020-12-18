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
    self.h = 100
    self.w = 100
    self.imsize = (self.h, self.w)
    self.observation_space = spaces.Box(low=0, high=255, shape=(self.h, self.w,1))
    self.num_steps = 0
    self.reward_history = np.zeros(401)
    self.reward_stack = np.empty((0,401))

  def reset(self):
    self.reward_history[:] = 0
    self.num_steps = 0
    return self.env_functions.reset(self.imsize, self.sensor_max_range)
  
  def step(self, action):
    self.num_steps+=1
    self.take_action.move(action+1)
    obs = self.env_functions.get_obs(self.imsize, self.sensor_max_range)
    reward = self.env_functions.get_reward()
    self.reward_history[self.num_steps-1] = reward
    #done = self.env_functions.terminate()
    #if(self.num_steps%1==0):
    #  print("Reward: ", reward)
    #  print("step_num: ", self.num_steps)
    done = self.num_steps>400 or self.env_functions.terminate()
    if(done):
      self.reward_stack = np.append(self.reward_stack, self.reward_history.reshape(1,-1), axis=0)
      rewards = self.env_functions.rewards
      print("Ep length ", self.num_steps)
      print("Ep first reward ", rewards[0])
      print("Ep Max reward ", rewards[1:self.num_steps].max())
      print("Ep min reward ", rewards[1:self.num_steps].min())
      print("Ep mean reward ", rewards[1:self.num_steps].mean())
      print("Ep stddev ", rewards[1:self.num_steps].std())
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

def fixed_policy(env):
  env.reset()
  for i in range(16):
    obs, _, _, _ = env.step(1)
  np.save("obs_depth_bunny.npy", obs)
  for i in range(16):
    env.step(1)
  env.step(5)
  for i in range(60):
    env.step(1)
  env.step(5)
  for i in range(60):
    env.step(1)

def random_policy(env):
  env.reset()
  for i in range(15):
    obs, _, _, _ = env.step(env.action_space.sample())
  #np.save("obs_depth_bunny_random.npy", obs)
  for i in range(115):
    env.step(env.action_space.sample())

if __name__=="__main__":
  rospy.init_node("Scan_Environment_Training")

  env = ScanEnv()
  obs = env.reset()
  #print(obs.min())

  model = PPO1.load("ScanEnv_minmaxreward2_bunny2", env=env)
  #model = PPO1(CnnPolicy, env, verbose=1)
  #model.learn(total_timesteps=20000)
  #model.save("ScanEnv_minmaxreward2_bunny2")
  #np.save("reward_history_1_bunny2.npy", env.reward_stack)

  #model = PPO1.load("ScanEnv_minmaxreward2_bunny", env=env)

  #obs = env.reset()

  while True:
    action, _states = model.predict(obs)
    obs, rewards, done, info = env.step(action)
    if done:
      #np.save("obs_cnn.npy", obs)
      o3d.io.write_point_cloud("learned_policy_buddha.pcd", env.env_functions.last_cloud)
      env.render()
      break
  
  #fixed_policy(env)
  #random_policy(env)
  #o3d.io.write_point_cloud("random_policy_buddha.pcd", env.env_functions.last_cloud)
