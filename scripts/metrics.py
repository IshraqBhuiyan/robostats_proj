#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
from scipy.ndimage import uniform_filter1d

data1 = np.load("reward_history_1_bunny.npy")
data2 = np.load("reward_history_1_bunny2.npy")

num_episodes = data1.shape[0] + data2.shape[0]
data = np.vstack((data1, data2))
print(num_episodes)

mean_eplength = np.zeros(num_episodes)
mean_reward = np.zeros(num_episodes)
total_reward = np.zeros(num_episodes)

for i in range(num_episodes):
  curr_data = data[i]
  end = np.max(np.nonzero(curr_data))
  curr_data = curr_data[:end+1]
  mean_eplength[i] = end+1
  mean_reward[i] = np.mean(curr_data)
  total_reward[i] = np.sum(curr_data)

mean_reward_smooth = uniform_filter1d(mean_reward, 10)
mean_eplength_smooth = uniform_filter1d(mean_eplength,10)
total_reward_smooth = uniform_filter1d(total_reward, 10)
plt.figure()
plt.plot(mean_reward)
plt.plot(mean_reward_smooth)
plt.xlabel("Episodes")
plt.ylabel("Mean Reward")
plt.title("Mean Reward Over Time")
plt.legend(["Actual", "Smoothed"])
plt.figure()
plt.plot(mean_eplength)
plt.plot(mean_eplength_smooth)
plt.xlabel("Episodes")
plt.ylabel("Episode length")
plt.title("Episode Length Over Time")
plt.legend(["Actual", "Smoothed"])
plt.figure()
plt.plot(total_reward)
plt.plot(total_reward_smooth)
plt.xlabel("Episodes")
plt.ylabel("Epsiode Total Reward")
plt.title("Total Episode Reward Over Time")
plt.legend(["Actual", "Smoothed"])
plt.show()