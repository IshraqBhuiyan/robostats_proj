#!/usr/bin/env python3

import open3d as o3d
import numpy as np

if __name__== "__main__":
  filename = "bunny.pcd"
  pcd = o3d.io.read_point_cloud(filename)
  points = np.asarray(pcd.points)