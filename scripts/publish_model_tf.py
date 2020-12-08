#!/usr/bin/env python

import sys
import math
import rospy
import tf
import tf_conversions
import gazebo_msgs.msg
import geometry_msgs.msg
import numpy as np
from IPython import embed

def handle_link_states(msg):
  br = tf.TransformBroadcaster()
  pose_idx = None
  for (i, name) in enumerate(msg.name):
    if(name=="scanner"):
      pose_idx = i
  if(pose_idx==None):
    #rospy.logerr("Didn't find transform")
    return
  pose = msg.pose[pose_idx]
  translation = (pose.position.x, pose.position.y, pose.position.z)
  orientation = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)

  #tf_pose = tf_conversions.toTf(pose)
  br.sendTransform(translation, orientation, rospy.Time.now(), "/scan_origin", "/world")

if __name__ == "__main__":
  rospy.init_node('publish_model_tf')
  rospy.Subscriber("/gazebo/model_states", gazebo_msgs.msg.ModelStates, handle_link_states)
  rospy.spin()