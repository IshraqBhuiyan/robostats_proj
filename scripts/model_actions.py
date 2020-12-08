#!/usr/bin/env python

from gazebo_msgs.srv import SetModelState
import gazebo_msgs.srv
import gazebo_msgs.msg
from laser_assembler.srv import AssembleScans2
import sensor_msgs.msg
import rospy
import tf_conversions
from IPython import embed
import numpy as np
import tf_conversions

def handle_link_states(msg):
  #print("Handler called")
  pose_idx = None
  for (i, name) in enumerate(msg.name):
    if(name=="scanner"):
      pose_idx = i
  if(pose_idx==None):
    #rospy.logerr("Didn't find transform")
    return
  #embed()
  global current_state
  current_state = msg.pose[pose_idx]

if __name__== "__main__":
  global current_state
  current_state = None
  rospy.init_node("test_thingy")
  rospy.wait_for_service("/gazebo/set_model_state")
  rospy.wait_for_service("assemble_scans2")
  rospy.Subscriber("/gazebo/model_states", gazebo_msgs.msg.ModelStates, handle_link_states)

  while(current_state==None):
    #print("waiting for message")
    pass
  movement = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
  move_req = gazebo_msgs.srv.SetModelStateRequest()
  move_req.model_state.model_name="scanner"
  move_req.model_state.pose = current_state
  move_req.model_state.twist.linear.z = 0.4
  move_req.model_state.reference_frame = "world"

  movement(move_req)

  rospy.sleep(0.8)
  move_req.model_state.twist.linear.z = 0
  move_req.model_state.pose=current_state
  movement(move_req)
  orient = move_req.model_state.pose.orientation
  orient = np.array([orient.x, orient.y, orient.z, orient.w])
  rot = tf_conversions.transformations.quaternion_from_euler(np.pi/2, 0,0)
  orient = tf_conversions.transformations.quaternion_multiply(rot, orient)
  move_req.model_state.pose.orientation.x = orient[0]
  move_req.model_state.pose.orientation.y = orient[1]
  move_req.model_state.pose.orientation.z = orient[2]
  move_req.model_state.pose.orientation.w = orient[3]
  move_req.model_state.twist.linear.y = -0.4
  movement(move_req)
  rospy.sleep(1)
  move_req.model_state.twist.linear.y = 0
  move_req.model_state.pose=current_state
  movement(move_req)

  assemble_scans = rospy.ServiceProxy("assemble_scans2", AssembleScans2)
  resp = assemble_scans(rospy.Time.now() - rospy.Duration(10), rospy.Time.now())

  pcd = resp.cloud
  pcd_pub = rospy.Publisher("assembled_pc", sensor_msgs.msg.PointCloud2, queue_size=2)
  for i in range(20):
    pcd_pub.publish(pcd)
    rospy.sleep(0.2)

  embed()


