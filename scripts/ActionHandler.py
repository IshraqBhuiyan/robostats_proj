from gazebo_msgs.srv import SetModelState
import gazebo_msgs.srv
import gazebo_msgs.msg
import sensor_msgs.msg
import rospy
import numpy as np

def make_move_req():
  move_req = gazebo_msgs.srv.SetModelStateRequest()
  move_req.model_state.model_name = "scanner"
  move_req.model_state.reference_frame = "scanner"
  return move_req

class ActionHandler():

  def __init__(self):
    rospy.wait_for_service("/gazebo/set_model_state")
    self.motion_handler = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    self.r = 0.4
    self.theta = 15 #degrees
    self.theta = np.deg2rad(self.theta)
    self.l = np.sqrt(2*self.r*self.r - 2*self.r*self.r*np.cos(self.theta))
    self.dt = .5
    self.ang_vel = (self.theta/2)/self.dt
    self.lin_vel = self.l/self.dt
    self.dt_yaw = 1
    self.ang_vel_yaw = np.deg2rad(90)/self.dt_yaw
  
  def move(self, action):
    if(action==1):
      self.move_north()
    elif(action==2):
      self.move_south()
    elif(action==3):
      self.move_east()
    elif(action==4):
      self.move_west()
    elif(action==5):
      self.yaw_left()
    elif(action==6):
      self.yaw_right()
  
  def move_north(self):
    stop_req = make_move_req()
    move_req = make_move_req()
    move_req.model_state.twist.angular.x = self.ang_vel
    self.motion_handler(move_req)
    rospy.sleep(self.dt)
    self.motion_handler(stop_req)
    
    move_req = make_move_req()
    move_req.model_state.twist.linear.z = self.lin_vel
    self.motion_handler(move_req)
    rospy.sleep(self.dt)
    self.motion_handler(stop_req)

    move_req = make_move_req()
    move_req.model_state.twist.angular.x = self.ang_vel
    self.motion_handler(move_req)
    rospy.sleep(self.dt)
    self.motion_handler(stop_req)
  
  def move_south(self):
    stop_req = make_move_req()
    move_req = make_move_req()
    move_req.model_state.twist.angular.x = -self.ang_vel
    self.motion_handler(move_req)
    rospy.sleep(self.dt)
    self.motion_handler(stop_req)
    
    move_req = make_move_req()
    move_req.model_state.twist.linear.z = -self.lin_vel
    self.motion_handler(move_req)
    rospy.sleep(self.dt)
    self.motion_handler(stop_req)

    move_req = make_move_req()
    move_req.model_state.twist.angular.x = -self.ang_vel
    self.motion_handler(move_req)
    rospy.sleep(self.dt)
    self.motion_handler(stop_req)
  
  def move_east(self):
    stop_req = make_move_req()
    move_req = make_move_req()
    move_req.model_state.twist.angular.z = self.ang_vel
    self.motion_handler(move_req)
    rospy.sleep(self.dt)
    self.motion_handler(stop_req)
    
    move_req = make_move_req()
    move_req.model_state.twist.linear.x = -self.lin_vel
    self.motion_handler(move_req)
    rospy.sleep(self.dt)
    self.motion_handler(stop_req)

    move_req = make_move_req()
    move_req.model_state.twist.angular.z = self.ang_vel
    self.motion_handler(move_req)
    rospy.sleep(self.dt)
    self.motion_handler(stop_req)
  
  def move_west(self):
    stop_req = make_move_req()
    move_req = make_move_req()
    move_req.model_state.twist.angular.z = -self.ang_vel
    self.motion_handler(move_req)
    rospy.sleep(self.dt)
    self.motion_handler(stop_req)
    
    move_req = make_move_req()
    move_req.model_state.twist.linear.x = self.lin_vel
    self.motion_handler(move_req)
    rospy.sleep(self.dt)
    self.motion_handler(stop_req)

    move_req = make_move_req()
    move_req.model_state.twist.angular.z = -self.ang_vel
    self.motion_handler(move_req)
    rospy.sleep(self.dt)
    self.motion_handler(stop_req)

  def yaw_right(self):
    stop_req = make_move_req()
    move_req = make_move_req()
    move_req.model_state.twist.angular.y = -self.ang_vel_yaw
    self.motion_handler(move_req)
    rospy.sleep(self.dt_yaw)
    self.motion_handler(stop_req)
  
  def yaw_left(self):
    stop_req = make_move_req()
    move_req = make_move_req()
    move_req.model_state.twist.angular.y = self.ang_vel_yaw
    self.motion_handler(move_req)
    rospy.sleep(self.dt_yaw)
    self.motion_handler(stop_req)

  
