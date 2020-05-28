#!/usr/bin/env python
import rospy
import copy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point, Twist, Pose

from random import random
from math import sin

server = None
vel_pub = None
body_pub = None

def platformCallback( msg ):
  time = rospy.Time.now()
  vel_msg = Twist()

  vel_pub.publish(vel_msg)

  server.setPose("spot_platform_control", geometry_msgs::Pose());

  server.applyChanges();

def bodyCallback( msg ):
  time = rospy.Time.now()
  body_msg = Pose()

  body_pub.publish(vel_msg)

  server.setPose("spot_body_control", geometry_msgs::Pose());

  server.applyChanges();

def makeBodyMarker():
  int_marker = InteractiveMarker()
  int_marker.header.frame_id = "body"
  int_marker.pose.position = Point(0,0,0)
  int_marker.scale = 0.3

  int_marker.name = "spot_body_control"

  control = InteractiveMarkerControl()
  control.orientation.w = 1
  control.orientation.x = 1
  control.orientation.y = 0
  control.orientation.z = 0
  control.name = "rotate_x"
  control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
  control.always_visible = True
  int_marker.controls.append(control)

  control = InteractiveMarkerControl()
  control.orientation.w = 1
  control.orientation.x = 0
  control.orientation.y = 1
  control.orientation.z = 0
  control.name = "move_z"
  control.always_visible = True
  control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
  control.orientation_mode = InteractiveMarkerControl.FIXED
  int_marker.controls.append(control)

  control = InteractiveMarkerControl()
  control.orientation.w = 1
  control.orientation.x = 0
  control.orientation.y = 0
  control.orientation.z = 1
  control.name = "rotate_y"
  control.always_visible = True
  control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
  int_marker.controls.append(control)

  server.insert(int_marker, bodyCallback)

def makePlatformMarker():
  int_marker = InteractiveMarker()
  int_marker.header.frame_id = "front_right_upper_leg"
  int_marker.pose.position = Point(0,0,0)
  int_marker.scale = 0.4
  int_marker.name = "spot_platform_control"

  control = InteractiveMarkerControl()
  control.orientation.w = 1
  control.orientation.x = 1
  control.orientation.y = 0
  control.orientation.z = 0
  control.name = "move_x"
  control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
  control.always_visible = True
  int_marker.controls.append(control)

  control = InteractiveMarkerControl()
  control.orientation.w = 1
  control.orientation.x = 0
  control.orientation.y = 1
  control.orientation.z = 0
  control.name = "rotate_z"
  control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
  control.always_visible = True
  int_marker.controls.append(control)

  control = InteractiveMarkerControl()
  control.orientation.w = 1
  control.orientation.x = 0
  control.orientation.y = 0
  control.orientation.z = 1
  control.name = "move_y"
  control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
  control.always_visible = True
  int_marker.controls.append(control)

  server.insert(int_marker, platformCallback)

if __name__=="__main__":
  rospy.init_node("spot_controller")

  server = InteractiveMarkerServer("spot_controller")
  vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
  body_pub = rospy.Publisher('body_pose', Pose, queue_size=10)

  makeBodyMarker()
  makePlatformMarker()

  server.applyChanges()

  rospy.spin()
