#!/usr/bin/env python3
from geometry_msgs.msg import PoseStamped, Pose

import numpy as np
import rospy

from bosdyn.client.math_helpers import SE3Pose as bdSE3Pose
from bosdyn.client.math_helpers import Quat as bdQuat

# import tf  # NOTE: pending use

class SE3Pose(bdSE3Pose):
    '''TODO: move this to a seperate library
    This class is a wrapper around bosdyn.client.math_helpers.SE3Pose.
    It adds functions that are useful for ROS.'''
    def __init__(self, x=0, y=0, z=0, rot=bdQuat(1,0,0,0), 
                 reference_frame=None):
        super().__init__(x, y, z, rot)
        self.ref=reference_frame
    
    @staticmethod
    def from_ros_pose(pose:Pose):
        p = pose.position
        q = pose.orientation
        se3 = SE3Pose(x=p.x, y=p.y, z=p.z,
                rot=bdQuat(x=q.x, y=q.y, z=q.z, w=q.w))
        return se3
    
    def to_ros_pose(self):
        pose = Pose()
        p = pose.position
        p.x, p.y, p.z = self.x, self.y, self.z
        q = pose.orientation
        q.x, q.y, q.z, q.w = self.rot.x, self.rot.y, self.rot.z, self.rot.w 
        return pose
    
    @staticmethod
    def from_matrix(m): 
        return SE3Pose._from_super(bdSE3Pose.from_matrix(m))
    
    def __mul__(self, other): 
        return self._from_super(super().__mul__(other))

    @staticmethod
    def _from_super(s): 
        return SE3Pose(x=s.x, y=s.y, z=s.z, rot=s.rot)

class MarkerPoseSubscriber(object):
    '''Subscribes to a pose topic and updates the interactive marker accordingly.'''
    def __init__(self, topic_name, marker, server, T:SE3Pose=SE3Pose()):
        '''
        Args:
            topic_name: The name of the topic to subscribe to.
            marker: The interactive marker to update.
            server: The interactive marker server.
            T: A 4x4 transformation matrix to apply to the pose before updating the marker.
        '''
        self._sub = rospy.Subscriber(topic_name, PoseStamped, self)
        self._marker, self._server = marker, server
        
        if isinstance(T, np.ndarray): self._T = SE3Pose.from_matrix(T)
        elif isinstance(T, SE3Pose): self._T = T
        else: raise ValueError("T must be a 4x4 numpy array or a SE3Pose.")

    def __call__(self, pose_stamped):
        rospy.loginfo('Updating interactive marker pose.')
        rospy.logdebug(pose_stamped)
        self._server.applyChanges()
        
        # NOTE: Currently this expects the incomming pose to be in the 
        #       same reference frame as the interactive marker. 
        # rospy.loginfo(self._marker.header.frame_id)
        # rospy.loginfo(pose_stamped.header.frame_id)
        # listener = tf.TransformListener()
        # rospy.loginfo(listener.allFramesAsString())
        # trans = listener.lookupTransform(self._marker.header.frame_id, 
        #                          pose_stamped.header.frame_id, 
        #                          rospy.Time())
        # rospy.loginfo(trans)

        pose = SE3Pose.from_ros_pose(pose_stamped.pose) * self._T
        if not self._server.setPose(self._marker.name, pose.to_ros_pose()):
            rospy.logerr("Failed to set pose for marker.")
        else: self._server.applyChanges()

        
