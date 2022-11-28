import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal, FollowJointTrajectoryActionFeedback, FollowJointTrajectoryActionResult

class Spot_Joint_Trajectory_Controller(object):
    _feedback = FollowJointTrajectoryActionFeedback()
    _result = FollowJointTrajectoryActionResult()

    def __init__(self, name) -> None:
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name, FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        
        self._feedback.

if __name__ == '__main__':
    rospy.init_node('spot_arm_controller')
    server = Spot_Joint_Trajectory_Controller(rospy.get_name())
    rospy.spin()

