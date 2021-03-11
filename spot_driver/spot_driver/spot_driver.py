import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
import asyncio

import time
from math import radians, degrees, sqrt

from std_srvs.srv import Trigger
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from builtin_interfaces.msg import Time, Duration
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistWithCovarianceStamped, Twist, Pose

from bosdyn.api.geometry_pb2 import Quaternion
import bosdyn.geometry

from spot_driver_interfaces.msg import Metrics
from spot_driver_interfaces.msg import LeaseArray, LeaseResource
from spot_driver_interfaces.msg import FootState, FootStateArray
from spot_driver_interfaces.msg import FramePose
from spot_driver_interfaces.msg import EStopState, EStopStateArray
from spot_driver_interfaces.msg import WiFiState
from spot_driver_interfaces.msg import PowerState
from spot_driver_interfaces.msg import BehaviorFault, BehaviorFaultState
from spot_driver_interfaces.msg import SystemFault, SystemFaultState
from spot_driver_interfaces.msg import BatteryState, BatteryStateArray
from spot_driver_interfaces.msg import Feedback

from spot_driver_interfaces.srv import SetOrientation
from spot_driver_interfaces.srv import CreateFrame
from spot_driver_interfaces.srv import SetParameter
from spot_driver_interfaces.srv import StrikeAPose
from spot_driver_interfaces.srv import Turn
from spot_driver_interfaces.srv import Walk
from spot_driver_interfaces.srv import WalkTo
from spot_driver_interfaces.srv import WalkToInFrame
from spot_driver_interfaces.srv import WalkToRelative

from spot_driver.ros_helpers import *
from spot_driver.spot_wrapper import SpotWrapper, BASE_FRAME_NAME


class SpotDriver(Node):
    """Parent class for using the wrapper.  Defines all callbacks and keeps the wrapper alive"""

    obstacle_padding = 0.1  # [m]
    speed_limit = 0.5  # [m/s]
    horizontal_distance_accuracy = 0.75  # [m]
    angular_accuracy = 25.0  # [degree]
    goal_timeout = 30.0  # [s]

    rates = {
        'graph_nav': 1,
        'local_grid': 1,
        'robot_state': 10,
        'metrics': 1,
        'lease': 1,
        'front_image': 1,
        'side_image': 1,
        'rear_image': 1,
    }  # [Hz]
    auto_claim = True
    auto_power_on = True
    auto_stand = False
    timer_period = 0.1  # [second]

    _cb_group = ReentrantCallbackGroup()

    def __init__(self):
        super().__init__('spot_driver')
        self.get_logger().info("Starting ROS driver for Spot")

        self.declare_parameter('username')
        self.declare_parameter('password')
        self.declare_parameter('hostname')

        self.username = self.get_parameter(
            'username').get_parameter_value().string_value
        self.password = self.get_parameter(
            'password').get_parameter_value().string_value
        self.hostname = self.get_parameter(
            'hostname').get_parameter_value().string_value

        self._frame_publishers = {}
        self._publishers = {}
        self._subscriptions = []
        self._services = []

        self.callbacks = {}
        """Dictionary listing what callback to use for what data task"""
        self.callbacks['graph_nav'] = self.GraphNavCB
        self.callbacks['local_grid'] = self.LocalGridCB
        self.callbacks["robot_state"] = self.RobotStateCB
        self.callbacks["metrics"] = self.MetricsCB
        self.callbacks["lease"] = self.LeaseCB
        self.callbacks["front_image"] = self.FrontImageCB
        self.callbacks["side_image"] = self.SideImageCB
        self.callbacks["rear_image"] = self.RearImageCB

        """
        Main function for the SpotROS class.
        Gets config from ROS and initializes the wrapper.
        Holds lease from wrapper and updates all async tasks at the ROS rate
        """
        self.spot_wrapper = SpotWrapper(
            username=self.username,
            password=self.password,
            hostname=self.hostname,
            logger=self.get_logger(),
            rates=self.rates,
            callbacks=self.callbacks)

        if self.spot_wrapper.is_valid:
            self.get_logger().info("Spot wrapper is valid")
            # PointCloud #
            self.point_cloud_pub = self.create_publisher(
                PointCloud, 'point_cloud')
            # Images #
            self._publishers['back_image'] = self.create_publisher(
                Image, 'camera/back/image')
            self._publishers['frontleft_image'] = self.create_publisher(
                Image, 'camera/frontleft/image')
            self._publishers['frontright_image'] = self.create_publisher(
                Image, 'camera/frontright/image')
            self._publishers['left_image'] = self.create_publisher(
                Image, 'camera/left/image')
            self._publishers['right_image'] = self.create_publisher(
                Image, 'camera/right/image')
            # Depth #
            self._publishers['back_depth'] = self.create_publisher(
                Image, 'depth/back/image')
            self._publishers['frontleft_depth'] = self.create_publisher(
                Image, 'depth/frontleft/image')
            self._publishers['frontright_depth'] = self.create_publisher(
                Image, 'depth/frontright/image')
            self._publishers['left_depth'] = self.create_publisher(
                Image, 'depth/left/image')
            self._publishers['right_depth'] = self.create_publisher(
                Image, 'depth/right/image')

            # Image Camera Info #
            self._publishers['back_camera_info'] = self.create_publisher(
                CameraInfo, 'camera/back/camera_info')
            self._publishers['frontleft_camera_info'] = self.create_publisher(
                CameraInfo, 'camera/frontleft/camera_info')
            self._publishers['frontright_camera_info'] = self.create_publisher(
                CameraInfo, 'camera/frontright/camera_info')
            self._publishers['left_camera_info'] = self.create_publisher(
                CameraInfo, 'camera/left/camera_info')
            self._publishers['right_camera_info'] = self.create_publisher(
                CameraInfo, 'camera/right/camera_info')
            # Depth Camera Info #
            self._publishers['back_depth_info'] = self.create_publisher(
                CameraInfo, 'depth/back/camera_info')
            self._publishers['frontleft_depth_info'] = self.create_publisher(
                CameraInfo, 'depth/frontleft/camera_info')
            self._publishers['frontright_depth_info'] = self.create_publisher(
                CameraInfo, 'depth/frontright/camera_info')
            self._publishers['left_depth_info'] = self.create_publisher(
                CameraInfo, 'depth/left/camera_info')
            self._publishers['right_depth_info'] = self.create_publisher(
                CameraInfo, 'depth/right/camera_info')

            # Status Publishers #
            self._publishers['joint_states'] = self.create_publisher(
                JointState, 'joint_states')
            """Defining a TF publisher manually because of conflicts between Python3 and tf"""
            self._publishers['tf'] = self.create_publisher(TFMessage, 'tf')
            self._publishers['metrics'] = self.create_publisher(
                Metrics, 'status/metrics')
            self._publishers['leases'] = self.create_publisher(
                LeaseArray, 'status/leases')
            self._publishers['odom_twist'] = self.create_publisher(
                TwistWithCovarianceStamped, 'odometry/twist')
            self._publishers['vision_twist'] = self.create_publisher(
                TwistWithCovarianceStamped, 'odometry/twist')
            self._publishers['feet'] = self.create_publisher(
                FootStateArray, 'status/feet')
            self._publishers['estop'] = self.create_publisher(
                EStopStateArray, 'status/estop')
            self._publishers['wifi'] = self.create_publisher(
                WiFiState, 'status/wifi')
            self._publishers['power_state'] = self.create_publisher(
                PowerState, 'status/power_state')
            self._publishers['battery_states'] = self.create_publisher(
                BatteryStateArray, 'status/battery_states')
            self._publishers['behavior_faults'] = self.create_publisher(
                BehaviorFaultState, 'status/behavior_faults')
            self._publishers['system_faults'] = self.create_publisher(
                SystemFaultState, 'status/system_faults')

            self._publishers['feedback'] = self.create_publisher(
                Feedback, 'status/feedback')

            self._subscriptions.append(self.create_subscription(
                Twist, 'cmd_vel', self.cmdVelCallback))
            self._subscriptions.append(self.create_subscription(
                Pose, 'body_pose', self.bodyPoseCallback))

            self._services.append(self.create_service(
                Trigger, "claim", self.handle_claim))
            self._services.append(self.create_service(
                Trigger, "release", self.handle_release))
            self._services.append(self.create_service(
                Trigger, "stop", self.handle_stop))
            self._services.append(self.create_service(
                Trigger, "self_right", self.handle_self_right))
            self._services.append(self.create_service(
                Trigger, "sit", self.handle_sit))
            self._services.append(self.create_service(
                Trigger, "stand", self.handle_stand))
            self._services.append(self.create_service(
                Trigger, "power_on", self.handle_power_on))
            self._services.append(self.create_service(
                Trigger, "power_off", self.handle_safe_power_off))
            self._services.append(self.create_service(
                Trigger, "estop/hard", self.handle_estop_hard))
            self._services.append(self.create_service(
                Trigger, "estop/gentle", self.handle_estop_soft))

            # Custom services
            self._services.append(self.create_service(
                SetOrientation, "spot/set_orientation", self.handle_orient))
            self._services.append(self.create_service(
                CreateFrame, 'spot/create_frame', self.handle_create_frame))
            self._services.append(self.create_service(
                SetParameter, 'spot/set_parameter', self.handle_set_parameter))
            self._services.append(self.create_service(
                StrikeAPose, 'spot/strike_a_pose', self.handle_strike_a_pose))
            self._services.append(self.create_service(
                Turn, 'spot/turn', self.handle_turn,
                callback_group=self._cb_group))
            self._services.append(self.create_service(
                Walk, 'spot/walk', self.handle_walk,
                callback_group=self._cb_group))
            self._services.append(self.create_service(
                WalkTo, 'spot/walk_to', self.handle_walk_to,
                callback_group=self._cb_group))
            self._services.append(self.create_service(
                WalkToInFrame, 'spot/walk_to_in_frame',
                self.handle_walk_to_in_frame,
                callback_group=self._cb_group))
            self._services.append(self.create_service(
                WalkToRelative, 'spot/walk_to_relative',
                self.handle_walk_to_relative,
                callback_group=self._cb_group))

            self.set_mobility_params()

            if self.auto_claim:
                self.spot_wrapper.claim()
                if self.auto_power_on:
                    self.spot_wrapper.power_on()
                    if self.auto_stand:
                        self.spot_wrapper.stand()

            self.timer = self.create_timer(
                self.timer_period, self.timer_callback)
        else:
            self.get_logger().warning("Spot wrapper is not valid!")

    def set_mobility_params(self):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = 0.0
        q.w = 1.0

        euler_zxy = q.to_euler_zxy()
        self.spot_wrapper.set_mobility_params(
            body_height=0.0,
            footprint_R_body=euler_zxy,
            obstacle_padding=self.obstacle_padding,
            speed_limit=self.speed_limit,
        )

    def timer_callback(self):
        self.spot_wrapper.updateTasks()
        feedback_msg = Feedback()
        feedback_msg.standing = self.spot_wrapper.is_standing
        feedback_msg.sitting = self.spot_wrapper.is_sitting
        feedback_msg.moving = self.spot_wrapper.is_moving
        id = self.spot_wrapper.id
        try:
            feedback_msg.serial_number = id.serial_number
            feedback_msg.species = id.species
            feedback_msg.version = id.version
            feedback_msg.nickname = id.nickname
            feedback_msg.computer_serial_number = id.computer_serial_number
        except:
            pass
        self._publishers['feedback'].publish(feedback_msg)

    def GraphNavCB(self, results):
        """Callback for when the Spot Wrapper gets net graph nav data

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        # self.get_logger().info('Callback for new robot graph nav data')
        graph_nav = self.spot_wrapper.graph_nav

        point_cloud = GetPointCloudMsgFromGraphNav(
            graph_nav, self.spot_wrapper)
        self.point_cloud_pub.publish(point_cloud)

    def LocalGridCB(self, results):
        """TODO: add functionality
        """
        # self.get_logger().info('Callback for new robot local grid')
        local_grid = self.spot_wrapper.local_grid
        # self.get_logger().warning(f'local_grid: {local_grid}')

    def RobotStateCB(self, results):
        """Callback for when the Spot Wrapper gets new robot state data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        #self.get_logger().info('Callback for new robot state data')
        state = self.spot_wrapper.robot_state

        if state:
            ## joint states ##
            joint_state = GetJointStatesFromState(state, self.spot_wrapper)
            self._publishers['joint_states'].publish(joint_state)

            ## TF ##
            tf_msg = GetTFFromState(state, self.spot_wrapper)
            if len(tf_msg.transforms) > 0:
                self._publishers['tf'].publish(tf_msg)

            # Odom Twist #
            twist_odom_msg = GetOdomTwistFromState(state, self.spot_wrapper)
            self._publishers['odom_twist'].publish(twist_odom_msg)

            # Vision Twist #
            twist_vision_msg = GetVisionTwistFromState(
                state, self.spot_wrapper)
            self._publishers['vision_twist'].publish(twist_vision_msg)

            # Frame Twists
            for frame_name, frame_publisher in self._frame_publishers.items():
                frame_tform_body = self.spot_wrapper.get_frame_transform(
                    frame_name)

                frame_pose_msg = FramePose()
                frame_pose_msg.name = frame_name
                frame_pose_msg.x = frame_tform_body.position.x
                frame_pose_msg.y = frame_tform_body.position.y
                frame_pose_msg.z = frame_tform_body.position.z
                frame_pose_msg.yaw = degrees(
                    frame_tform_body.rotation.to_yaw())

                frame_publisher.publish(frame_pose_msg)

            # Feet #
            foot_array_msg = GetFeetFromState(state, self.spot_wrapper)
            self._publishers['feet'].publish(foot_array_msg)

            # EStop #
            estop_array_msg = GetEStopStateFromState(state, self.spot_wrapper)
            self._publishers['estop'].publish(estop_array_msg)

            # WIFI #
            wifi_msg = GetWifiFromState(state, self.spot_wrapper)
            self._publishers['wifi'].publish(wifi_msg)

            # Battery States #
            battery_states_array_msg = GetBatteryStatesFromState(
                state, self.spot_wrapper)
            self._publishers['battery_states'].publish(
                battery_states_array_msg)

            # Power State #
            power_state_msg = GetPowerStatesFromState(state, self.spot_wrapper)
            self._publishers['power_state'].publish(power_state_msg)

            # System Faults #
            system_fault_state_msg = GetSystemFaultsFromState(
                state, self.spot_wrapper)
            self._publishers['system_faults'].publish(system_fault_state_msg)

            # Behavior Faults #
            behavior_fault_state_msg = getBehaviorFaultsFromState(
                state, self.spot_wrapper)
            self._publishers['behavior_faults'].publish(
                behavior_fault_state_msg)

    def MetricsCB(self, results):
        """Callback for when the Spot Wrapper gets new metrics data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        metrics = self.spot_wrapper.metrics
        if metrics:
            metrics_msg = Metrics()
            local_time = self.spot_wrapper.robotToLocalTime(metrics.timestamp)

            metrics_msg.header.stamp = Time()
            metrics_msg.header.stamp.sec = local_time.seconds
            metrics_msg.header.stamp.nanosec = local_time.nanos

            for metric in metrics.metrics:
                if metric.label == "distance":
                    metrics_msg.distance = metric.float_value
                if metric.label == "gait cycles":
                    metrics_msg.gait_cycles = metric.int_value
                if metric.label == "time moving":
                    metrics_msg.time_moving = Duration()
                    metrics_msg.time_moving.sec = metric.duration.seconds
                    metrics_msg.time_moving.nanosec = metric.duration.nanos
                if metric.label == "electric power":
                    metrics_msg.electric_power = Duration()
                    metrics_msg.electric_power.sec = metric.duration.seconds
                    metrics_msg.electric_power.nanosec = metric.duration.nanos

            self._publishers['metrics'].publish(metrics_msg)

    def LeaseCB(self, results):
        """Callback for when the Spot Wrapper gets new lease data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        lease_array_msg = LeaseArray()
        lease_list = self.spot_wrapper.lease
        if lease_list:
            for resource in lease_list:
                new_resource = LeaseResource()
                new_resource.resource = resource.resource
                new_resource.lease.resource = resource.lease.resource
                new_resource.lease.epoch = resource.lease.epoch

                for seq in resource.lease.sequence:
                    new_resource.lease.sequence.append(seq)

                new_resource.lease_owner.client_name = resource.lease_owner.client_name
                new_resource.lease_owner.user_name = resource.lease_owner.user_name

                lease_array_msg.resources.append(new_resource)

            self._publishers['leases'].publish(lease_array_msg)

    def FrontImageCB(self, results):
        """Callback for when the Spot Wrapper gets new front image data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        data = self.spot_wrapper.front_images
        if data:
            image_msg0, camera_info_msg0, camera_tf_msg0 = getImageMsg(
                data[0], self.spot_wrapper)
            self._publishers['frontleft_image'](image_msg0)
            self._publishers['frontleft_camera_info'].publish(camera_info_msg0)
            self._publishers['tf'].publish(camera_tf_msg0)
            image_msg1, camera_info_msg1, camera_tf_msg1 = getImageMsg(
                data[1], self.spot_wrapper)
            self._publishers['frontright_image'].publish(image_msg1)
            self._publishers['frontright_camera_info'].publish(
                camera_info_msg1)
            self._publishers['tf'].publish(camera_tf_msg1)
            image_msg2, camera_info_msg2, camera_tf_msg2 = getImageMsg(
                data[2], self.spot_wrapper)
            self._publishers['frontleft_depth'].publish(image_msg2)
            self._publishers['frontleft_depth_info'].publish(camera_info_msg2)
            self._publishers['tf'].publish(camera_tf_msg2)
            image_msg3, camera_info_msg3, camera_tf_msg3 = getImageMsg(
                data[3], self.spot_wrapper)
            self._publishers['frontright_depth'].publish(image_msg3)
            self._publishers['frontright_depth_info'].publish(camera_info_msg3)
            self._publishers['tf'].publish(camera_tf_msg3)

    def SideImageCB(self, results):
        """Callback for when the Spot Wrapper gets new side image data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        data = self.spot_wrapper.side_images
        if data:
            image_msg0, camera_info_msg0, camera_tf_msg0 = getImageMsg(
                data[0], self.spot_wrapper)
            self._publishers['left_image'].publish(image_msg0)
            self._publishers['left_camera_info'].publish(camera_info_msg0)
            self._publishers['tf'].publish(camera_tf_msg0)
            image_msg1, camera_info_msg1, camera_tf_msg1 = getImageMsg(
                data[1], self.spot_wrapper)
            self._publishers['right_image'].publish(image_msg1)
            self._publishers['right_camera_info'].publish(camera_info_msg1)
            self._publishers['tf'].publish(camera_tf_msg1)
            image_msg2, camera_info_msg2, camera_tf_msg2 = getImageMsg(
                data[2], self.spot_wrapper)
            self._publishers['left_depth'].publish(image_msg2)
            self._publishers['left_depth_info'].publish(camera_info_msg2)
            self._publishers['tf'].publish(camera_tf_msg2)
            image_msg3, camera_info_msg3, camera_tf_msg3 = getImageMsg(
                data[3], self.spot_wrapper)
            self._publishers['right_depth'].publish(image_msg3)
            self._publishers['right_depth_info'].publish(camera_info_msg3)
            self._publishers['tf'].publish(camera_tf_msg3)

    def RearImageCB(self, results):
        """Callback for when the Spot Wrapper gets new rear image data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        data = self.spot_wrapper.rear_images
        if data:
            mage_msg0, camera_info_msg0, camera_tf_msg0 = getImageMsg(
                data[0], self.spot_wrapper)
            self._publishers['back_image'].publish(mage_msg0)
            self._publishers['back_camera_info'].publish(camera_info_msg0)
            self._publishers['tf'].publish(camera_tf_msg0)

            mage_msg1, camera_info_msg1, camera_tf_msg1 = getImageMsg(
                data[1], self.spot_wrapper)
            self._publishers['back_depth'].publish(mage_msg1)
            self._publishers['back_depth_info'].publish(camera_info_msg1)
            self._publishers['tf'].publish(camera_tf_msg1)

    def handle_claim(self, request, response):
        """ROS service handler for the claim service"""
        resp = self.spot_wrapper.claim()
        response.success = resp[0]
        response.message = resp[1]
        return response

    def handle_release(self, request, response):
        """ROS service handler for the release service"""
        resp = self.spot_wrapper.release()
        response.success = resp[0]
        response.message = resp[1]
        return response

    def handle_stop(self, request, response):
        """ROS service handler for the stop service"""
        resp = self.spot_wrapper.stop()
        response.success = resp[0]
        response.message = resp[1]
        return response

    def handle_self_right(self, request, response):
        """ROS service handler for the self-right service"""
        resp = self.spot_wrapper.self_right()
        response.success = resp[0]
        response.message = resp[1]
        return response

    def handle_orient(self, request, response):
        """ROS service handler for the orient service"""
        resp = self.spot_wrapper.orient(
            roll=request.roll,
            pitch=request.pitch,
            yaw=request.yaw,
            height=request.height,
            duration=request.duration,
        )
        response.success = resp[0]
        response.message = resp[1]
        return response

    def handle_sit(self, request, response):
        """ROS service handler for the sit service"""
        resp = self.spot_wrapper.sit()
        response.success = resp[0]
        response.message = resp[1]
        return response

    def handle_stand(self, request, response):
        """ROS service handler for the stand service"""
        resp = self.spot_wrapper.stand()
        response.success = resp[0]
        response.message = resp[1]
        return response

    def handle_power_on(self, request, response):
        """ROS service handler for the power-on service"""
        resp = self.spot_wrapper.power_on()
        response.success = resp[0]
        response.message = resp[1]
        return response

    def handle_safe_power_off(self, request, response):
        """ROS service handler for the safe-power-off service"""
        resp = self.spot_wrapper.safe_power_off()
        response.success = resp[0]
        response.message = resp[1]
        return response

    def handle_estop_hard(self, request, response):
        """ROS service handler to hard-eStop the robot.  The robot will immediately cut power to the motors"""
        resp = self.spot_wrapper.assertEStop(True)
        response.success = resp[0]
        response.message = resp[1]
        return response

    def handle_estop_soft(self, request, response):
        """ROS service handler to soft-eStop the robot.  The robot will try to settle on the ground before cutting power to the motors"""
        resp = self.spot_wrapper.assertEStop(False)
        response.success = resp[0]
        response.message = resp[1]
        return response

    # Custom service callbacks
    def create_frame(self, frame_name, position_x, position_y, heading):
        self.spot_wrapper.create_frame(
            frame_name=frame_name,
            position_x=position_x,
            position_y=position_y,
            heading=heading,
        )

        self.get_logger().info(f'Created frame: {frame_name}')
        if frame_name not in self._frame_publishers:
            topic_name = f'{frame_name}/pose'
            self._frame_publishers[frame_name] = self.create_publisher(
                msg_type=FramePose,
                topic=topic_name,
                callback_group=self._cb_group,
                qos_profile=qos_profile_sensor_data)

    def handle_create_frame(self, request, response):
        """ROS service handler"""
        self.get_logger().info(f'Service request "create_frame" received')

        try:
            self.create_frame(
                request.frame_name,
                request.position_x,
                request.position_y,
                request.heading,
            )
            response.success = True

        except Exception as e:
            response.success = False
            self.get_logger.error(f'create_frame service: {e}')

        return response

    def handle_set_parameter(self, request, response):
        """ROS service handler"""
        self.get_logger().info(f'Service request "set_parameter" received')

        if request.name == 'obstacle_padding':
            try:
                self.get_logger().info(
                    f'  Setting parameter {request.name} to {request.value}')
                self.obstacle_padding = float(request.value)
                self.set_mobility_params()
                response.success = True
            except:
                response.success = False
            self.get_logger().info(f'  Result: {response.success}')

        return response

    def handle_strike_a_pose(self, request, response):
        """ROS service handler"""
        self.get_logger().info(f'Service request "strike_a_pose" received')
        self.get_logger().error(f'This service has not been implemented yet')
        return response

    async def handle_turn(self, request, response):
        """ROS service handler"""
        self.get_logger().info(f'Service request "turn" received')

        base_tform_body = self.spot_wrapper.get_base_transform()
        body_tform_goal = self.spot_wrapper.create_SE3Pose(
            x=0.0,
            y=0.0,
            z=0.0,
            yaw=radians(request.degrees),
        )

        base_tform_goal = base_tform_body * body_tform_goal
        self.spot_wrapper.trajectory_cmd(
            goal_x=base_tform_goal.x,
            goal_y=base_tform_goal.y,
            goal_heading=base_tform_goal.rot.to_yaw(),
        )
        response.success = await self._wait_for_reach_goal(
            goal_tform=base_tform_goal,
            goal_timeout=request.max_duration,
        )

        return response

    async def handle_walk(self, request, response):
        """ROS service handler"""
        self.get_logger().info(f'Service request "walk" received')
        try:
            await self.spot_wrapper.velocity_cmd(
                v_x=request.twist.linear.x,
                v_y=request.twist.linear.y,
                v_rot=request.twist.angular.z,
                cmd_duration=request.duration,
            )
            response.success = True
        except Exception as e:
            response.success = False
            self.get_logger().error(f'Walk service: {e}')
        return response

    async def handle_walk_to(self, request, response):
        """ROS service handler"""
        self.get_logger().info(
            f'Service request "walk_to" received\n\t{request}')
        try:
            base_tform_goal = self.spot_wrapper.create_SE3Pose(
                x=request.goal_x,
                y=request.goal_y,
                z=0,
                yaw=radians(request.heading),
            )
            self.spot_wrapper.trajectory_cmd(
                goal_x=base_tform_goal.x,
                goal_y=base_tform_goal.y,
                goal_heading=base_tform_goal.rot.to_yaw(),
                cmd_duration=request.max_duration,
            )
            response.success = await self._wait_for_reach_goal(
                goal_tform=base_tform_goal,
                goal_timeout=request.max_duration,
            )
        except Exception as e:
            self.get_logger().error(f'Walk_to error: {e}')
            response.success = False

        return response

    async def handle_walk_to_in_frame(self, request, response):
        """ROS service handler"""
        self.get_logger().info(f'Service request "walk_to_in_frame" received')

        base_tform_frame = self.spot_wrapper.get_transform_between(
            BASE_FRAME_NAME,
            request.frame_name,
        )
        if base_tform_frame is None:
            self.get_logger().warning(
                f'walk_to_in_frame "{BASE_FRAME_NAME} to '
                f'{request.frame_name}" not found!')
            response.success = False
            return response

        frame_tform_goal = self.spot_wrapper.create_SE3Pose(
            x=request.goal_x,
            y=request.goal_y,
            z=0,
            yaw=radians(request.heading),
        )

        base_tform_goal = base_tform_frame * frame_tform_goal
        self.spot_wrapper.trajectory_cmd(
            goal_x=base_tform_goal.x,
            goal_y=base_tform_goal.y,
            goal_heading=base_tform_goal.rot.to_yaw(),
            cmd_duration=request.max_duration,
        )

        response.success = await self._wait_for_reach_goal(
            goal_tform=base_tform_goal,
            goal_timeout=request.max_duration,
        )
        return response

    async def handle_walk_to_relative(self, request, response):
        """ROS service handler"""
        self.get_logger().info(
            f'Service request "walk_to_relative" received:\n\t{request}')

        base_tform_body = self.spot_wrapper.get_base_transform()
        body_tform_goal = self.spot_wrapper.create_SE3Pose(
            x=request.dx,
            y=request.dy,
            z=0,
            yaw=radians(request.degrees),
        )

        base_tform_goal = base_tform_body * body_tform_goal
        self.spot_wrapper.trajectory_cmd(
            goal_x=base_tform_goal.x,
            goal_y=base_tform_goal.y,
            goal_heading=base_tform_goal.rot.to_yaw(),
            cmd_duration=request.max_duration,
        )
        response.success = await self._wait_for_reach_goal(
            goal_tform=base_tform_goal,
            goal_timeout=request.max_duration,
        )

        return response

    async def _wait_for_reach_goal(self, goal_tform, goal_timeout=None):
        if goal_timeout is None:
            goal_timeout = self.goal_timeout
        self.get_logger().info(
            f'_wait_for_reach_goal:\n ' +
            f'transform: {goal_tform}\n ' +
            f'goal_timeout:{goal_timeout}')

        start_time = time.time()
        reached_goal = False
        while True:
            current_base_tform = self.spot_wrapper.get_base_transform()
            horizontal_distance = sqrt(
                (goal_tform.x - current_base_tform.x)**2
                + (goal_tform.y - current_base_tform.y)**2
            )
            angular_difference = degrees(
                abs(goal_tform.rot.to_yaw() - current_base_tform.rot.to_yaw()))
            angular_distance = abs(
                (angular_difference + 180.0) % 360.0 - 180.0)
            self.get_logger().info(
                f'horizontal_distance: {horizontal_distance} <= ' +
                f'{self.horizontal_distance_accuracy}')
            self.get_logger().info(
                f'angular_distance: {angular_distance} <= ' +
                f'{self.angular_accuracy}')

            reached_horizontal = horizontal_distance <= \
                self.horizontal_distance_accuracy
            reached_angle = angular_distance <= self.angular_accuracy
            reached_goal = reached_horizontal and reached_angle

            goal_time = time.time() - start_time
            timeout_reached = goal_time >= goal_timeout
            self.get_logger().info(
                f'goal_time: {goal_time:.2f}/{goal_timeout:.2f}')
            if timeout_reached:
                self.get_logger().info(f'\ttimeoutreached')
                return False
            if reached_goal:
                self.get_logger().info(f'\tgoal reached')
                return True
            time.sleep(0.5)

    def cmdVelCallback(self, data):
        """Callback for cmd_vel command"""
        self.get_logger().info(f'Service request "cmd_vel" received')
        self.spot_wrapper.velocity_cmd(
            data.linear.x, data.linear.y, data.angular.z)

    def bodyPoseCallback(self, data):
        """Callback for cmd_vel command"""
        q = Quaternion()
        q.x = data.orientation.x
        q.y = data.orientation.y
        q.z = data.orientation.z
        q.w = data.orientation.w

        euler_zxy = q.to_euler_zxy()
        self.spot_wrapper.set_mobility_params(
            body_height=data.position.z,
            footprint_R_body=euler_zxy,
            obstacle_padding=self.obstacle_padding,
            speed_limit=self.speed_limit,
        )

    def shutdown(self):
        self.get_logger().info("Shutting down ROS driver for Spot")
        self.spot_wrapper.sit()
        self.spot_wrapper.disconnect()


def main(args=None):
    rclpy.init(args=args)

    spotdriver = SpotDriver()
    executor = MultiThreadedExecutor()

    rclpy.spin(spotdriver, executor=executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    spotdriver.shutdown()
    spotdriver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
