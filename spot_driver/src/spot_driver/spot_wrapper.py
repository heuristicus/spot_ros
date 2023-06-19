import time
import math
#################################
# Additional import statements for recording
import os
import sys
import argparse
import logging
#Source: Required for recording_command_line.py in the Spot SDK examples
#################################

import numpy as np

import bosdyn.client.auth
from bosdyn.client import create_standard_sdk, ResponseError, RpcError
from bosdyn.client import robot_command
from bosdyn.client.async_tasks import AsyncPeriodicQuery, AsyncTasks
from bosdyn.geometry import EulerZXY

from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.robot_command import (
    RobotCommandClient,
    RobotCommandBuilder,
    blocking_stand,
)
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.frame_helpers import (
    get_odom_tform_body,
    ODOM_FRAME_NAME,
    BODY_FRAME_NAME,
    VISION_FRAME_NAME,
    get_se2_a_tform_b,
    get_a_tform_b
)
from bosdyn.client.power import safe_power_off, PowerClient, power_on
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.image import ImageClient, build_image_request
from bosdyn.client.docking import DockingClient, blocking_dock_robot, blocking_undock
from bosdyn.api import image_pb2
from bosdyn.api import estop_pb2, image_pb2
from bosdyn.api.graph_nav import graph_nav_pb2
from bosdyn.api.graph_nav import map_pb2
from bosdyn.api.graph_nav import nav_pb2
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client import power
from bosdyn.client import frame_helpers
from bosdyn.client import math_helpers
from bosdyn.client import robot_command
from bosdyn.client.exceptions import InternalServerError

from bosdyn.client.math_helpers import SE2Pose as bdSE2Pose
from bosdyn.client.math_helpers import SE3Pose as bdSE3Pose
from bosdyn.client.math_helpers import Quat as bdQuat

#################### Added code ######################
from bosdyn.client.recording import GraphNavRecordingServiceClient
from bosdyn.client.map_processing import MapProcessingServiceClient
# Sourced from recording_command_line.py in Spot-SDK, required to record
######################################################

from . import graph_nav_util
from .utils.feedback_handlers import (
    handle_se2_traj_cmd_feedback
)

from bosdyn.api import arm_command_pb2
import bosdyn.api.robot_state_pb2 as robot_state_proto
from bosdyn.api.robot_state_pb2 import ManipulatorState
from bosdyn.api import basic_command_pb2
from bosdyn.api import synchronized_command_pb2
from bosdyn.api import robot_command_pb2
from bosdyn.api import geometry_pb2
from bosdyn.api import trajectory_pb2
from bosdyn.api.gripper_command_pb2 import ClawGripperCommand
from bosdyn.util import seconds_to_duration
from google.protobuf.duration_pb2 import Duration
from google.protobuf.timestamp_pb2 import Timestamp

from .utils.asynchronous_tasks import AsyncMetrics, AsyncLease, AsyncImageService, AsyncIdle, AsyncEStopMonitor
from bosdyn.client.robot_command import block_until_arm_arrives

"""List of image sources for front image periodic query"""
front_image_infos = {
    "frontleft_fisheye_image": image_pb2.Image.PIXEL_FORMAT_RGB_U8,
    "frontright_fisheye_image": image_pb2.Image.PIXEL_FORMAT_RGB_U8,
    "frontleft_depth": image_pb2.Image.PIXEL_FORMAT_DEPTH_U16,
    "frontright_depth": image_pb2.Image.PIXEL_FORMAT_DEPTH_U16,
    "frontleft_depth_in_visual_frame": image_pb2.Image.PIXEL_FORMAT_DEPTH_U16,
    "frontright_depth_in_visual_frame": image_pb2.Image.PIXEL_FORMAT_DEPTH_U16,
}
front_image_sources = list(front_image_infos.keys())

"""List of image sources for side image periodic query"""
side_image_infos = {
    "left_fisheye_image": image_pb2.Image.PIXEL_FORMAT_RGB_U8,
    "right_fisheye_image": image_pb2.Image.PIXEL_FORMAT_RGB_U8,
    "left_depth": image_pb2.Image.PIXEL_FORMAT_DEPTH_U16,
    "right_depth": image_pb2.Image.PIXEL_FORMAT_DEPTH_U16,
    "left_depth_in_visual_frame": image_pb2.Image.PIXEL_FORMAT_DEPTH_U16,
    "right_depth_in_visual_frame": image_pb2.Image.PIXEL_FORMAT_DEPTH_U16,
}
side_image_sources = list(side_image_infos.keys())

"""List of image sources for rear image periodic query"""
rear_image_infos = {
    "back_fisheye_image": image_pb2.Image.PIXEL_FORMAT_RGB_U8,
    "back_depth": image_pb2.Image.PIXEL_FORMAT_DEPTH_U16,
    "back_depth_in_visual_frame": image_pb2.Image.PIXEL_FORMAT_DEPTH_U16,
}
rear_image_sources = list(rear_image_infos.keys())

"""List of image sources for hand image periodic query"""
hand_image_infos = {
    "hand_image": None, #image_pb2.Image.PIXEL_FORMAT_RGB_U8,
    "hand_depth": image_pb2.Image.PIXEL_FORMAT_DEPTH_U16,
    "hand_color_image": None, #image_pb2.Image.PIXEL_FORMAT_RGB_U8,
    "hand_depth_in_hand_color_frame": image_pb2.Image.PIXEL_FORMAT_DEPTH_U16,
}
hand_image_sources = list(hand_image_infos.keys())

class AsyncRobotState(AsyncPeriodicQuery):
    """Class to get robot state at regular intervals.  get_robot_state_async query sent to the robot at every tick.  Callback registered to defined callback function.

    Attributes:
        client: The Client to a service on the robot
        logger: Logger object
        rate: Rate (Hz) to trigger the query
        callback: Callback function to call when the results of the query are available
    """

    def __init__(self, client, logger, rate, callback):
        super(AsyncRobotState, self).__init__(
            "robot-state", client, logger, period_sec=1.0 / max(rate, 1.0)
        )
        self._callback = None
        if rate > 0.0:
            self._callback = callback

    def _start_query(self):
        if self._callback:
            callback_future = self._client.get_robot_state_async()
            callback_future.add_done_callback(self._callback)
            return callback_future

class SpotWrapper:
    """Generic wrapper class to encompass release 1.1.4 API features as well as maintaining leases automatically"""

    def __init__(
        self,
        username,
        password,
        hostname,
        logger,
        estop_timeout=9.0,
        rates={},
        callbacks={},
        client_metadata=None, ############ Added this for recording ##############
    ):
        self._username = username
        self._password = password
        self._hostname = hostname
        self._logger = logger
        self._rates = rates
        self._callbacks = callbacks
        self._estop_timeout = estop_timeout
        self._keep_alive = True
        self._valid = True

        self._mobility_params = RobotCommandBuilder.mobility_params()
        self._is_standing = False
        self._is_sitting = True
        self._is_moving = False
        self._at_goal = False
        self._near_goal = False
        self._trajectory_status_unknown = False
        self._last_stand_command = None
        self._last_sit_command = None
        self._last_trajectory_command = None
        self._last_trajectory_command_precise = None
        self._last_velocity_command_time = None
        self._last_docking_command = None


        def build_img_requests_helper(infos):
            requests = []
            for source, pxl_format in infos.items():
                IMG_FORMAT = image_pb2.Image.FORMAT_RAW
                requests.append(
                    build_image_request(source, 
                                        pixel_format=pxl_format,
                                        image_format=IMG_FORMAT))
            return requests
        self._front_image_requests = build_img_requests_helper(front_image_infos)
        self._side_image_requests = build_img_requests_helper(side_image_infos)
        self._rear_image_requests = build_img_requests_helper(rear_image_infos)
        self._hand_image_requests = build_img_requests_helper(hand_image_infos)


        try:
            self._sdk = create_standard_sdk("ros_spot")
        except Exception as e:
            self._logger.error("Error creating SDK object: %s", e)
            self._valid = False
            return

        self._logger.info("Initialising robot at {}".format(self._hostname))
        self._robot = self._sdk.create_robot(self._hostname)

        authenticated = False
        while not authenticated:
            try:
                self._logger.info("Trying to authenticate with robot...")
                self._robot.authenticate(self._username, self._password)
                self._robot.start_time_sync()
                self._logger.info("Successfully authenticated.")
                authenticated = True
            except RpcError as err:
                sleep_secs = 15
                self._logger.warn(
                    "Failed to communicate with robot: {}\nEnsure the robot is powered on and you can "
                    "ping {}. Robot may still be booting. Will retry in {} seconds".format(
                        err, self._hostname, sleep_secs
                    )
                )
                time.sleep(sleep_secs)
            except bosdyn.client.auth.InvalidLoginError as err:
                self._logger.error("Failed to log in to robot: {}".format(err))
                self._valid = False
                return

        if self._robot:
            # Clients
            self._logger.info("Creating clients...")
            initialised = False
            while not initialised:
                try:
                    self._robot.time_sync.wait_for_sync()
                    self._robot_state_client = self._robot.ensure_client(
                        RobotStateClient.default_service_name
                    )
                    self._robot_command_client = self._robot.ensure_client(
                        RobotCommandClient.default_service_name
                    )
                    self._graph_nav_client = self._robot.ensure_client(
                        GraphNavClient.default_service_name
                    )
                    self._power_client = self._robot.ensure_client(
                        PowerClient.default_service_name
                    )
                    self._lease_client = self._robot.ensure_client(
                        LeaseClient.default_service_name
                    )
                    self._lease_wallet = self._lease_client.lease_wallet
                    self._image_client = self._robot.ensure_client(
                        ImageClient.default_service_name
                    )
                    self._estop_client = self._robot.ensure_client(
                        EstopClient.default_service_name
                    )
                    self._docking_client = self._robot.ensure_client(
                        DockingClient.default_service_name
                    )
                    ############################################ Added code begins here
                    # Setup the recording service client.
                    # What this says is "Initialize the space on the robot to hold the recording
                    # and signal to the client we are doing so"
                    self._recording_client = self._robot.ensure_client(
                        GraphNavRecordingServiceClient.default_service_name)
                    #############################################
                    initialised = True
                except Exception as e:
                    sleep_secs = 15
                    self._logger.warn(
                        "Unable to create client service: {}. This usually means the robot hasn't "
                        "finished booting yet. Will wait {} seconds and try again.".format(
                            e, sleep_secs
                        )
                    )
                    time.sleep(sleep_secs)

            # Store the most recent knowledge of the state of the robot based on rpc calls.
            self._current_graph = None
            self._current_edges = dict()  # maps to_waypoint to list(from_waypoint)
            self._current_waypoint_snapshots = dict()  # maps id to waypoint snapshot
            self._current_edge_snapshots = dict()  # maps id to edge snapshot
            self._current_annotation_name_to_wp_id = dict()

            # Async Tasks
            self._async_task_list = []
            self._robot_state_task = AsyncRobotState(
                self._robot_state_client,
                self._logger,
                max(0.0, self._rates.get("robot_state", 0.0)),
                self._callbacks.get("robot_state", lambda: None),
            )
            self._robot_metrics_task = AsyncMetrics(
                self._robot_state_client,
                self._logger,
                max(0.0, self._rates.get("metrics", 0.0)),
                self._callbacks.get("metrics", lambda: None),
            )
            self._lease_task = AsyncLease(
                self._lease_client,
                self._logger,
                max(0.0, self._rates.get("lease", 0.0)),
                self._callbacks.get("lease", lambda: None),
            )

            self._hand_image_task  = self._make_image_service("hand_image",  self._hand_image_requests)
            self._front_image_task = self._make_image_service("front_image", self._front_image_requests)
            self._side_image_task  = self._make_image_service("side_image",  self._side_image_requests)
            self._rear_image_task  = self._make_image_service("rear_image",  self._rear_image_requests)

            self._hand_pointcloud_task  = self._make_image_service("hand_pointcloud",  self._hand_image_requests)
            self._front_pointcloud_task = self._make_image_service("front_pointcloud", self._front_image_requests)
            self._rear_pointcloud_task  = self._make_image_service("rear_pointcloud",  self._rear_image_requests)
            self._side_pointcloud_task  = self._make_image_service("side_pointcloud",  self._side_image_requests)

            self._idle_task = AsyncIdle(self._robot_command_client, self._logger, 10.0, self)

            self._estop_endpoint = self._estop_keepalive = None
            self._estop_monitor = AsyncEStopMonitor(self._estop_client, self._logger, 20.0, self)

            self._async_tasks = AsyncTasks([])
            def _add_task(task): 
                if task: self._async_tasks.add_task(task)

            for t in [  
                self._robot_state_task,
                self._robot_metrics_task,
                self._lease_task,
                self._front_image_task,
                self._side_image_task,
                self._rear_image_task,
                self._idle_task,
                self._estop_monitor,
            ]: _add_task(t)
           
            if self._robot.has_arm():
                for t in [
                    self._hand_image_task,
                    self._hand_pointcloud_task,
                    self._front_pointcloud_task,
                    self._rear_pointcloud_task,
                    self._side_pointcloud_task,
                ]: _add_task(t)

            self._robot_id = None
            self._lease = None
        ###############################################################
        # Create the recording environment.
        # Translation: "Set up more backend framework for ensuring we are able to store recorded info"
        self._recording_environment = GraphNavRecordingServiceClient.make_recording_environment(
            waypoint_env=GraphNavRecordingServiceClient.make_waypoint_environment(
                client_metadata=client_metadata))
        #Source in recording_command_line.py example in Spot-SDK
        ################################################################

    def _make_image_service(self, cb_name, data_requester):
        '''Helper function to create an AsyncImageService'''
        rate = max(0.0, self._rates.get(cb_name, 0.0))
        if rate < 0.0001: return None
        return AsyncImageService(
                    self._image_client,
                    self._logger,
                    rate,
                    self._callbacks.get(cb_name, lambda: None),
                    data_requester,
                )

    @property
    def logger(self):
        """Return logger instance of the SpotWrapper"""
        return self._logger

    @property
    def is_valid(self):
        """Return boolean indicating if the wrapper initialized successfully"""
        return self._valid

    @property
    def id(self):
        """Return robot's ID"""
        return self._robot_id

    @property
    def robot_state(self):
        """Return latest proto from the _robot_state_task"""
        return self._robot_state_task.proto

    @property
    def metrics(self):
        """Return latest proto from the _robot_metrics_task"""
        return self._robot_metrics_task.proto

    @property
    def lease(self):
        """Return latest proto from the _lease_task"""
        return self._lease_task.proto

    @property
    def front_images(self):
        """Return latest proto from the _front_image_task"""
        return self._front_image_task.proto

    @property
    def side_images(self):
        """Return latest proto from the _side_image_task"""
        return self._side_image_task.proto

    @property
    def rear_images(self):
        """Return latest proto from the _rear_image_task"""
        return self._rear_image_task.proto

    @property
    def hand_images(self):
        """Return latest proto from the _hand_image_task"""
        return self._hand_image_task.proto

    @property
    def is_standing(self):
        """Return boolean of standing state"""
        return self._is_standing

    @property
    def is_sitting(self):
        """Return boolean of standing state"""
        return self._is_sitting

    @property
    def is_moving(self):
        """Return boolean of walking state"""
        return self._is_moving

    @property
    def near_goal(self):
        return self._near_goal

    @property
    def at_goal(self):
        return self._at_goal

    @property
    def time_skew(self):
        """Return the time skew between local and spot time"""
        return self._robot.time_sync.endpoint.clock_skew

    def resetMobilityParams(self):
        """
        Resets the mobility parameters used for motion commands to the default values provided by the bosdyn api.
        Returns:
        """
        self._mobility_params = RobotCommandBuilder.mobility_params()

    def robotToLocalTime(self, timestamp):
        """Takes a timestamp and an estimated skew and return seconds and nano seconds in local time

        Args:
            timestamp: google.protobuf.Timestamp
        Returns:
            google.protobuf.Timestamp
        """

        rtime = Timestamp()

        rtime.seconds = timestamp.seconds - self.time_skew.seconds
        rtime.nanos = timestamp.nanos - self.time_skew.nanos
        if rtime.nanos < 0:
            rtime.nanos = rtime.nanos + 1000000000
            rtime.seconds = rtime.seconds - 1

        # Workaround for timestamps being incomplete
        if rtime.seconds < 0:
            rtime.seconds = 0
            rtime.nanos = 0

        return rtime

    def claim(self):
        """Get a lease for the robot, a handle on the estop endpoint, and the ID of the robot."""
        try:
            self._robot_id = self._robot.get_id()
            self.getLease(hijack=True)
            self.resetEStop()
            return True, "Success"
        except (ResponseError, RpcError) as err:
            self._logger.error("Failed to initialize robot communication: %s", err)
            return False, str(err)

    def updateTasks(self):
        """Loop through all periodic tasks and update their data if needed."""
        try:
            self._async_tasks.update()
        except Exception as e:
            print(f"Update tasks failed with error: {str(e)}")

    def resetEStop(self):
        """Get keepalive for eStop"""
        self._estop_endpoint = EstopEndpoint(
            self._estop_client, "ros", self._estop_timeout
        )
        self._estop_endpoint.force_simple_setup()  # Set this endpoint as the robot's sole estop.
        self._estop_keepalive = EstopKeepAlive(self._estop_endpoint)

    def assertEStop(self, severe=True):
        """Forces the robot into eStop state.

        Args:
            severe: Default True - If true, will cut motor power immediately.  If false, will try to settle the robot on the ground first
        """
        try:
            if severe:
                self._estop_keepalive.stop()
            else:
                self._estop_keepalive.settle_then_cut()

            return True, "Success"
        except:
            return False, "Error"

    def disengageEStop(self):
        """Disengages the E-Stop"""
        try:
            self._estop_keepalive.allow()
            return True, "Success"
        except:
            return False, "Error"

    def releaseEStop(self):
        """Stop eStop keepalive"""
        if self._estop_keepalive:
            self._estop_keepalive.stop()
            self._estop_keepalive = None
            self._estop_endpoint = None

    def getLease(self, hijack=False):
        """Get a lease for the robot and keep the lease alive automatically."""
        
        if hijack: self._lease = self._lease_client.take()
        else: self._lease = self._lease_client.acquire()
        self._lease_keepalive = LeaseKeepAlive(self._lease_client)

    def releaseLease(self):
        """Return the lease on the body."""
        if self._lease:
            self._lease_keepalive.shutdown()
            self._lease_client.return_lease(self._lease)
            self._lease = None

    def release(self):
        """Return the lease on the body and the eStop handle."""
        try:
            self.sit()
            self.disconnect()
            return True, "Success"
        except Exception as e:
            return False, str(e)

    def disconnect(self):
        """Release control of robot as gracefully as posssible."""
        if self._robot.time_sync:
            self._robot.time_sync.stop()
        self.releaseLease()
        self.releaseEStop()

    def _robot_command(self, command_proto, end_time_secs=None, timesync_endpoint=None):
        """Generic blocking function for sending commands to robots.

        Args:
            command_proto: robot_command_pb2 object to send to the robot.  Usually made with RobotCommandBuilder
            end_time_secs: (optional) Time-to-live for the command in seconds
            timesync_endpoint: (optional) Time sync endpoint
        """
        try:
            id = self._robot_command_client.robot_command(
                lease=None,
                command=command_proto,
                end_time_secs=end_time_secs,
                timesync_endpoint=timesync_endpoint,
            )
            return True, "Success", id
        except Exception as e:
            return False, str(e), None

    def stop(self):
        """Stop the robot's motion."""
        response = self._robot_command(RobotCommandBuilder.stop_command())
        return response[0], response[1]

    def self_right(self):
        """Have the robot self-right itself."""
        response = self._robot_command(RobotCommandBuilder.selfright_command())
        return response[0], response[1]

    def sit(self):
        """Stop the robot's motion and sit down if able."""
        self.arm_stow()
        response = self._robot_command(RobotCommandBuilder.synchro_sit_command())
        self._last_sit_command = response[2]
        return response[0], response[1]

    def stand(
        self, monitor_command=True, body_height=0, body_yaw=0, body_pitch=0, body_roll=0
    ):
        """
        If the e-stop is enabled, and the motor power is enabled, stand the robot up.
        Executes a stand command, but one where the robot will assume the pose specified by the given parameters.

        If no parameters are given this behave just as a normal stand command

        Args:
            monitor_command: Track the state of the command in the async idle, which sets is_standing
            body_height: Offset of the body relative to normal stand height, in metres
            body_yaw: Yaw of the body in radians
            body_pitch: Pitch of the body in radians
            body_roll: Roll of the body in radians

        """
        if any([body_height, body_yaw, body_pitch, body_roll]):
            # If any of the orientation parameters are nonzero use them to pose the body
            body_orientation = EulerZXY(yaw=body_yaw, pitch=body_pitch, roll=body_roll)
            response = self._robot_command(
                RobotCommandBuilder.synchro_stand_command(
                    body_height=body_height, footprint_R_body=body_orientation
                )
            )
        else:
            # Otherwise just use the mobility params
            response = self._robot_command(
                RobotCommandBuilder.synchro_stand_command(params=self._mobility_params)
            )

        if monitor_command:
            self._last_stand_command = response[2]
        return response[0], response[1]

    def safe_power_off(self):
        """Stop the robot's motion and sit if possible.  Once sitting, disable motor power."""
        try:
            self.sit()
            
            self._logger.info('Powering Off...')
            self._robot.power_off()
            # self.spot.releaseLease()  # TODO: check if this ought to be used. 
            
            self._logger.info('Disconnecting...')
            self.disconnect()

            assert not self._robot.is_powered_on(), "Robot power off failed."
            self._logger.info('Done!')
            return True, "Success"
        except Exception as e:
            return False, str(e)
    
    def __del__(self): self.safe_power_off()

    def clear_behavior_fault(self, id):
        """Clear the behavior fault defined by id."""
        try:
            rid = self._robot_command_client.clear_behavior_fault(
                behavior_fault_id=id, lease=None
            )
            return True, "Success", rid
        except Exception as e:
            return False, str(e), None

    def power_on(self):
        """Enble the motor power if e-stop is enabled."""
        try:
            self._robot.power_on()
            return True, "Success"
        except Exception as e:
            return False, str(e)

    def set_mobility_params(self, mobility_params):
        """Set Params for mobility and movement

        Args:
            mobility_params: spot.MobilityParams, params for spot mobility commands.
        """
        self._mobility_params = mobility_params

    def get_mobility_params(self):
        """Get mobility params"""
        return self._mobility_params

    def velocity_cmd(self, v_x, v_y, v_rot, cmd_duration=0.125):
        """Send a velocity motion command to the robot.

        Args:
            v_x: Velocity in the X direction in meters
            v_y: Velocity in the Y direction in meters
            v_rot: Angular velocity around the Z axis in radians
            cmd_duration: (optional) Time-to-live for the command in seconds.  Default is 125ms (assuming 10Hz command rate).
        """
        end_time = time.time() + cmd_duration
        response = self._robot_command(
            RobotCommandBuilder.synchro_velocity_command(
                v_x=v_x, v_y=v_y, v_rot=v_rot, params=self._mobility_params
            ),
            end_time_secs=end_time,
            timesync_endpoint=self._robot.time_sync.endpoint,
        )
        self._last_velocity_command_time = end_time
        return response[0], response[1]

    def _transform_bd_pose(self, pose, reference_frame:str, target_frame:str):
        Tf_tree = self._robot_state_client.get_robot_state().kinematic_state.transforms_snapshot
        if isinstance(pose, bdSE2Pose):
            T = get_se2_a_tform_b(Tf_tree, target_frame, reference_frame)
        elif isinstance(pose, bdSE3Pose):
            T = get_a_tform_b(Tf_tree, target_frame, reference_frame)
        else:
            raise ValueError("pose must be either bdSE2Pose or bdSE3Pose")
        return T * pose

    def trajectory_cmd(
        self,
        goal_x,
        goal_y,
        goal_heading,
        cmd_duration,
        reference_frame=BODY_FRAME_NAME,
        frame_name="odom",
        precise_position=False,
        blocking=False,
        build_on_command=None,
    ):
        """Send a trajectory motion command to the robot.

        Args:
            goal_x: Position X coordinate in meters
            goal_y: Position Y coordinate in meters
            goal_heading: Pose heading in radians
            cmd_duration: Time-to-live for the command in seconds.
            frame_name: frame_name to be used to calc the target position. 'odom' or 'vision'
            precise_position: if set to false, the status STATUS_NEAR_GOAL and STATUS_AT_GOAL will be equivalent. If
            true, the robot must complete its final positioning before it will be considered to have successfully
            reached the goal.
            reference_frame: The frame in which the goal is represented

        Returns: (bool, str) tuple indicating whether the command was successfully sent, and a message
        """
        
        if frame_name not in [ODOM_FRAME_NAME, VISION_FRAME_NAME]: 
            raise ValueError("frame_name must be 'vision' or 'odom'")
        self._logger.info("Recieved Pose Trajectory Command.")
        self._logger.debug(f"\tx: {goal_x}, y: {goal_y}, goal_heading: {goal_heading}")
        self._logger.debug(f"cmd_duration: {cmd_duration}, frame_name: {frame_name}, precise_position: {precise_position}")

        self._at_goal = False
        self._near_goal = False
        self._trajectory_status_unknown = False
        self._last_trajectory_command_precise = precise_position
        end_time = time.time() + cmd_duration

        T_in_ref = bdSE2Pose(x=goal_x, y=goal_y, angle=goal_heading)
        T_in_target = self._transform_bd_pose(T_in_ref, reference_frame, frame_name)

        response = self._robot_command(
                RobotCommandBuilder.synchro_se2_trajectory_point_command(
                    goal_x=T_in_target.x,
                    goal_y=T_in_target.y,
                    goal_heading=T_in_target.angle,
                    frame_name=frame_name,
                    params=self._mobility_params,
                    build_on_command=build_on_command,
                ),
                end_time_secs=end_time,
            )

        if response[0]: 
            self._last_trajectory_command = response[2]
            if blocking: 
                handle_se2_traj_cmd_feedback(response[2], 
                                             self._robot_command_client)

        return response[0], response[1], response[2]

    def list_graph(self):
        """List waypoint ids of garph_nav
        Args:
          upload_path : Path to the root directory of the map.
        """
        ids, eds = self._list_graph_waypoint_and_edge_ids()
        # skip waypoint_ for v2.2.1, skip waypiont for < v2.2
        return [
            v
            for k, v in sorted(
                ids.items(), key=lambda id: int(id[0].replace("waypoint_", ""))
            )
        ]

    def battery_change_pose(self, dir_hint=1):
        """Robot sit down and roll on to it its side for easier battery access"""
        response = self._robot_command(
            RobotCommandBuilder.battery_change_pose_command(dir_hint)
        )
        return response[0], response[1]
        

    def navigate_to(
        self,
        upload_path,
        navigate_to,
        initial_localization_fiducial=True,
        initial_localization_waypoint=None,
    ):
        """navigate with graph nav.

        Args:
           upload_path : Path to the root directory of the map.
           navigate_to : Waypont id string for where to goal
           initial_localization_fiducial : Tells the initializer whether to use fiducials
           initial_localization_waypoint : Waypoint id string of current robot position (optional)
        """
        # Filepath for uploading a saved graph's and snapshots too.
        if upload_path[-1] == "/":
            upload_filepath = upload_path[:-1]
        else:
            upload_filepath = upload_path


        # FIX ME somehow,,,, if the robot is stand, need to sit the robot before starting garph nav
        if self.is_standing and not self.is_moving:
            self.sit()

        assert not self._robot.is_estopped(), "Robot is estopped. cannot complete navigation"
        assert self.check_is_powered_on(), "Robot not powered on, cannot complete navigation"
        assert self._lease != None, "No lease claim, cannot complete navigations"
        self._clear_graph()
        self._upload_graph_and_snapshots(upload_filepath)
        if initial_localization_fiducial:
            self._set_initial_localization_fiducial()
        if initial_localization_waypoint:
            self._set_initial_localization_waypoint([initial_localization_waypoint])
        self._list_graph_waypoint_and_edge_ids()
        self._get_localization_state()
        resp = self._navigate_to([navigate_to])

        return resp

    # Arm ############################################
    def ensure_arm_power_and_stand(self):
        if not self._robot.has_arm():
            return False, "Spot with an arm is required for this service"

        if not self._robot.is_powered_on():
            try:
                self._logger.info("Spot is powering on within the timeout of 20 secs")
                self._robot.power_on(timeout_sec=20)
                assert self._robot.is_powered_on(), "Spot failed to power on"
                self._logger.info("Spot is powered on")
            except Exception as e:
                return (
                    False,
                    f"Exception occured while Spot or its arm were trying to power on\n{e}",
                )

        if not self._is_standing:
            robot_command.blocking_stand(
                command_client=self._robot_command_client, timeout_sec=10.0
            )
            self._logger.info("Spot is standing")
        else:
            self._logger.info("Spot is already standing")

        return True, "Spot has an arm, is powered on, and standing"


    def arm_stow(self):

        state = self.robot_state.manipulator_state

        if state.stow_state == ManipulatorState.StowState.STOWSTATE_STOWED:
            self._logger.info("Arm is stowed.")
            return True, "Arm is stowed."

        if state.is_gripper_holding_item:
            # Open Gripper
            s, _ = self.gripper_open()
            if not s: return s, "Failed to open gripper"

        # Stow Arm
        stow = RobotCommandBuilder.arm_stow_command()
        cmd_id = self._robot_command_client.robot_command(stow)
        self._logger.info("Command stow issued")
        s = block_until_arm_arrives(self._robot_command_client, 
                                          cmd_id, 10.0)
        if not s: return False, "Arm failed to stow"
        
        if state.gripper_open_percentage > 1e-5:
            # Close Gripper
            s, _ = self.gripper_close()
            if not s: return s, "Failed to close gripper"
        
        return True, "Stow arm success"

    def arm_unstow(self):
        try:
            success, msg = self.ensure_arm_power_and_stand()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                # Unstow Arm
                unstow = RobotCommandBuilder.arm_ready_command()

                # Command issue with RobotCommandClient
                cmd_id = self._robot_command_client.robot_command(unstow)
                block_until_arm_arrives(self._robot_command_client, cmd_id, 10.0)
                
                self._logger.info("Command unstow issued")
                time.sleep(2.0)

        except Exception as e:
            return False, "Exception occured while trying to unstow"

        return True, "Unstow arm success"

    def arm_carry(self):
        try:
            success, msg = self.ensure_arm_power_and_stand()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                # Get Arm in carry mode
                carry = RobotCommandBuilder.arm_carry_command()

                # Command issue with RobotCommandClient
                cmd_id = self._robot_command_client.robot_command(carry)
                block_until_arm_arrives(self._robot_command_client, cmd_id, 10.0)
                self._logger.info("Command carry issued")
                time.sleep(2.0)

        except Exception as e:
            return False, "Exception occured while carry mode was issued"

        return True, "Carry mode success"

    def make_arm_trajectory_command(self, arm_joint_trajectory):
        """Helper function to create a RobotCommand from an ArmJointTrajectory.
        Copy from 'spot-sdk/python/examples/arm_joint_move/arm_joint_move.py'"""

        joint_move_command = arm_command_pb2.ArmJointMoveCommand.Request(
            trajectory=arm_joint_trajectory
        )
        arm_command = arm_command_pb2.ArmCommand.Request(
            arm_joint_move_command=joint_move_command
        )
        sync_arm = synchronized_command_pb2.SynchronizedCommand.Request(
            arm_command=arm_command
        )
        arm_sync_robot_cmd = robot_command_pb2.RobotCommand(
            synchronized_command=sync_arm
        )
        return RobotCommandBuilder.build_synchro_command(arm_sync_robot_cmd)

    def arm_joint_move(self, joint_targets):
        # All perspectives are given when looking at the robot from behind after the unstow service is called
        # Joint1: 0.0 arm points to the front. positive: turn left, negative: turn right)
        # RANGE: -3.14 -> 3.14
        # Joint2: 0.0 arm points to the front. positive: move down, negative move up
        # RANGE: 0.4 -> -3.13 (
        # Joint3: 0.0 arm straight. moves the arm down
        # RANGE: 0.0 -> 3.1415
        # Joint4: 0.0 middle position. negative: moves ccw, positive moves cw
        # RANGE: -2.79253 -> 2.79253
        # # Joint5: 0.0 gripper points to the front. positive moves the gripper down
        # RANGE: -1.8326 -> 1.8326
        # Joint6: 0.0 Gripper is not rolled, positive is ccw
        # RANGE: -2.87 -> 2.87
        # Values after unstow are: [0.0, -0.9, 1.8, 0.0, -0.9, 0.0]
        if abs(joint_targets[0]) > 3.14:
            msg = "Joint 1 has to be between -3.14 and 3.14"
            self._logger.warn(msg)
            return False, msg
        elif joint_targets[1] > 0.4 or joint_targets[1] < -3.13:
            msg = "Joint 2 has to be between -3.13 and 0.4"
            self._logger.warn(msg)
            return False, msg
        elif joint_targets[2] > 3.14 or joint_targets[2] < 0.0:
            msg = "Joint 3 has to be between 0.0 and 3.14"
            self._logger.warn(msg)
            return False, msg
        elif abs(joint_targets[3]) > 2.79253:
            msg = "Joint 4 has to be between -2.79253 and 2.79253"
            self._logger.warn(msg)
            return False, msg
        elif abs(joint_targets[4]) > 1.8326:
            msg = "Joint 5 has to be between -1.8326 and 1.8326"
            self._logger.warn(msg)
            return False, msg
        elif abs(joint_targets[5]) > 2.87:
            msg = "Joint 6 has to be between -2.87 and 2.87"
            self._logger.warn(msg)
            return False, msg
        try:
            success, msg = self.ensure_arm_power_and_stand()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                trajectory_point = (
                    RobotCommandBuilder.create_arm_joint_trajectory_point(
                        joint_targets[0],
                        joint_targets[1],
                        joint_targets[2],
                        joint_targets[3],
                        joint_targets[4],
                        joint_targets[5],
                    )
                )
                arm_joint_trajectory = arm_command_pb2.ArmJointTrajectory(
                    points=[trajectory_point]
                )
                arm_command = self.make_arm_trajectory_command(arm_joint_trajectory)

                # Send the request
                cmd_id = self._robot_command_client.robot_command(arm_command)

                # Query for feedback to determine how long it will take
                feedback_resp = self._robot_command_client.robot_command_feedback(
                    cmd_id
                )
                joint_move_feedback = (
                    feedback_resp.feedback.synchronized_feedback.arm_command_feedback.arm_joint_move_feedback
                )
                time_to_goal: Duration = joint_move_feedback.time_to_goal
                time_to_goal_in_seconds: float = time_to_goal.seconds + (
                    float(time_to_goal.nanos) / float(10**9)
                )
                time.sleep(time_to_goal_in_seconds)
                return True, "Spot Arm moved successfully"

        except Exception as e:
            return False, "Exception occured during arm movement: " + str(e)

    def force_trajectory(self, data):
        try:
            success, msg = self.ensure_arm_power_and_stand()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                # Unstow arm
                arm_ready_command = RobotCommandBuilder.arm_ready_command()

                # Send command via the RobotCommandClient
                self._robot_command_client.robot_command(arm_ready_command)

                self._logger.info("Unstow command issued.")
                time.sleep(2.0)

                # Demonstrate an example force trajectory by ramping up and down a vertical force over
                # 10 seconds

                def create_wrench_from_msg(forces, torques):
                    force = geometry_pb2.Vec3(x=forces[0], y=forces[1], z=forces[2])
                    torque = geometry_pb2.Vec3(x=torques[0], y=torques[1], z=torques[2])
                    return geometry_pb2.Wrench(force=force, torque=torque)

                # Duration in seconds.
                traj_duration = 5

                # first point on trajectory
                wrench0 = create_wrench_from_msg(data.forces_pt0, data.torques_pt0)
                t0 = seconds_to_duration(0)
                traj_point0 = trajectory_pb2.WrenchTrajectoryPoint(
                    wrench=wrench0, time_since_reference=t0
                )

                # Second point on the trajectory
                wrench1 = create_wrench_from_msg(data.forces_pt1, data.torques_pt1)
                t1 = seconds_to_duration(traj_duration)
                traj_point1 = trajectory_pb2.WrenchTrajectoryPoint(
                    wrench=wrench1, time_since_reference=t1
                )

                # Build the trajectory
                trajectory = trajectory_pb2.WrenchTrajectory(
                    points=[traj_point0, traj_point1]
                )

                # Build the trajectory request, putting all axes into force mode
                arm_cartesian_command = arm_command_pb2.ArmCartesianCommand.Request(
                    root_frame_name=ODOM_FRAME_NAME,
                    wrench_trajectory_in_task=trajectory,
                    x_axis=arm_command_pb2.ArmCartesianCommand.Request.AXIS_MODE_FORCE,
                    y_axis=arm_command_pb2.ArmCartesianCommand.Request.AXIS_MODE_FORCE,
                    z_axis=arm_command_pb2.ArmCartesianCommand.Request.AXIS_MODE_FORCE,
                    rx_axis=arm_command_pb2.ArmCartesianCommand.Request.AXIS_MODE_FORCE,
                    ry_axis=arm_command_pb2.ArmCartesianCommand.Request.AXIS_MODE_FORCE,
                    rz_axis=arm_command_pb2.ArmCartesianCommand.Request.AXIS_MODE_FORCE,
                )
                arm_command = arm_command_pb2.ArmCommand.Request(
                    arm_cartesian_command=arm_cartesian_command
                )
                synchronized_command = (
                    synchronized_command_pb2.SynchronizedCommand.Request(
                        arm_command=arm_command
                    )
                )
                robot_command = robot_command_pb2.RobotCommand(
                    synchronized_command=synchronized_command
                )

                # Send the request
                self._robot_command_client.robot_command(robot_command)
                self._logger.info("Force trajectory command sent")

                time.sleep(10.0)

        except Exception as e:
            return False, "Exception occured during arm movement"

        return True, "Moved arm successfully"

    def _block_for_gripper(self, id, timeout_sec):
        class Timer: 
            def __init__(self, timeout_sec):
                self.timeout_sec = timeout_sec
                self.start_time = time.time()
            def ringing(self):
                return (time.time() - self.start_time) >= self.timeout_sec

        # Ensure it is a gripper command id
        resp = self._robot_command_client.robot_command_feedback(id)
        f = resp.feedback.synchronized_feedback
        if not f.HasField('gripper_command_feedback') or\
           not f.gripper_command_feedback.HasField('claw_gripper_feedback'):
            raise RuntimeError('Wrong command id passed to _block_for_gripper...')
        
        timer = Timer(timeout_sec)
        # Start checking status: return on fail, break otherwise
        status = f.gripper_command_feedback.claw_gripper_feedback.status
        while not timer.ringing():
            time.sleep(0.1)
            resp = self._robot_command_client.robot_command_feedback(id)
            f = resp.feedback.synchronized_feedback
            status = f.gripper_command_feedback.claw_gripper_feedback.status
            if status == ClawGripperCommand.Feedback.STATUS_UNKNOWN: return False
            elif status != ClawGripperCommand.Feedback.STATUS_IN_PROGRESS: break
        time.sleep(0.2)  # Allow the gripper to apply some force
        return False if timer.ringing() else True
        

    def gripper_open(self):
        try:
            success, msg = self.ensure_arm_power_and_stand()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                # Open gripper
                command = RobotCommandBuilder.claw_gripper_open_command()

                # Command issue with RobotCommandClient
                cmd_id = self._robot_command_client.robot_command(command)
                self._logger.info("Command gripper open sent")
                # time.sleep(2.0)
                self._block_for_gripper(cmd_id, 2.0)

        except Exception as e:
            self._logger.warning(e)
            return False, "Exception occured while gripper was moving"

        return True, "Open gripper success"

    def gripper_close(self):
        try:
            success, msg = self.ensure_arm_power_and_stand()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                # Close gripper
                command = RobotCommandBuilder.claw_gripper_close_command()

                # Command issue with RobotCommandClient
                cmd_id = self._robot_command_client.robot_command(command)
                self._logger.info("Command gripper close sent")
                # time.sleep(2.0)
                self._block_for_gripper(cmd_id, 2.0)

        except Exception as e:
            return False, "Exception occured while gripper was moving"

        return True, "Closed gripper successfully"

    def gripper_angle_open(self, gripper_ang):
        # takes an angle between 0 (closed) and 90 (fully opened) and opens the
        # gripper at this angle
        if gripper_ang > 90 or gripper_ang < 0:
            return False, "Gripper angle must be between 0 and 90"
        try:
            success, msg = self.ensure_arm_power_and_stand()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                # The open angle command does not take degrees but the limits
                # defined in the urdf, that is why we have to interpolate
                closed = 0.349066
                opened = -1.396263
                angle = gripper_ang / 90.0 * (opened - closed) + closed
                command = RobotCommandBuilder.claw_gripper_open_angle_command(angle)

                # Command issue with RobotCommandClient
                self._robot_command_client.robot_command(command)
                self._logger.info("Command gripper open angle sent")
                time.sleep(2.0)

        except Exception as e:
            return False, "Exception occured while gripper was moving"

        return True, "Opened gripper successfully"

    def hand_pose(self, 
                  position:np.array,
                  quaternion:np.array, 
                  reference_frame:str = BODY_FRAME_NAME, 
                  duration:float = 5.0, 
                  gripper_opening:float = None,
                  blocking=True, 
                ):
        '''
        Commands the gripper to move to a configuration.
        :param position: The position of the gripper in the reference frame.
        :param quaternion: The orientation of the gripper in the reference frame.
                           Quaternion is in the form (w, x, y, z).
        :param reference_frame: The reference frame of the position and orientation.
                                Defaults to BODY_FRAME_NAME.
        :param duration: The time in seconds to move to the target pose.
                         Defaults to 5.0.
        :param gripper_opening: The fraction of the gripper opening. If None, the gripper
                                will not be commanded.  Defaults to None.
        :param blocking: If True, the function will block until the command is complete.
        '''
        try:
            success, msg = self.ensure_arm_power_and_stand()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                arm_cmd = RobotCommandBuilder.arm_pose_command(
                    *position, *quaternion, reference_frame, duration
                )
                
                if gripper_opening:
                    claw_cmd = RobotCommandBuilder.claw_gripper_open_fraction_command
                    gripper_cmd = claw_cmd(gripper_opening)
                    command = RobotCommandBuilder.build_synchro_command(gripper_cmd, arm_cmd)
                else: 
                    command = RobotCommandBuilder.build_synchro_command(arm_cmd)


                # Send the request
                cmd_id = self._robot_command_client.robot_command(command)
                self._logger.info("Moving arm to position.")

                ARM_FEEDBACK = arm_command_pb2.ArmCartesianCommand.Feedback
                if blocking: 
                    status = ARM_FEEDBACK.STATUS_IN_PROGRESS
                    while status == ARM_FEEDBACK.STATUS_IN_PROGRESS:
                        feedback_resp = self._robot_command_client.robot_command_feedback(cmd_id)
                        arm_car_feedback = feedback_resp.feedback.synchronized_feedback\
                                                        .arm_command_feedback.arm_cartesian_feedback
                        self._logger.debug(
                            'Distance to go: ' +
                            '{:.2f} meters'.format(arm_car_feedback.measured_pos_distance_to_goal) +
                            ', {:.2f} radians'.format(arm_car_feedback.measured_rot_distance_to_goal))

                        time.sleep(0.25)
                        status = arm_car_feedback.status
                    if status == ARM_FEEDBACK.STATUS_TRAJECTORY_COMPLETE:
                        self._logger.info('Move complete.')
                    else:
                        self._logger.info('Move failed. Arm status: {}'.format(status))
                        raise Exception(str(status))

        except Exception as e:
            return False, f"Error in hand_pose:\n{e}"

        return True, "Moved arm successfully"

    ###################################################################

    ## copy from spot-sdk/python/examples/graph_nav_command_line/graph_nav_command_line.py
    def _get_localization_state(self, *args):
        """Get the current localization and state of the robot."""
        state = self._graph_nav_client.get_localization_state()
        self._logger.info("Got localization: \n%s" % str(state.localization))
        odom_tform_body = get_odom_tform_body(
            state.robot_kinematics.transforms_snapshot
        )
        self._logger.info(
            "Got robot state in kinematic odometry frame: \n%s" % str(odom_tform_body)
        )

    def _set_initial_localization_fiducial(self, *args):
        """Trigger localization when near a fiducial."""
        robot_state = self._robot_state_client.get_robot_state()
        current_odom_tform_body = get_odom_tform_body(
            robot_state.kinematic_state.transforms_snapshot
        ).to_proto()
        # Create an empty instance for initial localization since we are asking it to localize
        # based on the nearest fiducial.
        localization = nav_pb2.Localization()
        self._graph_nav_client.set_localization(
            initial_guess_localization=localization,
            ko_tform_body=current_odom_tform_body,
        )

    def _set_initial_localization_waypoint(self, *args):
        """Trigger localization to a waypoint."""
        # Take the first argument as the localization waypoint.
        if len(args) < 1:
            # If no waypoint id is given as input, then return without initializing.
            self._logger.error("No waypoint specified to initialize to.")
            return
        destination_waypoint = graph_nav_util.find_unique_waypoint_id(
            args[0][0],
            self._current_graph,
            self._current_annotation_name_to_wp_id,
            self._logger,
        )
        if not destination_waypoint:
            # Failed to find the unique waypoint id.
            return
        robot_state = self._robot_state_client.get_robot_state()
        current_odom_tform_body = get_odom_tform_body(
            robot_state.kinematic_state.transforms_snapshot
        ).to_proto()
        # Create an initial localization to the specified waypoint as the identity.
        localization = nav_pb2.Localization()
        localization.waypoint_id = destination_waypoint
        localization.waypoint_tform_body.rotation.w = 1.0
        self._graph_nav_client.set_localization(
            initial_guess_localization=localization,
            # It's hard to get the pose perfect, search +/-20 deg and +/-20cm (0.2m).
            max_distance=0.2,
            max_yaw=20.0 * math.pi / 180.0,
            fiducial_init=graph_nav_pb2.SetLocalizationRequest.FIDUCIAL_INIT_NO_FIDUCIAL,
            ko_tform_body=current_odom_tform_body,
        )

    def _list_graph_waypoint_and_edge_ids(self, *args):
        """List the waypoint ids and edge ids of the graph currently on the robot."""

        # Download current graph
        graph = self._graph_nav_client.download_graph()
        if graph is None:
            self._logger.error("Empty graph.")
            return
        self._current_graph = graph

        localization_id = (
            self._graph_nav_client.get_localization_state().localization.waypoint_id
        )

        # Update and print waypoints and edges
        (
            self._current_annotation_name_to_wp_id,
            self._current_edges,
        ) = graph_nav_util.update_waypoints_and_edges(
            graph, localization_id, self._logger
        )
        return self._current_annotation_name_to_wp_id, self._current_edges

    def _upload_graph_and_snapshots(self, upload_filepath):
        """Upload the graph and snapshots to the robot."""
        self._logger.info("Loading the graph from disk into local storage...")
        with open(upload_filepath + "/graph", "rb") as graph_file:
            # Load the graph from disk.
            data = graph_file.read()
            self._current_graph = map_pb2.Graph()
            self._current_graph.ParseFromString(data)
            self._logger.info(
                "Loaded graph has {} waypoints and {} edges".format(
                    len(self._current_graph.waypoints), len(self._current_graph.edges)
                )
            )
        for waypoint in self._current_graph.waypoints:
            # Load the waypoint snapshots from disk.
            with open(
                upload_filepath + "/waypoint_snapshots/{}".format(waypoint.snapshot_id),
                "rb",
            ) as snapshot_file:
                waypoint_snapshot = map_pb2.WaypointSnapshot()
                waypoint_snapshot.ParseFromString(snapshot_file.read())
                self._current_waypoint_snapshots[
                    waypoint_snapshot.id
                ] = waypoint_snapshot
        for edge in self._current_graph.edges:
            # Load the edge snapshots from disk.
            with open(
                upload_filepath + "/edge_snapshots/{}".format(edge.snapshot_id), "rb"
            ) as snapshot_file:
                edge_snapshot = map_pb2.EdgeSnapshot()
                edge_snapshot.ParseFromString(snapshot_file.read())
                self._current_edge_snapshots[edge_snapshot.id] = edge_snapshot
        # Upload the graph to the robot.
        self._logger.info("Uploading the graph and snapshots to the robot...")
        self._graph_nav_client.upload_graph(
            lease=self._lease.lease_proto, graph=self._current_graph
        )
        # Upload the snapshots to the robot.
        for waypoint_snapshot in self._current_waypoint_snapshots.values():
            self._graph_nav_client.upload_waypoint_snapshot(waypoint_snapshot)
            self._logger.info("Uploaded {}".format(waypoint_snapshot.id))
        for edge_snapshot in self._current_edge_snapshots.values():
            self._graph_nav_client.upload_edge_snapshot(edge_snapshot)
            self._logger.info("Uploaded {}".format(edge_snapshot.id))

        # The upload is complete! Check that the robot is localized to the graph,
        # and it if is not, prompt the user to localize the robot before attempting
        # any navigation commands.
        localization_state = self._graph_nav_client.get_localization_state()
        if not localization_state.localization.waypoint_id:
            # The robot is not localized to the newly uploaded graph.
            self._logger.info(
                "Upload complete! The robot is currently not localized to the map; please localize",
                "the robot before attempting a navigation command.",
            )

    def _navigate_to(self, *args):
        """Navigate to a specific waypoint."""
        # Take the first argument as the destination waypoint.
        if len(args) < 1:
            # If no waypoint id is given as input, then return without requesting navigation.
            self._logger.info("No waypoint provided as a destination for navigate to.")
            return
        assert not self._robot.is_estopped(), "Robot is estopped. cannot complete navigation"
        assert self.check_is_powered_on(), "Robot not powered on, cannot complete navigation"
        assert self._lease != None, "No lease claim, cannot complete navigations"
        self._lease = self._lease_wallet.get_lease()
        destination_waypoint = graph_nav_util.find_unique_waypoint_id(
            args[0][0],
            self._current_graph,
            self._current_annotation_name_to_wp_id,
            self._logger,
        )
        if not destination_waypoint:
            # Failed to find the appropriate unique waypoint id for the navigation command.
            return
        if not self.toggle_power(should_power_on=True):
            self._logger.info(
                "Failed to power on the robot, and cannot complete navigate to request."
            )
            return

        # Stop the lease keepalive and create a new sublease for graph nav.
        self._lease = self._lease_wallet.advance()
        sublease = self._lease.create_sublease()
        self._lease_keepalive.shutdown()

        # Navigate to the destination waypoint.
        is_finished = False
        nav_to_cmd_id = -1
        while not is_finished:
            # Issue the navigation command about twice a second such that it is easy to terminate the
            # navigation command (with estop or killing the program).
            nav_to_cmd_id = self._graph_nav_client.navigate_to(
                destination_waypoint, 1.0, leases=[sublease.lease_proto]
            )
            time.sleep(0.5)  # Sleep for half a second to allow for command execution.
            # Poll the robot for feedback to determine if the navigation command is complete. Then sit
            # the robot down once it is finished.
            is_finished = self._check_success(nav_to_cmd_id)

        self._lease = self._lease_wallet.advance()
        self._lease_keepalive = LeaseKeepAlive(self._lease_client)
        # Update the lease and power off the robot if appropriate.
        if self.check_is_powered_on():
            # Sit the robot down + power off after the navigation command is complete.
            self.toggle_power(should_power_on=False)

        status = self._graph_nav_client.navigation_feedback(nav_to_cmd_id)
        if (
            status.status
            == graph_nav_pb2.NavigationFeedbackResponse.STATUS_REACHED_GOAL
        ):
            return True, "Successfully completed the navigation commands!"
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_LOST:
            return (
                False,
                "Robot got lost when navigating the route, the robot will now sit down.",
            )
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_STUCK:
            return (
                False,
                "Robot got stuck when navigating the route, the robot will now sit down.",
            )
        elif (
            status.status
            == graph_nav_pb2.NavigationFeedbackResponse.STATUS_ROBOT_IMPAIRED
        ):
            return False, "Robot is impaired."
        else:
            return False, "Navigation command is not complete yet."

    def _navigate_route(self, *args):
        """Navigate through a specific route of waypoints."""
        if len(args) < 1:
            # If no waypoint ids are given as input, then return without requesting navigation.
            self._logger.error("No waypoints provided for navigate route.")
            return
        assert not self._robot.is_estopped(), "Robot is estopped. cannot complete navigation"
        assert self.check_is_powered_on(), "Robot not powered on, cannot complete navigation"
        assert self._lease != None, "No lease claim, cannot complete navigations"
        waypoint_ids = args[0]
        for i in range(len(waypoint_ids)):
            waypoint_ids[i] = graph_nav_util.find_unique_waypoint_id(
                waypoint_ids[i],
                self._current_graph,
                self._current_annotation_name_to_wp_id,
                self._logger,
            )
            if not waypoint_ids[i]:
                # Failed to find the unique waypoint id.
                return
        edge_ids_list = []
        all_edges_found = True
        # Attempt to find edges in the current graph that match the ordered waypoint pairs.
        # These are necessary to create a valid route.
        for i in range(len(waypoint_ids) - 1):
            start_wp = waypoint_ids[i]
            end_wp = waypoint_ids[i + 1]
            edge_id = self._match_edge(self._current_edges, start_wp, end_wp)
            if edge_id is not None:
                edge_ids_list.append(edge_id)
            else:
                all_edges_found = False
                self._logger.error(
                    "Failed to find an edge between waypoints: ",
                    start_wp,
                    " and ",
                    end_wp,
                )
                self._logger.error(
                    "List the graph's waypoints and edges to ensure pairs of waypoints has an edge."
                )
                break

        self._lease = self._lease_wallet.get_lease()
        if all_edges_found:
            if not self.toggle_power(should_power_on=True):
                self._logger.error(
                    "Failed to power on the robot, and cannot complete navigate route request."
                )
                return

            # Stop the lease keepalive and create a new sublease for graph nav.
            self._lease = self._lease_wallet.advance()
            sublease = self._lease.create_sublease()
            self._lease_keepalive.shutdown()

            # Navigate a specific route.
            route = self._graph_nav_client.build_route(waypoint_ids, edge_ids_list)
            is_finished = False
            while not is_finished:
                # Issue the route command about twice a second such that it is easy to terminate the
                # navigation command (with estop or killing the program).
                nav_route_command_id = self._graph_nav_client.navigate_route(
                    route, cmd_duration=1.0, leases=[sublease.lease_proto]
                )
                time.sleep(
                    0.5
                )  # Sleep for half a second to allow for command execution.
                # Poll the robot for feedback to determine if the route is complete. Then sit
                # the robot down once it is finished.
                is_finished = self._check_success(nav_route_command_id)

            self._lease = self._lease_wallet.advance()
            self._lease_keepalive = LeaseKeepAlive(self._lease_client)

            # Update the lease and power off the robot if appropriate.
            if self.check_is_powered_on():
                # Sit the robot down + power off after the navigation command is complete.
                self.toggle_power(should_power_on=False)

    def _clear_graph(self, *args):
        """Clear the state of the map on the robot, removing all waypoints and edges."""
        return self._graph_nav_client.clear_graph(lease=self._lease.lease_proto)

    def toggle_power(self, should_power_on):
        """Power the robot on/off dependent on the current power state."""
        is_powered_on = self.check_is_powered_on()
        if not is_powered_on and should_power_on:
            # Power on the robot up before navigating when it is in a powered-off state.
            power_on(self._power_client)
            motors_on = False
            while not motors_on:
                future = self._robot_state_client.get_robot_state_async()
                state_response = future.result(
                    timeout=10
                )  # 10 second timeout for waiting for the state response.
                if (
                    state_response.power_state.motor_power_state
                    == robot_state_proto.PowerState.STATE_ON
                ):
                    motors_on = True
                else:
                    # Motors are not yet fully powered on.
                    time.sleep(0.25)
        elif is_powered_on and not should_power_on:
            # Safe power off (robot will sit then power down) when it is in a
            # powered-on state.
            safe_power_off(self._robot_command_client, self._robot_state_client)
        else:
            # Return the current power state without change.
            return is_powered_on
        # return the new power state of the robot.
        return self.check_is_powered_on()

    def check_is_powered_on(self):
        """Determine if the robot is powered on or off."""
        power_state = self._robot_state_client.get_robot_state().power_state
        powered_on = power_state.motor_power_state == power_state.STATE_ON
        return powered_on

    def _check_success(self, command_id=-1):
        """Use a navigation command id to get feedback from the robot and sit when command succeeds."""
        if command_id == -1:
            # No command, so we have not status to check.
            return False
        status = self._graph_nav_client.navigation_feedback(command_id)
        if (
            status.status
            == graph_nav_pb2.NavigationFeedbackResponse.STATUS_REACHED_GOAL
        ):
            # Successfully completed the navigation commands!
            return True
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_LOST:
            self._logger.error(
                "Robot got lost when navigating the route, the robot will now sit down."
            )
            return True
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_STUCK:
            self._logger.error(
                "Robot got stuck when navigating the route, the robot will now sit down."
            )
            return True
        elif (
            status.status
            == graph_nav_pb2.NavigationFeedbackResponse.STATUS_ROBOT_IMPAIRED
        ):
            self._logger.error("Robot is impaired.")
            return True
        else:
            # Navigation command is not complete yet.
            return False

    def _match_edge(self, current_edges, waypoint1, waypoint2):
        """Find an edge in the graph that is between two waypoint ids."""
        # Return the correct edge id as soon as it's found.
        for edge_to_id in current_edges:
            for edge_from_id in current_edges[edge_to_id]:
                if (waypoint1 == edge_to_id) and (waypoint2 == edge_from_id):
                    # This edge matches the pair of waypoints! Add it the edge list and continue.
                    return map_pb2.Edge.Id(
                        from_waypoint=waypoint2, to_waypoint=waypoint1
                    )
                elif (waypoint2 == edge_to_id) and (waypoint1 == edge_from_id):
                    # This edge matches the pair of waypoints! Add it the edge list and continue.
                    return map_pb2.Edge.Id(
                        from_waypoint=waypoint1, to_waypoint=waypoint2
                    )
        return None

    def dock(self, dock_id):
        """Dock the robot to the docking station with fiducial ID [dock_id]."""
        try:
            # Make sure we're powered on and standing
            self._robot.power_on()
            self.stand()
            # Dock the robot
            self.last_docking_command = dock_id
            blocking_dock_robot(self._robot, dock_id)
            self.last_docking_command = None
            return True, "Success"
        except Exception as e:
            return False, str(e)

    def undock(self, timeout=20):
        """Power motors on and undock the robot from the station."""
        try:
            # Maker sure we're powered on
            self._robot.power_on()
            # Undock the robot
            blocking_undock(self._robot, timeout)
            return True, "Success"
        except Exception as e:
            return False, str(e)

    def get_docking_state(self, **kwargs):
        """Get docking state of robot."""
        state = self._docking_client.get_docking_state(**kwargs)
        return state

    ############################################################################
    # Additional Code begins here, these functions are analogs to 
    # spot-sdk\python\examples\graph_nav_command_line\recording_commandline.py

    def should_we_start_recording(self):
        """Helper function to authenticate whether recording can happen"""
        #Step 1: verify that there is no current graph already on the robot
        graph = self._graph_nav_client.download_graph()
        if graph is not None:
            #So the robot already has a graph on it. Now we have to check if this graph has waypoints
            # if it does have waypoints, we should localize to them
            if len(graph.waypoints) > 0:
                localization_state = self._graph_nav_client.get_localization_state()
                if not localization_state.localization.waypoint_id:
                    # Not localized to anything in the map. The best option is to clear the graph or
                    # attempt to localize to the current map.
                    # Returning false since the GraphNav system is not in the state it should be to
                    # begin recording.
                    self._logger.error("Robot already has a map on it. Either clear it or fix localization issues")
                    return False
        # We get here if there is no preloaded map. This means we are good to go!
        return True

    def record(self, *args):
        """Prompts the Robot to record a map of its own motion."""
        should_start_recording = self.should_we_start_recording() #This is run first to give us the ok to record
        if not should_start_recording: #Failed the ready check, report back
            self._logger.error("The system is not in the proper state to start recording.", \
                   "Try using the graph_nav_command_line to either clear the map or", \
                   "attempt to localize to the map.")
            return
        try:
            self._recording_client.start_recording(recording_environment=self._recording_environment) #Attempt the recording procedure
            self._logger.info("Successfully started recording a map.")
        except Exception as err: #Any issue in the start-up process will be redirected here
            self._logger.error("Start recording failed: " + str(err))
        return
    
    def stop_recording(self, *args):
        """Prompts the Robot to stop recording"""
        """Stop or pause recording a map."""
        #Returns a fail if its not recording in the first place
        first_iter = True #Stores the first iteration
        while True: #Keep running every second
            try: #Attempt the stop
                self._recording_client.stop_recording() #Command to stop
                self._logger.info("Successfully stopped recording a map.") #Success
                break
            except bosdyn.client.recording.NotReadyYetError as err:
                # It is possible that we are not finished recording yet due to
                # background processing. Try again every 1 second.
                if first_iter:
                    self._logger.info("Cleaning up recording...")
                first_iter = False
                time.sleep(1.0)
                continue
            except Exception as err:
                self._logger.error("Stop recording failed: " + str(err))
                break
        return
    
    def get_recording_status(self, *args): #Function that tells whether robot is recording or not, for debugging
        """Get the recording service's status."""
        status = self._recording_client.get_record_status()
        if status.is_recording:
            self._logger.info("The recording service is on.")
        else:
            self._logger.info("The recording service is off.")
    
    def _write_bytes(self, filepath, filename, data): #Helper function
        """Write data to a file. Used for all downloading procedures"""
        os.makedirs(filepath, exist_ok=True)
        with open(filepath + filename, 'wb+') as f:
            f.write(data)
            f.close()

    def _write_full_graph(self, graph, download_filepath): #Helper function
        """Download the graph from robot to the specified, local filepath location."""
        graph_bytes = graph.SerializeToString()
        self._write_bytes(download_filepath, '/graph', graph_bytes)

    def _download_and_write_waypoint_snapshots(self, waypoints, download_filepath): #Helper function
        """Download the waypoint snapshots from robot to the specified, local filepath location."""
        num_waypoint_snapshots_downloaded = 0
        for waypoint in waypoints:
            if len(waypoint.snapshot_id) == 0:
                continue
            try:
                waypoint_snapshot = self._graph_nav_client.download_waypoint_snapshot(
                    waypoint.snapshot_id)
            except Exception:
                # Failure in downloading waypoint snapshot. Continue to next snapshot.
                self._logger.error("Failed to download waypoint snapshot: " + waypoint.snapshot_id)
                continue
            self._write_bytes(download_filepath + '/waypoint_snapshots',
                              '/' + waypoint.snapshot_id, waypoint_snapshot.SerializeToString())
            num_waypoint_snapshots_downloaded += 1
            self._logger.info("Downloaded {} of the total {} waypoint snapshots.".format(
                num_waypoint_snapshots_downloaded, len(waypoints)))
            

    def _download_and_write_edge_snapshots(self, edges, download_filepath): #Helper function
        """Download the edge snapshots from robot to the specified, local filepath location."""
        num_edge_snapshots_downloaded = 0
        num_to_download = 0
        for edge in edges:
            if len(edge.snapshot_id) == 0:
                continue
            num_to_download += 1
            try:
                edge_snapshot = self._graph_nav_client.download_edge_snapshot(edge.snapshot_id)
            except Exception:
                # Failure in downloading edge snapshot. Continue to next snapshot.
                self._logger.error("Failed to download edge snapshot: " + edge.snapshot_id)
                continue
            self._write_bytes(download_filepath + '/edge_snapshots', '/' + edge.snapshot_id,
                              edge_snapshot.SerializeToString())
            num_edge_snapshots_downloaded += 1
            self._logger.info("Downloaded {} of the total {} edge snapshots.".format(
                num_edge_snapshots_downloaded, num_to_download))

    def download_recording(self, download_filepath=os.getcwd(), *args): #Download function to use
        """Downloads the graph that has been recorded and writes it into subdirectory"""
        # Filepath for the location to put the downloaded graph and snapshots.
        # What this says is "if no specific file path was given, make a folder that will store it"
        if download_filepath[-1] == "/":
            download_filepath = download_filepath + "downloaded_graph"
        else:
            download_filepath = download_filepath + "/downloaded_graph"
        """Download the graph and snapshots from the robot."""
        graph = self._graph_nav_client.download_graph()
        if graph is None:
            self._logger.error("Failed to download the graph.")
            return
        self._write_full_graph(graph, download_filepath)
        self._logger.info("Graph downloaded with {} waypoints and {} edges".format(
            len(graph.waypoints), len(graph.edges)))
        # Download the waypoint and edge snapshots.
        self._download_and_write_waypoint_snapshots(graph.waypoints, download_filepath)
        self._download_and_write_edge_snapshots(graph.edges, download_filepath)
    
    ############################################################################