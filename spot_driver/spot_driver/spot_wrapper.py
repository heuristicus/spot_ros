import os
import time
import subprocess
import numpy as np

from bosdyn.geometry import EulerZXY
from bosdyn.client import (
    power,
    create_standard_sdk,
    ResponseError,
    RpcError,
)
from bosdyn.client.async_tasks import AsyncPeriodicQuery, AsyncTasks
from bosdyn.client.frame_helpers import (
    BODY_FRAME_NAME,
    VISION_FRAME_NAME,
    ODOM_FRAME_NAME,
    get_odom_tform_body,
    get_vision_tform_body,
    add_edge_to_tree,
    get_a_tform_b,
)
import bosdyn.client.math_helpers as math_helpers
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder
from bosdyn.client.power import PowerClient
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.image import ImageClient, build_image_request
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.local_grid import LocalGridClient


from bosdyn.client.world_object import WorldObjectClient
from bosdyn.client.world_object import (
    make_add_world_object_req,
    make_change_world_object_req,
    make_delete_world_object_req,
)
from bosdyn.api import world_object_pb2


from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.api import (
    image_pb2,
    geometry_pb2,
    trajectory_pb2,
)
from bosdyn.api import basic_command_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.api.geometry_pb2 import SE2Velocity, SE2VelocityLimit, Vec2
from bosdyn.api.trajectory_pb2 import SE2Trajectory
import bosdyn.api.robot_state_pb2 as robot_state_proto
from bosdyn.util import now_timestamp

from google.protobuf.timestamp_pb2 import Timestamp

front_image_sources = [
    'frontleft_fisheye_image',
    'frontright_fisheye_image',
    'frontleft_depth',
    'frontright_depth',
]
"""List of image sources for front image periodic query"""
side_image_sources = [
    'left_fisheye_image',
    'right_fisheye_image',
    'left_depth',
    'right_depth',
]
"""List of image sources for side image periodic query"""
rear_image_sources = ['back_fisheye_image', 'back_depth']
"""List of image sources for rear image periodic query"""

OBSTACLE_BASE_PADDING = 0.5  # m
VELOCITY_BASE_SPEED = 0.5  # m/s
VELOCITY_BASE_ANGULAR = 0.8  # rad/sec
VELOCITY_CMD_DURATION = 0.6  # seconds
TRAJECTORY_CMD_DURATION = 30  # seconds
BASE_FRAME_NAME = ODOM_FRAME_NAME


class AsyncLocalGrid(AsyncPeriodicQuery):
    """Class to get robot local_grid at regular intervals.  get_local_grids_async query sent to the robot at every tick.  Callback registered to defined callback function.

        Attributes:
            client: The Client to a service on the robot
            logger: Logger object
            rate: Rate (Hz) to trigger the query
            callback: Callback function to call when the results of the query are available
    """
    _local_grid_type_names = ['intensity',
                              #   'terrain',
                              #   'terrain_valid',
                              #   'no_step',
                              #   'obstacle_distance',
                              ]

    def __init__(self, client, logger, rate, callback):
        super(AsyncLocalGrid, self).__init__("local-grid", client, logger,
                                             period_sec=1.0/max(rate, 1.0))
        self._callback = None
        if rate > 0.0:
            self._callback = callback

    def _start_query(self):
        if self._callback:
            callback_future = self._client.get_local_grids_async(
                local_grid_type_names=self._local_grid_type_names,
            )
            callback_future.add_done_callback(self._callback)
            return callback_future


class AsyncGraphNav(AsyncPeriodicQuery):
    """Class to get robot graph_nav at regular intervals.  get_localization_state_async query sent to the robot at every tick.  Callback registered to defined callback function.

        Attributes:
            client: The Client to a service on the robot
            logger: Logger object
            rate: Rate (Hz) to trigger the query
            callback: Callback function to call when the results of the query are available
    """

    def __init__(self, client, logger, rate, callback):
        super(AsyncGraphNav, self).__init__("graph-nav", client, logger,
                                            period_sec=1.0/max(rate, 1.0))
        self._callback = None
        if rate > 0.0:
            self._callback = callback

    def _start_query(self):
        if self._callback:
            callback_future = self._client.get_localization_state_async(
                request_live_point_cloud=True,
            )
            callback_future.add_done_callback(self._callback)
            return callback_future


class AsyncRobotState(AsyncPeriodicQuery):
    """Class to get robot state at regular intervals.  get_robot_state_async query sent to the robot at every tick.  Callback registered to defined callback function.

        Attributes:
            client: The Client to a service on the robot
            logger: Logger object
            rate: Rate (Hz) to trigger the query
            callback: Callback function to call when the results of the query are available
    """

    def __init__(self, client, logger, rate, callback):
        super(AsyncRobotState, self).__init__("robot-state", client, logger,
                                              period_sec=1.0/max(rate, 1.0))
        self._callback = None
        if rate > 0.0:
            self._callback = callback

    def _start_query(self):
        if self._callback:
            callback_future = self._client.get_robot_state_async()
            callback_future.add_done_callback(self._callback)
            return callback_future


class AsyncMetrics(AsyncPeriodicQuery):
    """Class to get robot metrics at regular intervals.  get_robot_metrics_async query sent to the robot at every tick.  Callback registered to defined callback function.

        Attributes:
            client: The Client to a service on the robot
            logger: Logger object
            rate: Rate (Hz) to trigger the query
            callback: Callback function to call when the results of the query are available
    """

    def __init__(self, client, logger, rate, callback):
        super(AsyncMetrics, self).__init__("robot-metrics", client, logger,
                                           period_sec=1.0/max(rate, 1.0))
        self._callback = None
        if rate > 0.0:
            self._callback = callback

    def _start_query(self):
        if self._callback:
            callback_future = self._client.get_robot_metrics_async()
            callback_future.add_done_callback(self._callback)
            return callback_future


class AsyncLease(AsyncPeriodicQuery):
    """Class to get lease state at regular intervals.  list_leases_async query sent to the robot at every tick.  Callback registered to defined callback function.

        Attributes:
            client: The Client to a service on the robot
            logger: Logger object
            rate: Rate (Hz) to trigger the query
            callback: Callback function to call when the results of the query are available
    """

    def __init__(self, client, logger, rate, callback):
        super(AsyncLease, self).__init__("lease", client, logger,
                                         period_sec=1.0/max(rate, 1.0))
        self._callback = None
        if rate > 0.0:
            self._callback = callback

    def _start_query(self):
        if self._callback:
            callback_future = self._client.list_leases_async()
            callback_future.add_done_callback(self._callback)
            return callback_future


class AsyncImageService(AsyncPeriodicQuery):
    """Class to get images at regular intervals.  get_image_from_sources_async query sent to the robot at every tick.  Callback registered to defined callback function.

        Attributes:
            client: The Client to a service on the robot
            logger: Logger object
            rate: Rate (Hz) to trigger the query
            callback: Callback function to call when the results of the query are available
    """

    def __init__(self, client, logger, rate, callback, image_requests):
        super(AsyncImageService, self).__init__("robot_image_service",
                                                client,
                                                logger,
                                                period_sec=1.0/max(rate, 1.0))
        self._callback = None
        if rate > 0.0:
            self._callback = callback
        self._image_requests = image_requests

    def _start_query(self):
        if self._callback:
            callback_future = self._client.get_image_async(
                self._image_requests)
            callback_future.add_done_callback(self._callback)
            return callback_future


class AsyncIdle(AsyncPeriodicQuery):
    """Class to check if the robot is moving, and if not, command a stand with the set mobility parameters

        Attributes:
            client: The Client to a service on the robot
            logger: Logger object
            rate: Rate (Hz) to trigger the query
            spot_wrapper: A handle to the wrapper library
    """

    def __init__(self, client, logger, rate, spot_wrapper):
        super(AsyncIdle, self).__init__("idle", client, logger,
                                        period_sec=1.0/rate)

        self._spot_wrapper = spot_wrapper

    def _start_query(self):
        if self._spot_wrapper._last_stand_command != None:
            self._spot_wrapper._is_sitting = False
            response = self._client.robot_command_feedback(
                self._spot_wrapper._last_stand_command)
            if (response.feedback.mobility_feedback.stand_feedback.status ==
                    basic_command_pb2.StandCommand.Feedback.STATUS_IS_STANDING):
                self._spot_wrapper._is_standing = True
                self._spot_wrapper._last_stand_command = None
            else:
                self._spot_wrapper._is_standing = False

        if self._spot_wrapper._last_sit_command != None:
            self._spot_wrapper._is_standing = False
            response = self._client.robot_command_feedback(
                self._spot_wrapper._last_sit_command)
            if (response.feedback.mobility_feedback.sit_feedback.status ==
                    basic_command_pb2.SitCommand.Feedback.STATUS_IS_SITTING):
                self._spot_wrapper._is_sitting = True
                self._spot_wrapper._last_sit_command = None
            else:
                self._spot_wrapper._is_sitting = False

        is_moving = False

        if self._spot_wrapper._last_motion_command_time != None:
            if time.time() < self._spot_wrapper._last_motion_command_time:
                is_moving = True
            else:
                self._spot_wrapper._last_motion_command_time = None

        if self._spot_wrapper._last_motion_command != None:
            response = self._client.robot_command_feedback(
                self._spot_wrapper._last_motion_command)
            if (response.feedback.mobility_feedback.se2_trajectory_feedback.status ==
                    basic_command_pb2.SE2TrajectoryCommand.Feedback.STATUS_GOING_TO_GOAL):
                is_moving = True
            else:
                self._spot_wrapper._last_motion_command = None

        self._spot_wrapper._is_moving = is_moving

        if self._spot_wrapper.is_standing and not self._spot_wrapper.is_moving:
            self._spot_wrapper.stand(False)


def create_mobility_params(body_height=0.0,
                           footprint_R_body=EulerZXY(),
                           locomotion_hint=spot_command_pb2.HINT_AUTO,
                           stair_hint=False,
                           external_force_params=None,
                           obstacle_padding=None,
                           speed_limit=None,
                           angular_speed_limit=None):
    """ Functionality of 'RobotCommandBuilder.mobility_params' with added
        obstacle padding and velocity limit.
    """
    if obstacle_padding is None:
        obstacle_padding = OBSTACLE_BASE_PADDING

    if speed_limit is None:
        speed_limit = VELOCITY_BASE_SPEED

    if angular_speed_limit is None:
        angular_speed_limit = VELOCITY_BASE_ANGULAR

    obstacles = spot_command_pb2.ObstacleParams(
        disable_vision_body_obstacle_avoidance=False,
        disable_vision_foot_obstacle_avoidance=False,
        disable_vision_foot_constraint_avoidance=False,
        obstacle_avoidance_padding=obstacle_padding,
    )

    vel_limit = SE2VelocityLimit(
        max_vel=SE2Velocity(
            linear=Vec2(x=speed_limit,
                        y=speed_limit),
            angular=angular_speed_limit),
    )

    # Simplified body control params
    position = geometry_pb2.Vec3(z=body_height)
    rotation = footprint_R_body.to_quaternion()
    pose = geometry_pb2.SE3Pose(position=position, rotation=rotation)
    point = trajectory_pb2.SE3TrajectoryPoint(pose=pose)
    traj = trajectory_pb2.SE3Trajectory(points=[point])
    body_control = spot_command_pb2.BodyControlParams(
        base_offset_rt_footprint=traj)
    return spot_command_pb2.MobilityParams(
        body_control=body_control,
        locomotion_hint=locomotion_hint,
        stair_hint=stair_hint,
        external_force_params=external_force_params,
        vel_limit=vel_limit,
        obstacle_params=obstacles)


class SpotWrapper():
    """Generic wrapper class to encompass release 1.1.4 API features as well as maintaining leases automatically"""

    def __init__(self,
                 username,
                 password,
                 hostname,
                 logger,
                 rates=None,
                 callbacks=None):
        self._username = username
        self._password = password
        self._hostname = hostname
        self._logger = logger
        if rates is None:
            self._rates = {}
        else:
            self._rates = rates
        if callbacks is None:
            self._callbacks = {}
        else:
            self._callbacks = callbacks
        self._keep_alive = True
        self._valid = False

        self._mobility_params = create_mobility_params()
        self._is_standing = False
        self._is_sitting = True
        self._is_moving = False
        self._last_stand_command = None
        self._last_sit_command = None
        self._last_motion_command = None
        self._last_motion_command_time = None

        self._front_image_requests = []
        for source in front_image_sources:
            self._front_image_requests.append(
                build_image_request(
                    source,
                    image_format=image_pb2.Image.Format.Value('FORMAT_RAW')))

        self._side_image_requests = []
        for source in side_image_sources:
            self._side_image_requests.append(
                build_image_request(
                    source,
                    image_format=image_pb2.Image.Format.Value('FORMAT_RAW')))

        self._rear_image_requests = []
        for source in rear_image_sources:
            self._rear_image_requests.append(
                build_image_request(
                    source,
                    image_format=image_pb2.Image.Format.Value('FORMAT_RAW')))

        # Blocking until connection is made
        try:
            self._connect_to_spot()
        except RuntimeError as e:
            self._logger.error(f"Could not connect to Spot: {e}")

        if self._robot:
            # Transforms
            self._transforms = {}

            # Clients
            try:
                self._world_object_client = self._robot.ensure_client(
                    WorldObjectClient.default_service_name)
                self._robot_graph_nav_client = self._robot.ensure_client(
                    GraphNavClient.default_service_name)
                self._robot_local_grid_client = self._robot.ensure_client(
                    LocalGridClient.default_service_name)
                self._robot_state_client = self._robot.ensure_client(
                    RobotStateClient.default_service_name)
                self._robot_command_client = self._robot.ensure_client(
                    RobotCommandClient.default_service_name)
                self._power_client = self._robot.ensure_client(
                    PowerClient.default_service_name)
                self._lease_client = self._robot.ensure_client(
                    LeaseClient.default_service_name)
                self._image_client = self._robot.ensure_client(
                    ImageClient.default_service_name)
                self._estop_client = self._robot.ensure_client(
                    EstopClient.default_service_name)
            except Exception as e:
                self._logger.error(f"Unable to create client service: {e}")
                self._valid = False
                return

            # Async Tasks
            self._async_task_list = []
            self._robot_graph_nav_task = AsyncGraphNav(
                self._robot_graph_nav_client,
                self._logger,
                max(0.0, self._rates.get("graph_nav", 0.0)),
                self._callbacks.get("graph_nav", lambda: None),
            )
            self._robot_local_grid_task = AsyncLocalGrid(
                self._robot_local_grid_client,
                self._logger,
                max(0.0, self._rates.get("local_grid", 0.0)),
                self._callbacks.get("local_grid", lambda: None),
            )
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
            self._front_image_task = AsyncImageService(
                self._image_client,
                self._logger,
                max(0.0, self._rates.get("front_image", 0.0)),
                self._callbacks.get("front_image", lambda: None),
                self._front_image_requests,
            )
            self._side_image_task = AsyncImageService(
                self._image_client,
                self._logger,
                max(0.0, self._rates.get("side_image", 0.0)),
                self._callbacks.get("side_image", lambda: None),
                self._side_image_requests,
            )
            self._rear_image_task = AsyncImageService(
                self._image_client,
                self._logger,
                max(0.0, self._rates.get("rear_image", 0.0)),
                self._callbacks.get("rear_image", lambda: None),
                self._rear_image_requests,
            )
            self._idle_task = AsyncIdle(
                self._robot_command_client,
                self._logger, 10.0, self)

            self._estop_endpoint = None

            self._async_tasks = AsyncTasks(
                [self._robot_graph_nav_task,
                 self._robot_local_grid_task,
                 self._robot_state_task,
                 self._robot_metrics_task,
                 self._lease_task,
                 self._front_image_task,
                 self._side_image_task,
                 self._rear_image_task,
                 self._idle_task]
            )

            self._robot_id = None
            self._lease = None

    def _connect_to_spot(self):
        """Create a connection with Spot (blocking function)"""
        creating_sdk = True
        while creating_sdk:
            try:
                self._sdk = create_standard_sdk('ros_spot')
                creating_sdk = False
            except Exception as e:
                self._logger.error(f"Error creating SDK object: {e}")
                time.sleep(2.0)

        self._robot = self._sdk.create_robot(self._hostname)

        authenticating = True
        while authenticating:
            try:
                self._robot.authenticate(self._username, self._password)
                self._robot.start_time_sync()
                authenticating = False
            except RpcError as err:
                self._logger.warning(
                    f"Failed to communicate with robot: {err}")
                time.sleep(2.0)

        self._valid = True

    @property
    def is_valid(self):
        """Return boolean indicating if the wrapper initialized successfully"""
        return self._valid

    @property
    def id(self):
        """Return robot's ID"""
        return self._robot_id

    @property
    def graph_nav(self):
        """Return latest proto from the _graph_nav_task"""
        return self._robot_graph_nav_task.proto

    @property
    def local_grid(self):
        """Return latest proto from the _local_grid_task"""
        return self._robot_local_grid_task.proto

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
    def time_skew(self):
        """Return the time skew between local and spot time"""
        return self._robot.time_sync.endpoint.clock_skew

    def robotToLocalTime(self, timestamp):
        """Takes a timestamp and an estimated skew and return seconds and nano seconds.

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

        return rtime

    def claim(self):
        """Get a lease for the robot, a handle on the estop endpoint, and the ID of the robot."""
        try:
            self._robot_id = self._robot.get_id()
            self.getLease()
            self.resetEStop()
            return True, "Success"
        except (ResponseError, RpcError) as err:
            self._logger.error(
                f"Failed to initialize robot communication: {err}")
            return False, str(err)

    def updateTasks(self):
        """Loop through all periodic tasks and update their data if needed."""
        self._async_tasks.update()

    def resetEStop(self):
        """Get keepalive for eStop"""
        self._estop_endpoint = EstopEndpoint(self._estop_client, 'ros', 9.0)
        # Set this endpoint as the robot's sole estop.
        self._estop_endpoint.force_simple_setup()
        self._estop_keepalive = EstopKeepAlive(self._estop_endpoint)

    def assertEStop(self, severe=True):
        """Forces the robot into eStop state.

        Args:
            severe: Default True - If true, will cut motor power immediately.  If false, will try to settle the robot on the ground first
        """
        try:
            if severe:
                self._estop_endpoint.stop()
            else:
                self._estop_endpoint.settle_then_cut()

            return True, "Success"
        except:
            return False, "Error"

    def releaseEStop(self):
        """Stop eStop keepalive"""
        if self._estop_keepalive:
            self._estop_keepalive.stop()
            self._estop_keepalive = None
            self._estop_endpoint = None

    def getLease(self, force=True):
        """Get a lease for the robot and keep the lease alive automatically."""
        if force:
            self._lease = self._lease_client.take()
        else:
            self._lease = self._lease_client.acquire()
        self._lease_keepalive = LeaseKeepAlive(self._lease_client)

    def releaseLease(self):
        """Return the lease on the body."""
        if self._lease:
            self._lease_client.return_lease(self._lease)
            self._lease = None

    def release(self):
        """Return the lease on the body and the eStop handle."""
        try:
            self.releaseLease()
            self.releaseEStop()
            return True, "Success"
        except Exception as e:
            return False, str(e)

    def disconnect(self):
        """Release control of robot as gracefully as possible."""
        if self._robot.time_sync:
            self._robot.time_sync.stop()
        self.releaseLease()
        self.releaseEStop()

    def _robot_command(self, command_proto, end_time_secs=None):
        """Generic blocking function for sending commands to robots.

        Args:
            command_proto: robot_command_pb2 object to send to the robot.  Usually made with RobotCommandBuilder
            end_time_secs: (optional) Time-to-live for the command in seconds
        """
        try:
            id = self._robot_command_client.robot_command(
                lease=None,
                command=command_proto,
                end_time_secs=end_time_secs,
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

    def orient(self, roll, pitch, yaw, height, duration):
        orientation = EulerZXY(yaw, roll, pitch)
        end_time = time.time() + duration
        response = self._robot_command(
            RobotCommandBuilder.stand_command(
                body_height=height,
                footprint_R_body=orientation),
            end_time_secs=end_time,
        )
        self._last_stand_command = response[2]
        return response[0], response[1]

    def sit(self):
        """Stop the robot's motion and sit down if able."""
        response = self._robot_command(RobotCommandBuilder.sit_command())
        self._last_sit_command = response[2]
        return response[0], response[1]

    def stand(self, monitor_command=True):
        """If the e-stop is enabled, and the motor power is enabled, stand the robot up."""
        response = self._robot_command(
            RobotCommandBuilder.stand_command(params=self._mobility_params))
        if monitor_command:
            self._last_stand_command = response[2]
        return response[0], response[1]

    def safe_power_off(self):
        """Stop the robot's motion and sit if possible.  Once sitting, disable motor power."""
        response = self._robot_command(
            RobotCommandBuilder.safe_power_off_command())
        return response[0], response[1]

    def power_on(self):
        """Enble the motor power if e-stop is enabled."""
        try:
            power.power_on(self._power_client)
            return True, "Success"
        except:
            return False, "Error"

    def set_mobility_params(self,
                            body_height=0,
                            footprint_R_body=EulerZXY(),
                            locomotion_hint=1,
                            stair_hint=False,
                            external_force_params=None,
                            obstacle_padding=0.5,
                            speed_limit=0.8):
        """Define body, locomotion, and stair parameters.

        Args:
            body_height: Body height in meters
            footprint_R_body: (EulerZXY) – The orientation of the body frame with respect to the footprint frame (gravity aligned framed with yaw computed from the stance feet)
            locomotion_hint: Locomotion hint
            stair_hint: Boolean to define stair motion
            obstacle_padding: padding in meters for the obstacle avoidance
            speed_limit: velocity limit in meter per second
        """
        self._mobility_params = create_mobility_params(
            body_height=body_height,
            footprint_R_body=footprint_R_body,
            locomotion_hint=locomotion_hint,
            stair_hint=stair_hint,
            external_force_params=external_force_params,
            obstacle_padding=obstacle_padding,
            speed_limit=speed_limit)

    def velocity_cmd(self, v_x, v_y, v_rot, cmd_duration=None):
        """Send a velocity motion command to the robot.

        Args:
            v_x: Velocity in the X direction in meters
            v_y: Velocity in the Y direction in meters
            v_rot: Angular velocity around the Z axis in radians
            cmd_duration: (optional) Time-to-live for the command in seconds.  Default is 125ms (assuming 10Hz command rate).
        """
        if cmd_duration is None:
            cmd_duration = VELOCITY_CMD_DURATION

        end_time = time.time() + cmd_duration
        self._robot_command(
            RobotCommandBuilder.velocity_command(
                v_x=v_x,
                v_y=v_y,
                v_rot=v_rot,
                params=self._mobility_params,
            ),
            end_time_secs=end_time,
        )
        self._last_motion_command_time = end_time

    def trajectory_cmd(self,
                       goal_x,
                       goal_y,
                       goal_heading,
                       cmd_duration=None,
                       frame_name=None):
        """Send a trajectory motion command to the robot.

        Args:
            goal_x: Position X coordinate.
            goal_y: Position Y coordinate.
            goal_heading: Pose heading in radians.
            cmd_duration: (optional) Time-to-live for the command in seconds.  Default is 125ms (assuming 10Hz command rate).
        """
        if frame_name is None:
            frame_name = BASE_FRAME_NAME
        if cmd_duration is None:
            cmd_duration = TRAJECTORY_CMD_DURATION

        end_time = time.time() + cmd_duration
        self._robot_command(
            RobotCommandBuilder.trajectory_command(
                goal_x=goal_x,
                goal_y=goal_y,
                goal_heading=goal_heading,
                frame_name=frame_name,
                params=self._mobility_params,
            ),
            end_time_secs=end_time,
        )
        self._last_motion_command_time = end_time

    def get_base_transform(self, base_frame=None):
        if base_frame is None:
            base_frame = BASE_FRAME_NAME

        if base_frame == ODOM_FRAME_NAME:
            base_tform_body = self.get_odom_transform()
        elif base_frame == VISION_FRAME_NAME:
            base_tform_body = self.get_vision_transform()
        else:
            return None
        return base_tform_body

    def get_odom_transform(self):
        odom_tform_body = get_odom_tform_body(
            self._robot_state_client.get_robot_state().kinematic_state.transforms_snapshot)
        return odom_tform_body

    def get_vision_transform(self):
        vision_tform_body = get_vision_tform_body(
            self._robot_state_client.get_robot_state().kinematic_state.transforms_snapshot)
        return vision_tform_body

    def get_frame_transform(self, frame_name, base_frame=None):
        if base_frame is None:
            base_frame = BASE_FRAME_NAME

        base_tform_body = self.get_base_transform(base_frame)
        base_tform_frame = self.get_transform_between(
            base_frame,
            frame_name,
        )
        if base_tform_frame is None:
            return None

        frame_tform_body = base_tform_frame.inverse() * base_tform_body
        return frame_tform_body

    def create_frame(self,
                     frame_name,
                     position_x=0.0,
                     position_y=0.0,
                     heading=0.0,
                     base_frame=None):
        """
        Creates a new frame with respect to the base frame
        via a body to frame transform.

        The positions (pose_x, pos_y) define the translation of
        the (current) body position to the frame.
        The pose_yaw is 'applied' after the translation.

        In other words:
        The pose_x, pose_y and pose_yaw are to be defined as
        if Spot would walk towards the origin of the frame
        and then rotate to match its 'zero-angle'.
        """
        if base_frame is None:
            base_frame = BASE_FRAME_NAME

        base_tform_body = self.get_base_transform(base_frame)
        pose_x, pose_y, pose_yaw = self._calculate_frame_translation(
            position_x, position_y, np.deg2rad(heading),
        )

        body_tform_frame = self.create_SE3Pose(
            x=pose_x,
            y=pose_y,
            z=0.0,
            yaw=pose_yaw,
        )

        self._transforms[frame_name] = {
            'base_frame': base_frame,
            'transform': base_tform_body * body_tform_frame,
        }

    def _calculate_frame_translation(self, x, y, yaw):
        distance = np.sqrt(x**2 + y**2)
        angle = np.arctan2(y, x)

        dx = -distance * np.cos(yaw - angle)
        dy = distance * np.sin(yaw - angle)
        dyaw = -yaw

        return dx, dy, dyaw

    def get_transform_between(self,
                              frame_from,
                              frame_to,
                              world_name='world_obj'):

        if frame_to not in self._transforms:
            return None

        if not frame_from == self._transforms[frame_to]['base_frame']:
            return None

        return self._transforms[frame_to]['transform']

    def create_SE3Pose(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        return math_helpers.SE3Pose(
            x=x,
            y=y,
            z=z,
            rot=math_helpers.Quat.from_yaw(yaw),
        )
