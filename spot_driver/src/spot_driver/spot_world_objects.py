import typing
import logging
import time

from bosdyn.client.robot import Robot
from bosdyn.client import robot_command
from bosdyn.client.world_object import WorldObjectClient, world_objects_pb2
from bosdyn.api import header_pb2
from google.protobuf.timestamp_pb2 import Timestamp


class SpotWorldObjects:
    def __init__(
        self,
        robot: Robot,
        logger: logging.Logger,
        robot_params: typing.Dict[str, typing.Any],
        robot_clients: typing.Dict[str, typing.Any],
    ):
        self._robot = robot
        self._logger = logger
        self._world_object_client: WorldObjectClient = robot_clients[
            "world_object_client"
        ]
        self._robot_params = robot_params

    def get_world_objects(self):
        world_objects = self._world_object_clien.list_world_objects().world_objects
