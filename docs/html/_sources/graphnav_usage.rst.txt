GraphNav Service
===========================

`Graph Navigation (GraphNav) <https://dev.bostondynamics.com/docs/concepts/autonomy/graphnav_service>`_
is Spot's native mapping and localization service that allows it to perform higher level Autonomy
features such as Autowalk and Missions.

Autowalk and Missions are not supported by the ROS driver, but the GraphNav map can still be used to
traverse a previously recorded Autowalk path, either recorded through the controller or with the command
line utility.

ROS Services and Actions
------------------------

+----------------------+-----------------------------------------------------------------------------------------------------------------------+
|   Service / Action   |                                                      Description                                                      |
+======================+=======================================================================================================================+
| /spot/navigate_init  | ROS Service. Initialize the starting point of the robot to the map, with a nearby AprilTag fiducial.                  |
+----------------------+-----------------------------------------------------------------------------------------------------------------------+
| /spot/navigate_to    | ROS Action. Go to a waypoint in the current graph, traversing edges formed when recording the map.                    |
+----------------------+-----------------------------------------------------------------------------------------------------------------------+
| /spot/navigate_route | ROS Action. Go to a series of waypoints (route) in the current graph, traversing edges formed when recording the map. |
+----------------------+-----------------------------------------------------------------------------------------------------------------------+

Usage
-----

1. Record a map with the controller or `command line utility <https://github.com/boston-dynamics/spot-sdk/tree/master/python/examples/graph_nav_command_line>`_.
2. Download the map to your computer from the controller, or directly from the robot with the commend line utility.
3. Upload the map to the robot (if rebooting or overwriting an existing map) with the ``/spot/navigate_init`` call, with the current waypoint name and a nearby AprilTag fiducial present. The name of the waypoint can be gotten from the ``/spot/list_graph`` ROS Service call.
4. Start using the ``/spot/navigate_to`` or ``/spot/navigate_route`` ROS Actions to navigate the map.

Demonstration
-------------

In the demonstration below, the robot is initialized to the map with the waypoint "ss" and the AprilTag fiducial 350.
The robot then navigates to each successive waypoint in the graph until it returns to the start point.
The RViz visualization shows the robot's current position and orientation, as well as the depth camera (colour points)
and lidar (white points) data of the environment.

.. image:: images/spot-graphnav.gif