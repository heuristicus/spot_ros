#!/usr/bin/env python3

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *

from typing import List 
import math
import rospy


def make_marker(mesh_scale=0.001, 
                color=[1.0, 0.0, 1.0, 1.0], 
                mesh=None, **kwargs):
    if not isinstance(mesh_scale, list) and not isinstance(mesh_scale, tuple): 
        mesh_scale = (mesh_scale, mesh_scale, mesh_scale)
    m = Marker()
    if mesh: 
        m.type = Marker.MESH_RESOURCE
        m.mesh_resource = mesh
    m.scale.x, m.scale.y, m.scale.z = tuple(mesh_scale)
    m.color.r, m.color.g, m.color.b, m.color.a = tuple(color)
    return m


def _get_move_control(axis:tuple, name:str)->InteractiveMarkerControl:
    '''Returns a control that moves an interactive marker along the given axis.
    Args:
        axis: A tuple of length 4 representing a quaternion (w, x, y, z).
        name: The name of the control.'''
    c = InteractiveMarkerControl()
    o = c.orientation
    o.w, o.x, o.y, o.z = axis
    c.name = name
    c.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    return c 


def _get_rotation_control(axis:tuple, name:str)->InteractiveMarkerControl:
    '''Returns a control that rotates an interactive marker around the given axis.
    Args:
        axis: A tuple of length 4 representing a quaternion (w, x, y, z).
        name: The name of the control.'''
    c = InteractiveMarkerControl()
    o = c.orientation
    o.w, o.x, o.y, o.z = axis
    c.name = name
    c.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    return c
    
def append_3d_controls(controls, 
                       control_axis:List[str]=['x','y','z','roll','pitch','yaw'], 
                       **kwargs):
    # controls[0].interaction_mode = InteractiveMarkerControl.NONE
    controls[0].interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
    sqrt_2 = math.sqrt(2)
    if 'x' in control_axis:
        controls.append(_get_move_control((sqrt_2, sqrt_2, 0, 0), 
                                           "move_x"))
    if 'y' in control_axis:
        controls.append(_get_move_control((sqrt_2, 0, sqrt_2, 0), 
                                           "move_y"))
    if 'z' in control_axis:
        controls.append(_get_move_control((sqrt_2, 0, 0, sqrt_2), 
                                           "move_z"))
    if 'roll' in control_axis:
        controls.append(_get_rotation_control((sqrt_2, sqrt_2, 0, 0), 
                                               "rotate_x"))
    if 'pitch' in control_axis:
        controls.append(_get_rotation_control((sqrt_2, 0, 0, sqrt_2), 
                                               "rotate_y"))
    if 'yaw' in control_axis:
        controls.append(_get_rotation_control((sqrt_2, 0, sqrt_2, 0), 
                                               "rotate_z"))
    

def add_controls(int_marker, **kwargs):
    '''Returns a control that moves an interactive marker.
    Args:
        scale (Optional): The scale of the marker. Check make_marker for defaults.
        color (Optional): The color of the marker. Check make_marker for defaults.
        mesh (Optional): The mesh of the marker. Check make_marker for defaults.
        axis (Optional): List of axis to move/rotate the marker along. 
            Check append_3d_controls for options.'''
    marker_control =  InteractiveMarkerControl()
    marker_control.always_visible = True
    marker_control.markers.append(make_marker(**kwargs))
    int_marker.controls.append(marker_control)
    append_3d_controls(int_marker.controls, **kwargs)
    return marker_control


def menuCb(feedback):
    rospy.loginfo("button clicked.")
    rospy.loginfo(feedback)

def create_interactive_marker(
        name, axis_scale=0.3, 
        reference_frame='odom',
        position=[1, 1, 1], 
        orientation=[0,0,0,1],
        mesh=None, mesh_scale=0.001,
        control_axis:List[str]=['x','y','z','roll','pitch','yaw']
        ):
    im = InteractiveMarker()
    im.header.frame_id = reference_frame
    p = im.pose.position
    p.x, p.y, p.z = tuple(position)
    o = im.pose.orientation 
    o.x, o.y, o.z, o.w = tuple(orientation)
    im.name, im.scale = name, axis_scale
    add_controls(im, mesh=mesh, 
                 control_axis=control_axis, 
                 mesh_scale=mesh_scale)
    return im
