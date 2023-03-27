

from bosdyn.api.world_object_pb2 import (
    WorldObject, 
    ListWorldObjectResponse,
)
from bosdyn.client.world_object import (
    WorldObjectClient, 
    make_add_world_object_req,
    make_change_world_object_req,
    make_delete_world_object_req,
)
from bosdyn.client.frame_helpers import (
    # GRAV_ALIGNED_BODY_FRAME_NAME,
    # get_vision_tform_body,
    VISION_FRAME_NAME,
    add_edge_to_tree,
    get_a_tform_b,
)
from bosdyn.client.image import (
    ImageClient, 
    build_image_request,
)


from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.sdk import Robot


from bosdyn.api.image_pb2 import ImageCapture 
from bosdyn.api import geometry_pb2 as geom
from bosdyn.api.geometry_pb2 import SE3Pose, Vec3, Quaternion
from bosdyn.util import now_timestamp

import numpy as np
import quaternion as np_quat

from google.protobuf.wrappers_pb2 import DoubleValue 

class WorldObjectHandler:

    def __init__(self, robot: Robot):
        self._robot = robot
        self._robot.time_sync.wait_for_sync()
        self._wo_client = self._robot.ensure_client(
            WorldObjectClient.default_service_name
        )
        self._state_client = self._robot.ensure_client(
            RobotStateClient.default_service_name
        )
        self._image_client = self._robot.ensure_client(
            ImageClient.default_service_name
        )

        self._ids2names = {}
        self._extra_info = {}
        self._ref = VISION_FRAME_NAME

        self._frame_tree_getters = [
            self._state_frame_tree_getter
        ]
        for p in self._image_sourced_frame_tree_getters():
            self._frame_tree_getters.append(p)

    def _state_frame_tree_getter(self):
        state = self._state_client.get_robot_state()
        return state.kinematic_state.transforms_snapshot
    
    def _image_sourced_frame_tree_getters(self, sources=None):
        if sources is None: 
            sources = [s.name for s in self._image_client.list_image_sources()]
        
        class GetImageFrameTreeFunctor:
            def __init__(self, client, source):
                self._req = build_image_request(source, quality_percent=0.01)
                self._client = client
            def __call__(self):
                i = self._client.get_image([self._req])[0]
                return i.shot.transforms_snapshot
        
        for s in sources:
            yield GetImageFrameTreeFunctor(self._image_client, s)

    def get_all_known_frames(self):
        frames = set()
        for frame_tree_getter in self._frame_tree_getters:
            for f in frame_tree_getter().child_to_parent_edge_map:
                frames.add(f)
        return frames
         
    def _get_frame_tree_with_frame(self, frame):
        '''
        Allows the user to add an object just by knowing the frame name.
        To get the tree the function queries a set of bosdyn spot services.
        TODO: Consider frame trees from other sources.
        '''
        for frame_tree_getter in self._frame_tree_getters:
            frame_tree = frame_tree_getter()
            if frame in frame_tree.child_to_parent_edge_map: 
                return frame_tree
        
        raise RuntimeError('Could not find the frame referenced.'+\
                            'If you are trying to add an object, provide'+\
                            'a frame tree with said object.')


    def add_cam_detected_object(self, name, pose, 
                             shot:ImageCapture, 
                             extra_info:dict,
                             **kwargs):
        '''
        Adds an object detected in with camera. Assigns the object an id.
        
        Returns:
            :id (int): unique identifier assigned by system.
        '''
        return self.add_object_given_tree(name, 
                                          pose, 
                                          shot.frame_name_image_sensor, 
                                          shot.transforms_snapshot, 
                                          extra_info)

    def add_object_given_tree(self, name, pose, relative_frame, 
                              frame_tree:geom.FrameTreeSnapshot, 
                              extra_info:dict, **kwargs):
        '''
        Adds an object given frame tree. Assigns the object an id.
        
        Returns:
            :id (int): unique identifier assigned by system.
        '''
        edges = dict(frame_tree.child_to_parent_edge_map)
        
        if len(pose) == 2: T = self._pose_to_pb2(pose[0], pose[1])
        elif pose.shape[:2] == (4,4): T = self._pose_mat_to_pb2(pose)
        else: raise RuntimeError("Pose format provided is not recognized!")
        
        add_edge_to_tree(edges, T, relative_frame, name)
        snapshot = geom.FrameTreeSnapshot(child_to_parent_edge_map=edges)
        
        add_wo_req = make_add_world_object_req(
            WorldObject(
                name=name, 
                transforms_snapshot=snapshot,
                acquisition_time=now_timestamp())
        )

        response = self._wo_client.mutate_world_objects(add_wo_req)

        id = response.mutated_object_id
        self._ids2names[id] = name 
        self._extra_info[id] = extra_info

        return id

    def add_object(self, name, pose, relative_frame, **kwargs):
        '''Adds object by trying to find the relative frame in known
        frame trees.'''
        frame_tree = self._get_frame_tree_with_frame(relative_frame)
        return self.add_object_given_tree(name,
                                          pose,
                                          relative_frame,
                                          frame_tree,
                                          kwargs)        


    def get_known_frames(self):
        '''
        TODO: currently just for testing.
        NOTE:
            for this to be useful, more than one client need to provide
            frame trees, this would merge them and provide all the frame 
            names in them
        '''
        tree = self._get_frame_tree_with_frame(VISION_FRAME_NAME)
        return [f for f in tree.child_to_parent_edge_map]        

    def _newid(self):
        id = len(self._ids2names) + 1
        self._ids2names[id]=None
        return id

    def get_object(self, name, id=None)->WorldObject:
        if id is None:
            ids = [k for k, v in self._ids2names.items() if v == name]
            if len(ids) != 1:
                raise RuntimeError('Since multiple added objects have the same name\
                                    an ID needs to be passed.')
            elif len(ids) == 0: 
                raise RuntimeError('No object with this name.')
            else: id = ids[0]

        def get_object_with_id(id):
            wos = self._wo_client.list_world_objects()
            for obj in wos.world_objects:
                print(f'looking at obj with id {obj.id}')
                if obj.id == id: return obj
                
        return get_object_with_id(id) 
        
    def get_object_transform(self, name, relative_frame=VISION_FRAME_NAME, id=None)->SE3Pose:
        '''
        TODO: Consider allowing the user to add a frame tree that tree would have to be 
        integrated with the existant tree and allows for reative transforms between all 
        frames in both
        '''
        obj = self.get_object(name, id)
        return get_a_tform_b(obj.transforms_snapshot, name, relative_frame)


    def _pose_to_pb2(self, position:np.array, quaternion:np_quat.quaternion)->SE3Pose:
        return geom.SE3Pose(
            position=geom.Vec3(x=position[0], 
                                       y=position[1], 
                                       z=position[2]),
            rotation=geom.Quaternion(x=quaternion.x, 
                                             y=quaternion.y, 
                                             z=quaternion.z, 
                                             w=quaternion.w)
        )

    def _pose_mat_to_pb2(self, pose_mat)->SE3Pose:
        p = pose_mat[0:3, -1]
        q = np_quat.from_rotation_matrix(pose_mat[0:3, 0:3])
        return self._pose_to_pb2(p, q)
    