#!/usr/bin/env python

import rospy, time
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from copy import deepcopy
from tf.transformations import quaternion_from_euler
import random
import rospkg


rospack = rospkg.RosPack()
# Add package path for models
path = rospack.get_path('takshak')

# print(path)

sdf_model1 = open(path + '/models/aruco_marker_0/model.sdf', 'r').read()
sdf_model2 = open(path + '/models/aruco_marker_1/model.sdf', 'r').read()
sdf_model3 = open(path + '/models/aruco_marker_2/model.sdf', 'r').read()
sdf_model4 = open(path + '/models/aruco_marker_3/model.sdf', 'r').read()
sdf_model5 = open(path + '/models/aruco_marker_4/model.sdf', 'r').read()

def create_model_request(sdf_model, modelname, px, py, pz, rr, rp, ry):
    """Create a SpawnModelRequest with the parameters of the model given.
    modelname: name of the model for gazebo
    px py pz: position of the model (and it's collision model)
    rr rp ry: rotation (roll, pitch, yaw) of the model
    sx sy sz: size of the model"""
    model = deepcopy(sdf_model)
    req = SpawnModelRequest()
    req.model_name = modelname
    req.model_xml = model
    req.initial_pose.position.x = px
    req.initial_pose.position.y = py
    req.initial_pose.position.z = pz
    q = quaternion_from_euler(rr, rp, ry)
    req.initial_pose.orientation.x = q[0]
    req.initial_pose.orientation.y = q[1]
    req.initial_pose.orientation.z = q[2]
    req.initial_pose.orientation.w = q[3]
    return req


if __name__ == '__main__':
    spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    rospy.loginfo('Waiting for /gazebo/spawn_sdf_model service...')
    spawn_srv.wait_for_service()
    rospy.loginfo('Connected to service!')
    
    pose = [-3, -5, -7, -9, -11]
    random.shuffle(pose)
    
    rospy.loginfo('Spawning marker1')
    req1 = create_model_request(sdf_model1, 'marker1', pose[0], 5.6, 0.4, 1.57, -1.57, 0.0)
    spawn_srv.call(req1)
    
    rospy.loginfo('Spawning marker2')
    req2 = create_model_request(sdf_model2, 'marker2', pose[1], 5.6, 0.4, 1.57, -1.57, 0.0)
    spawn_srv.call(req2)
    
    rospy.loginfo('Spawning marker3')
    req3 = create_model_request(sdf_model3, 'marker3', pose[2], 5.6, 0.4, 1.57, -1.57, 0.0)
    spawn_srv.call(req3)

    rospy.loginfo('Spawning marker4')
    req4 = create_model_request(sdf_model4, 'marker4', pose[3], 5.6, 0.4, 1.57, -1.57, 0.0)
    spawn_srv.call(req4)

    rospy.loginfo('Spawning marker5')
    req5 = create_model_request(sdf_model5, 'marker5', pose[4], 5.6, 0.4, 1.57, -1.57, 0.0)
    spawn_srv.call(req5)
    
    rospy.sleep(10.0)