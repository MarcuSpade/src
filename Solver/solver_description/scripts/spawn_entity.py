#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

def spawn_entity():
    rospy.init_node('spawn_entity_node')

    # Create a service proxy for the SpawnModel service
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)

    # Define the model path (URDF file) and pose
    model_path = "/home/marcusaraujo/cargo_ws/src/Solver/solver_description/urdf/solver.xacro"
    model_name = "cargobot"
    initial_pose = Pose()
    initial_pose.position.x = 0
    initial_pose.position.y = 0
    initial_pose.position.z = 0
    initial_pose.orientation.x = 0
    initial_pose.orientation.y = 0
    initial_pose.orientation.z = 0
    initial_pose.orientation.w = 1

    # Load the model and specify its pose
    try:
        spawn_model(model_name, open(model_path, 'r').read(), "", initial_pose, "world")
        rospy.loginfo("Successfully spawned {} in Gazebo!".format(model_name))
    except rospy.ServiceException as e:
        rospy.logerr("Failed to spawn {} in Gazebo: {}".format(model_name, e))

if __name__ == '__main__':
    try:
        spawn_entity()
    except rospy.ROSInterruptException:
        pass
