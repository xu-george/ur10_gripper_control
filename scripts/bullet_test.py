#!/usr/bin/python3
import pybullet as p

# get current path
import os
path = os.path.dirname(__file__)

# robot obs path: 
robot_path = path + "/urdf/model.urdf"


import pybullet as p

# Start PyBullet physics server
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(1)

# Load URDF file
robot_id = p.loadURDF(robot_path, [0, 0, 0], useFixedBase=True)


# Find link ID by name
link_name = "tool0"

# Get the number of joints in the robot
num_joints = p.getNumJoints(robot_id)
print("Number of joints in the robot:", num_joints)

# Iterate through all joints and find the joint connected to the link
link_id = -1
for i in range(num_joints):
    joint_info = p.getJointInfo(robot_id, i)
    joint_name = joint_info[1].decode('UTF-8')  # Decode byte string to regular string
    link_name = joint_info[-5]
    print("link_name: ", "id: ", link_name, i)


print("link_id", i)
# Print the link ID of the desired link
if link_id != -1:
    print("Link ID of", link_name, ":", link_id)
else:
    print(link_name, "not found in the URDF.")