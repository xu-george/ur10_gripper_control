#!/usr/bin/python3

"""
a code to combine the interface of moveit and gripper urcap control
provide: 
1. gripper control -- (0, 1), 
2. get gripper status -- (0, 1) 
3. controller ee with pybullet ik
4. joint control, 
5. get joint status 
6. get ee pose 
7. trajectory execution
"""
""""
TODO: -- add virtual ground plane, test trajectory execution
"""
import rospy
import pybullet as p
from collections import namedtuple
import numpy as np
from control_interface import ControlInterface

# get current path
import os
path = os.path.dirname(os.path.realpath(__file__))
robot_urdf = os.path.join(path, 'urdf/model.urdf')

class BulletControlInterface(ControlInterface):

    def __init__(self, ip ='192.168.147.169'):
        super(BulletControlInterface, self).__init__(ip)
        # initialize the pybullet
        p.connect(p.DIRECT)
        # load the urdf
        self.robotId = p.loadURDF(robot_urdf, [0, 0, 0], useFixedBase=True)
        self.endId = 8
        self.arm_num_dofs = 6
        self.get_joint_info()

    def get_joint_info(self):
        numJoints = p.getNumJoints(self.robotId)
        jointInfo = namedtuple('jointInfo', 
            ['id','name','type','damping','friction','lowerLimit','upperLimit','maxForce','maxVelocity','controllable'])
        self.joints = []
        self.controllable_joints = []
        for i in range(numJoints):
            info = p.getJointInfo(self.robotId, i)
            jointID = info[0]
            jointName = info[1].decode("utf-8")
            jointType = info[2]  # JOINT_REVOLUTE, JOINT_PRISMATIC, JOINT_SPHERICAL, JOINT_PLANAR, JOINT_FIXED
            jointDamping = info[6]
            jointFriction = info[7]
            jointLowerLimit = info[8]
            jointUpperLimit = info[9]
            jointMaxForce = info[10]
            jointMaxVelocity = info[11]
            controllable = (jointType != p.JOINT_FIXED)
            if controllable:
                self.controllable_joints.append(jointID)
                p.setJointMotorControl2(self.robotId, jointID, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
            info = jointInfo(jointID,jointName,jointType,jointDamping,jointFriction,jointLowerLimit,
                            jointUpperLimit,jointMaxForce,jointMaxVelocity,controllable)
            self.joints.append(info)

        assert len(self.controllable_joints) >= self.arm_num_dofs
        self.arm_controllable_joints = self.controllable_joints[:self.arm_num_dofs]

        self.arm_lower_limits = [info.lowerLimit for info in self.joints if info.controllable][:self.arm_num_dofs]
        self.arm_upper_limits = [info.upperLimit for info in self.joints if info.controllable][:self.arm_num_dofs]
        self.arm_joint_ranges = [info.upperLimit - info.lowerLimit for info in self.joints if info.controllable][:self.arm_num_dofs]

    def move_ee_pybullet(self, pose, orn):
        # using pybullet ik to calculate the joint values
        joint_poses = p.calculateInverseKinematics(self.robotId, self.endId, pose, orn,
                    )
        
        # get the controlable joint values
        jointPoses = [joint_poses[i] for i in range(self.arm_num_dofs)]
        # move the robot to the target joint values
        self.move_joint(jointPoses)      


if __name__ == '__main__':

    rospy.init_node('control', anonymous=True)
    control = BulletControlInterface(ip='192.168.147.169')

    #----------test the gripper control ----------
    # # move the gripper
    # control.move_gripper(0.5)
    # print(control.get_gripper_status())
    # # get the gripper status
    # time.sleep(2)

    # control.move_gripper(0)
    # print(control.get_gripper_status())
    # time.sleep(2)   

    # # test calibration of fk and ik   
    # for i in range(50):
    #     # fk
    #     pose, orn = control.get_ee_pose()
    #     # ik
    #     control.move_ee(pose, orn)
    # print("done")

    #----------test the ee control ----------
    # get the current pose
    pose, orn = control.get_ee_pose()
    print("position: ", pose)  
    print("orientation: ", orn)

    # test move 
    # pose[0] -= 0.1
    control.move_ee_pybullet(pose, orn)


