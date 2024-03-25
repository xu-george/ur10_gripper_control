#!/usr/bin/python3

import rospy
import numpy as np
from sensor_msgs.msg import Joy
from control_interface import ControlInterface 

class JoyControl():
    """
    A ros node to receive the joystick data and control the robot
    """
    def __init__(self, ip="192.168.147.169", min_workspace=[0.5, 0, 0.3], max_workspace=[0.8, 0.4, 0.8]):

        rospy.init_node('joy_control', anonymous=True)
        self.controller = ControlInterface(ip=ip)
        self.min_workspace = min_workspace
        self.max_workspace = max_workspace
        rospy.Subscriber('joy_values', Joy, self.joy_callback)
        rospy.spin()

    def joy_callback(self, data):
        """
        callback function to control the robot
        """
        dx = data.axes[1]*0.005 if abs(data.axes[1]) > 0.1 else 0.0
        dy = data.axes[0]*0.005 if abs(data.axes[0]) > 0.1 else 0.0
        dz = data.buttons[0]*0.01       

        # d_roll = data.axes[2]*0.05 if abs(data.axes[2]) > 0.01 else 0.0

        grippen_open = data.buttons[1]
        gripper_close = data.buttons[2]
        if grippen_open:
            d_gripper = -0.1
        elif gripper_close:
            d_gripper = 0.1
        else:
            d_gripper = 0.0

        self.step([dx, dy, dz, d_gripper])

    def step(self, action):
        """
        step function to control the robot
        action: [dx, dy, dz, d_gripper]
        """
        # # get the current pose and joint values
        pose, orn = self.controller.get_ee_pose()        
        gripper_pos = self.controller.get_gripper_status()
        # # set orientation to be fixed
        # orn = np.array([-0.70979513, 0.70440541, 0.00147084, 0.0013142])

        print("position: ", pose)
        print("action: ", action)

        # update the pose
        pose[0] += action[0]
        pose[1] += action[1]
        pose[2] += action[2]

        gripper_pos += action[3]

        # # limit the workspace
        pose = np.clip(pose, self.min_workspace, self.max_workspace)
        gripper_pos = np.clip(gripper_pos, 0, 1)

        # # update the gripper
        self.controller.move_gripper(gripper_pos)
        # move the end-effector
        self.controller.move_ee_with_waypoints(pose, orn, 1)


if __name__ == '__main__':

    try:
        control_joystick = JoyControl()

    except rospy.ROSInterruptException:
        pass

