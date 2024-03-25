#!/usr/bin/python3
import rospy
import pygame
from sensor_msgs.msg import Joy


class PubJoystick:
    def __init__(self):
        """
        # A ros node to publish the joystick data to the robot control interface
        """
        rospy.init_node('joystick_publisher', anonymous=True)
        self.pub = rospy.Publisher('joy_values', Joy, queue_size=1)
        self.rate = rospy.Rate(5)

        # initialize the joystick
        pygame.init()
        clock = pygame.time.Clock()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

    def publish(self):
        while not rospy.is_shutdown():
            pygame.event.get()
            joystick_msg = Joy()
            joystick_msg.axes = [self.joystick.get_axis(i) for i in range(2)]
            # dz, gripper_close, gripper_open
            joystick_msg.buttons = [self.joystick.get_hat(0)[1], self.joystick.get_button(0), self.joystick.get_button(1)]
            self.pub.publish(joystick_msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        pub_joystick = PubJoystick()
        pub_joystick.publish()
    except rospy.ROSInterruptException:
        pass