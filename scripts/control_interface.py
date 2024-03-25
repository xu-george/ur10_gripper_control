#!/usr/bin/python3

"""
a code to combine the interface of moveit and gripper urcap control
provide: 
1. gripper control -- (0, 1), 
2. get gripper status -- (0, 1) 
3. ee control -- pose,orn, a .move to point.  b. achieve with waypoints
4. joint control, 
5. get joint status 
6. get ee pose 
7. trajectory execution
"""
""""
TODO: -- add virtual ground plane
"""
import rospy
import time
import numpy as np
from geometry_msgs.msg import Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import RobotTrajectory, DisplayTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from cmodel_urcap import RobotiqCModelURCap

class ControlInterface:

    def __init__(self, ip ='192.168.147.169'):

        self.arm = MoveGroupCommander("manipulator")
        self.end_effector_link = self.arm.get_end_effector_link() 
        self.gripper = RobotiqCModelURCap(ip)

        self.home_joint_values = [0, -1.57, 1.57, -1.57, -1.57, 0]
        self._init_robot()

    def _init_robot(self):
        """
        initialize the robot -- moveit group and gripper
        """
        # initial movit group for arm           
        self.arm.set_pose_reference_frame('base_link')  
        self.arm.allow_replanning(True)                
        self.arm.set_planning_time(1)               
        self.arm.set_goal_position_tolerance(0.0005)     
        self.arm.set_goal_orientation_tolerance(0.001)
        self.arm.set_max_velocity_scaling_factor(0.2)       
        self.arm.set_max_acceleration_scaling_factor(0.1) 

        # initial gripper -- activate the gripper and auto calibrate
        self.gripper.activate()        
        self.gripper.move_and_wait_for_pos(self.gripper.get_max_position(), 255, 255)  # close the gripper

        # move the robot to home position
        self.move_joint(self.home_joint_values)

    def get_gripper_status(self):
        """
        get the current status of gripper
        """
        gripper_pos = self.gripper.get_current_position()
        min = self.gripper.get_min_position()
        max = self.gripper.get_max_position()
        # map to 0-1
        return (gripper_pos - max) / (min - max)
    
    def move_gripper(self, pos, speed=1, force=1):
        """
        move the gripper to the target position 0-1
        """
        # map 0-1 to max-min 
        max = self.gripper.get_max_position()
        min = self.gripper.get_min_position()
        pos = int(pos * (min - max) + max)
        speed = int(speed * 255)
        force = int(force * 255)        
        self.gripper.move_and_wait_for_pos(pos, speed, force)
    
    def get_ee_pose(self):
        """
        get the current pose of end-effector
        """
        pose_orn = self.arm.get_current_pose(self.end_effector_link)
        # change to numpy array
        pose = np.array([pose_orn.pose.position.x, pose_orn.pose.position.y, pose_orn.pose.position.z])
        orn = np.array([pose_orn.pose.orientation.x, pose_orn.pose.orientation.y, pose_orn.pose.orientation.z, pose_orn.pose.orientation.w])
        return pose, orn

    def get_joint_status(self):
        """
        get the current joint status of robot
        """
        return self.arm.get_current_joint_values()
    
    def move_ee(self, pose, orn):
        """
        move the end-effector to the target pose
        pose: [x, y, z]
        orn: [x, y, z, w]
        """
        pose_goal = Pose()

        # update the pose
        pose_goal.position.x = pose[0]
        pose_goal.position.y = pose[1]
        pose_goal.position.z = pose[2]
        pose_goal.orientation.x = orn[0]
        pose_goal.orientation.y = orn[1]
        pose_goal.orientation.z = orn[2]
        pose_goal.orientation.w = orn[3]    

        self.arm.set_pose_target(pose_goal)
        plan = self.arm.plan()
        self.arm.execute(plan, wait=True)
        success = self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()

        return success

    def move_joint(self, joint_values):
        """
        move the robot to the target joint values
        """
        self.arm.set_joint_value_target(joint_values)
        self.arm.go(wait=True)

    def move_ee_with_waypoints(self, pose, orn, speed=0.6):
        """
        move the end-effector to the target pose with waypoints
        pose: [x, y, z]
        orn: [x, y, z, w]
        """
        waypoints = []
        waypoints.append(self.arm.get_current_pose().pose)

        # update the pose
        pose_goal = Pose()
        pose_goal.position.x = pose[0]
        pose_goal.position.y = pose[1]
        pose_goal.position.z = pose[2]
        pose_goal.orientation.x = orn[0]
        pose_goal.orientation.y = orn[1]
        pose_goal.orientation.z = orn[2]
        pose_goal.orientation.w = orn[3]  

        waypoints.append(pose_goal)     
        # plan the trajectory
        self.trajectory_execution(waypoints, speed=speed, wait=True)

    def scale_trajectory_speed(self, traj, spd=0.1):
        new_traj = RobotTrajectory()
        new_traj = traj

        n_joints = len(traj.joint_trajectory.joint_names)
        n_points = len(traj.joint_trajectory.points)

        points = list(traj.joint_trajectory.points)

        for i in range(n_points):
            point = JointTrajectoryPoint()
            point.time_from_start = traj.joint_trajectory.points[i].time_from_start / spd
            point.velocities = list(traj.joint_trajectory.points[i].velocities)
            point.accelerations = list(traj.joint_trajectory.points[i].accelerations)
            point.positions = traj.joint_trajectory.points[i].positions

            for j in range(n_joints):
                point.velocities[j] = point.velocities[j] * spd
                point.accelerations[j] = point.accelerations[j] * spd

            points[i] = point

        new_traj.joint_trajectory.points = points     
        return   new_traj
    
    def trajectory_execution(self, waypoints, speed=0.6, wait=True):

        fraction = 0 
        maxtries = 20
        attempts = 0
		
        # fraction: It indicates the fraction of the specified path that was successfully computed. For example, if fraction is 1.0, it means the entire path was successfully planned. If it's less than 1.0, it indicates that only a portion of the path could be planned, 
        #and the rest might require manual adjustment or replanning.
        
        self.arm.set_start_state_to_current_state()
        while fraction < 1.0 and attempts < maxtries:
            # plan the trajectory with straight line
            (plan, fraction) = self.arm.compute_cartesian_path(waypoints, 0.01, 0.0, True)
	    
            attempts += 1     
            if attempts % 10 == 0:  
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            new_plan=self.scale_trajectory_speed(plan, speed)
            if not wait:
                self.arm.stop()
                self.arm.clear_pose_targets()
            self.arm.execute(new_plan, wait)
            return True
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.") 
            return False             

if __name__ == '__main__':

    rospy.init_node('control', anonymous=True)
    control = ControlInterface(ip='192.168.147.169')

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
    # for i in range(100):
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

    # # get the current joint values
    # joint_values = control.get_joint_status()
    # print("joint values: ", joint_values)

    # # move dx  
    # pose[0] += 0.1
    # control.move_ee(pose, orn)
    # time.sleep(1)
    # pose[0] -= 0.1
    # control.move_ee(pose, orn)
    # time.sleep(1)
    # # move dy
    # pose[1] += 0.1
    # control.move_ee(pose, orn)
    # time.sleep(1)
    # pose[1] -= 0.1
    # control.move_ee(pose, orn)
    # time.sleep(1)
    # # move dz
    # pose[2] += 0.1
    # control.move_ee(pose, orn)
    # time.sleep(1)
    # pose[2] -= 0.1
    # control.move_ee(pose, orn)

    # # orientation test 
    # pose, orn = control.get_ee_pose()
    # # change to euler angle
    # orn = np.array([orn[0], orn[1], orn[2], orn[3]])
    # print("orientation: ", orn) 
    # ToDo rotate around z axis in Euler angle

    # test move with waypoints
    pose[2] -= 0.3
    control.move_ee_with_waypoints(pose, orn, speed=0.1)


