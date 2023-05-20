#!/usr/bin/env python3
import rospy, tf
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

import matplotlib.pyplot as plt
import numpy as np
import math

class Map:
    def __init__(self, dim):
        """
        Initializes the map
        """
        self.dim = dim # map dimension
        
        # x and y positions of the grid cells (in meters)
        self.xposition = np.zeros([self.dim,self.dim])
        self.yposition = np.zeros([self.dim,self.dim])

        for i in range(0,self.dim):
            for j in range(0,self.dim):
                self.xposition[i,j] = (20/self.dim)*(j) - 10 + (20/self.dim)/2
                self.yposition[i,j] = (20/self.dim)*(i) - 10 + (20/self.dim)/2

        # initialize the map as an empty map (no obstacles)
        self.map = np.zeros([self.dim,self.dim,3])

        plt.ion()
        plt.show()

    def update_map(self, my_robot):
        z = my_robot.lid_measur
        """
        Updates and plots the map
        """

        # iterate over the grid cells of the map
        for i in range(0,self.dim):
            for j in range(0,self.dim):
                # calculate the distance r between grid cell [i,j] and the robot 
                r = math.sqrt((self.xposition[i,j]-my_robot.Curr_Pos[0])**2 + (self.yposition[i,j]-my_robot.Curr_Pos[1])**2)
                # calculate angle phi, the difference in angle between the orientation of the robot from one side and the 
                # line connecting the robot position to grid cell [i,j] from the other side
                phi = math.atan2(self.yposition[i,j]-my_robot.Curr_Pos[1], self.xposition[i,j]-my_robot.Curr_Pos[0]) - my_robot.Curr_Pos[2]
                # find which lidar measurement is pointing towards the grid cell [i,j]
                k = np.argmin(np.abs(my_robot.lid_angles-phi))

                # if distance r is out of the sensor's range or out of the sensor reading + its error/2, do not update the state of grid cell [i,j]
                # OR if the difference in angle between phi and the considered sensor measurement is bigger than the sensor measurement cone angle, do not update the state of grid cell [i,j]
                if (r > np.min([30,z[k]]) + 0.15) or (np.abs(phi-my_robot.lid_angles[k]) > 0.5):
                    pass
                # if sensor measurement is less than sensor range AND sensor measurement is almost equal to distance between robot and consider grid cell
                elif (z[k] < 30) and (np.abs(r-z[k]) < 0.15) and (self.map[i,j,0] < 100):
                    self.map[i,j] = self.map[i,j] + 25 # update state of cell [i,j] as being occupied
                # if the considered grid cell is closer than the sensor reading value
                elif r < z[k] and self.map[i,j,0] > 0:
                    self.map[i,j] = self.map[i,j] - 25 # update cell [i,j] as being free

                if self.map[i,j,0]==0 and self.map[i,j,1]==100: #old location of the robot (green point)
                    self.map[i,j,1] = 0 # return it black

        # Find and place robot on map
        # find the robot's cell on the map
        robot_i,robot_j = self.find_robot_in_map(my_robot)
        # color the robot's cell as green
        self.map[robot_i,robot_j,0] = 0
        self.map[robot_i,robot_j,1] = 100
        self.map[robot_i,robot_j,2] = 0

        # Plot the map
        map2 = np.transpose(self.map,(1,0,2))
        plt.imshow(map2,cmap='brg',vmin=0, vmax=100)  # grayscale: gray / rgb: brg
        plt.draw()
        # print(f"New Map")
        plt.pause(0.01)

        # remove robot point from map
        self.map[robot_i,robot_j,0] = 0
        self.map[robot_i,robot_j,1] = 100
        self.map[robot_i,robot_j,2] = 0



    def find_robot_in_map(self, my_robot):
        """
        Finds the x and y indices corresponding to the cell of the grid-based map where the robot is located.
        Input: Robot object.
        Output: Index of the cell the robot is located at.
        """

        # find the index that minimizes the difference between the robot's position and the coordinates of the cells
        robot_i = np.argmin(np.abs(self.xposition[1,:] - my_robot.Curr_Pos[0]))
        robot_j = np.argmin(np.abs(self.yposition[:,1] - my_robot.Curr_Pos[1]))

        return robot_j,robot_i
    

###########################################################################################################


class Robot:
    def __init__(self):
        self.Curr_Pos = [0,0,0]
        self.Curr_Pos_world = [0,0,0]
        self.first_scan = 1

    def get_world_pose(self, world_pose_msg: LinkStates):
        """
        Updates the variables representing the position of the robot
        """
        # Note: husky_base_link is number 1 if using the empty plane, number 36 if using the playpen

        # Update the world x, y, and theta of the robot
        real_x = world_pose_msg.pose[36].position.x
        real_y = world_pose_msg.pose[36].position.y
        real_orient = tf.transformations.euler_from_quaternion((world_pose_msg.pose[36].orientation.x, world_pose_msg.pose[36].orientation.y, world_pose_msg.pose[36].orientation.z, world_pose_msg.pose[36].orientation.w))[2]

        # # If you want to update the robot's position using real world (gazebo) data
        self.Curr_Pos_world = [real_x, real_y, real_orient]
        self.Curr_Pos = self.Curr_Pos_world

    def get_odometry_pose(self, odometry_pose_msg: Odometry):
        odom_x = odometry_pose_msg.pose.pose.position.x
        odom_y = odometry_pose_msg.pose.pose.position.y
        odom_orient = tf.transformations.euler_from_quaternion(( odometry_pose_msg.pose.pose.orientation.x, odometry_pose_msg.pose.pose.orientation.y, odometry_pose_msg.pose.pose.orientation.z, odometry_pose_msg.pose.pose.orientation.w))[2]

        # update robot's current pose
        self.Cur_Odom_Pos = [odom_x, odom_y, odom_orient]


    def update_lidar_measurements(self, lid_data: LaserScan):
        """
        Updates the lidar measurements used in plotting the map.
        This function is called whenever a message is recieved from the lidar. 
        """
        if self.first_scan:
            print("First scan")
            self.first_scan = 0
            lid_min_angl = lid_data.angle_min
            lid_max_angl = lid_data.angle_max
            self.lid_angles = -np.linspace(lid_min_angl,lid_max_angl,720)
        
        # Update lidar measurements
        self.lid_measur = lid_data.ranges


if __name__ == '__main__':
    
    my_map = Map(200)
    my_robot = Robot()
    
    rospy.init_node("husky_slam_node")
    
    sub = rospy.Subscriber("/gazebo/link_states", LinkStates, callback = my_robot.get_world_pose)
    husky_odom_sub = rospy.Subscriber("/husky_velocity_controller/odom", Odometry, callback = my_robot.get_odometry_pose)
    lidar_sub = rospy.Subscriber("/front/scan", LaserScan, callback = my_robot.update_lidar_measurements)

    rospy.loginfo("Node has been started")
    rospy.sleep(2)
    
    while not rospy.is_shutdown():
        my_map.update_map(my_robot)
        rospy.sleep(1)
        print('Move')
        rospy.sleep(5)
        print("Stop")
        rospy.sleep(3)
