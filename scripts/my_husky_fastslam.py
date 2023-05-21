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
    def __init__(self, number_of_sample_points = 40):
        self.Curr_Pos = [0,0,0]
        self.Curr_Pos_world = [0,0,0]
        self.Cur_Odom_Pos = [0,0,0]
        self.Old_Odom_Pos = [0,0,0]
        self.first_scan = 1
        self.number_of_sample_points = number_of_sample_points
        self.points_cloud = np.zeros((self.number_of_sample_points, 3))


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



    def calc_sample(self, b):
        """
        This function generates a random variable from a normal distribution with mean 0 and variance b
        """
        # specify the mean and standard deviation of the normal distribution
        mu, sigma = 0, abs(b)

        # generate a random variable from a normal distribution with the specified mean and standard deviation
        return np.random.normal(mu, sigma)

    def sample_motion_model_odometry(self):
        # alpha1, alpha2, alpha3, alpha4 = 0.3, 0.3, 0.3, 0.3
        # alpha1, alpha2, alpha3, alpha4 = 0.015, 0.0022, 0.017, 0.01  # majd
        alpha1, alpha2, alpha3, alpha4 = 0.015, 0.01, 0.050, 0.01  # omar
        """
        alpha1: effect of rotation on rotation error
        alpha2: effect of translation on rotation error
        alpha3: effect of translation on translation error
        alpha4: effect of rotation on translation error
        """

        # print(f"Cur Odom: {self.Cur_Odom_Pos}")
        # print(f"Old Odom: {self.Old_Odom_Pos}")

        # get velocities: step 2-3-4 in algorithm
        delta_rot_1 = math.atan2(self.Cur_Odom_Pos[1]-self.Old_Odom_Pos[1] , self.Cur_Odom_Pos[0]-self.Old_Odom_Pos[0]) - self.Old_Odom_Pos[2]
        delta_trans = math.sqrt( (self.Cur_Odom_Pos[1]-self.Old_Odom_Pos[1])**2 + (self.Cur_Odom_Pos[0]-self.Old_Odom_Pos[0])**2 )
        delta_rot_2 = self.Cur_Odom_Pos[2] - self.Old_Odom_Pos[2] - delta_rot_1

        for j in range(self.number_of_sample_points):
            # Subtract Errors: Steps 5-6-7
            delta_rot_1_hat = delta_rot_1 - self.calc_sample(alpha1*delta_rot_1 + alpha2*delta_trans)
            delta_trans_hat = delta_trans - self.calc_sample(alpha3*delta_trans + alpha4*(delta_rot_1+delta_rot_2))
            delta_rot_2_hat = delta_rot_2 - self.calc_sample(alpha1*delta_rot_2 + alpha2*delta_trans)

            # Compute Cloud: Step 8-9-10
            self.points_cloud[j][0] = self.points_cloud[j][0] + delta_trans_hat*math.cos(self.points_cloud[j][2]+delta_rot_1_hat)
            self.points_cloud[j][1] = self.points_cloud[j][1] + delta_trans_hat*math.sin(self.points_cloud[j][2]+delta_rot_1_hat)
            self.points_cloud[j][2] = self.points_cloud[j][2] + delta_rot_1_hat + delta_rot_2_hat

        # print(self.points_cloud)
        self.Old_Odom_Pos = self.Cur_Odom_Pos

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
