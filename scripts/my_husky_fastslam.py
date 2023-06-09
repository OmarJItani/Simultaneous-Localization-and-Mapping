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

        # Find and place the sampling particles on map
        # for each particle
        for ind in range(my_robot.number_of_sample_points):
            # find the particle's cell on map
            particle_i,particle_j = self.find_particles_in_map(my_robot,ind)
            # color the particle's cell as red
            self.map[particle_i,particle_j,0] = 100
            self.map[particle_i,particle_j,1] = 0
            self.map[particle_i,particle_j,2] = 0

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

        # remove particles from map
        for ind in range(my_robot.number_of_sample_points):
            particle_i,particle_j = self.find_particles_in_map(my_robot,ind)
            self.map[particle_i,particle_j,0] = 0
            self.map[particle_i,particle_j,1] = 0
            self.map[particle_i,particle_j,2] = 0



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

    def find_particles_in_map(self, my_robot, ind):
        """
        Finds the x and y indices corresponding to the cell of the grid-based map where the considered sampling particle is located.
        Input: Robot object.
        Output: Index of the cell the sampling particle is located at.
        """

        # find the index that minimizes the difference between the particle's position and the coordinates of the cells

        particle_i = np.argmin(np.abs(self.xposition[1,:] - my_robot.points_cloud[ind,0]))
        particle_j = np.argmin(np.abs(self.yposition[:,1] - my_robot.points_cloud[ind,1]))

        return particle_j,particle_i

########################################################################


class Robot:
    def __init__(self, number_of_sample_points = 40):
        self.Curr_Pos = [0,0,0]
        self.Curr_Pos_localize = [0,0,0]
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
        # self.Curr_Pos = self.Curr_Pos_world

        # # publish an updated velocity command
        # self.update_velocity_command()

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


    def cast_rays(self, my_map, x_particle, y_particle, number_of_rays, Zmin=1.0, Zmax=30.0, pixelToMeterRatio=0.1):

        # Get the dimensions of the image
        image_height, image_width = my_map.dim, my_map.dim

        x_particle = np.argmin(np.abs(my_map.xposition[1,:] - x_particle))
        y_particle = np.argmin(np.abs(my_map.yposition[:,1] - y_particle))
        # print(f"x,y: {x_particle,y_particle}")

        r = np.linspace(Zmin/pixelToMeterRatio, Zmax/pixelToMeterRatio, 1000)

        # angles are measured in the inertial frame (inertial x-axis is zero, anti-clockwise is positive)
        angles = np.linspace(0, -2*np.pi, number_of_rays+1)[:-1]
        ranges = np.zeros(number_of_rays)

        for i in range(number_of_rays):
            theta = angles[i]
            
            x = x_particle + r*np.cos(theta)
            y = y_particle + r*np.sin(theta)

            xint = np.int32(np.round(x))
            yint = np.int32(np.round(y))

            # use numpy.where() to find the indices of points that lie within the image boundaries
            valid_indices = np.where((xint >= 0) & (xint < image_width) & (yint >= 0) & (yint < image_height))
            # use the resulting indices to get the subset of x and y coordinates that lie within the image
            valid_xint = xint[valid_indices]
            valid_yint = yint[valid_indices]

            # Extract the pixel values at the points that lie within the image boundaries
            new_set = my_map.map[valid_xint, valid_yint, 0]

            # print(new_set.size)
            # If there are any valid points
            if new_set.size:
                # Find the index of the point with the highest pixel value
                intersection_index = np.argmax(new_set)
                # Get the coordinates of the point with the highest pixel value
                px, py = valid_xint[intersection_index], valid_yint[intersection_index]

                # # draw ray in green
                # my_map.map[valid_xint, valid_yint, 1] = 100
                
                if my_map.map[px, py, 0] == 0: # if no obstacle found, set range to Zmax
                    ranges[i] = Zmax
                else: # if obstacle found, set range to euclidean distance
                    ranges[i] = math.sqrt( (px - x_particle)**2 + (py - y_particle)**2  )*pixelToMeterRatio

            else: # the ray did not hit any obstacle in the image 
                ranges[i] = Zmax

        # image2 = np.flipud(np.transpose(my_map.map,(1,0,2)))
        # plt.imshow(image2,cmap='gray',vmin=0, vmax=1)  # grayscale: gray / rgb: brg

        # plt.draw()
        # # print(f"New Map")
        # plt.pause(0.01)

        return angles, ranges

    def measurement_model(self, Zmax=30.0, number_of_rays=4):
        # ztkstar is from raycasting
        # ztk is from lidar

        zt = self.lid_measur

        w_hit, w_short, w_max, w_rand = 0.95, 0.01, 0.02, 0.02 # sum of weights should be equal to 1
        lambda_short=0.3
        sigma_hit=0.25

        particles_prob = np.zeros(self.number_of_sample_points)
        # print("new set of particles")
        for particle_ind, particle in enumerate(self.points_cloud):
            angles, ranges = self.cast_rays(my_map, particle[0], particle[1], number_of_rays)

            # print(angles)
            # print(ranges)
            # print(f"At {round(self.lid_angles[120],2)}: {zt[120]} At {round(self.lid_angles[600],2)}: {zt[600]}")

            ray_prob = np.zeros(number_of_rays) # probabilities array

            # For each ray
            for ray in range(number_of_rays):

                # get the lidar direction k that correspond to the considered ray
                ztkstar = ranges[ray]
                k = np.argmin(np.abs((self.lid_angles-self.Curr_Pos[2]) - angles[ray])) # make sure that the lidar and the simulates lidar(raycasting) have same angles
                ztk = zt[k]


                # Compute probabilities
                # Compute p_hit
                if ztk > 0 and ztk < Zmax:
                    eta_hit = 1
                    N = (1 / (math.sqrt(2 * math.pi * sigma_hit**2))) * math.exp(-0.5 * ((ztk - ztkstar)**2 / sigma_hit**2))
                    p_hit = eta_hit * N
                else:
                    p_hit = 0

                # Compute p_short
                if ztk > 0 and ztk < ztkstar:
                    eta_short = 1 / (1 - math.exp(-lambda_short * ztkstar))
                    p_short = eta_short * lambda_short * math.exp(-lambda_short * ztk)
                else:
                    p_short = 0

                # Compute p_max
                if ztk >= Zmax:
                    p_max = 1
                else:
                    p_max = 0

                # Compute p_rand
                if ztk > 0 and ztk < Zmax:
                    p_rand = 1 / Zmax
                else:
                    p_rand = 0

                # Compute Overall probability for considered ray
                ray_prob[ray] = w_hit*p_hit + w_short*p_short + w_max*p_max + w_rand*p_rand
            
            # Compute probability of the considered particle (by multiplying the probabilities of all rays at the considered particle)
            particles_prob[particle_ind] = np.prod(ray_prob)
        
        return particles_prob


    def sampling_and_localization(self):

        # Resampling
        particles_prob = self.measurement_model()
        # print(f"particles_prob: {particles_prob}")
        # print(f"particles norm: {np.linalg.norm(particles_prob)}")
        # normalized_particles_prob = particles_prob / np.linalg.norm(particles_prob)
        normalized_particles_prob = particles_prob / particles_prob.sum()
        # print(normalized_particles_prob)
        resampled_points_cloud_indices = np.random.choice(range(self.number_of_sample_points), size=self.number_of_sample_points, p=normalized_particles_prob)

        self.points_cloud = self.points_cloud[resampled_points_cloud_indices]

        # Localization
        location_index = np.argmax(normalized_particles_prob)
        self.Curr_Pos_localize = self.points_cloud[location_index]

        self.Curr_Pos_localize[2] = self.Curr_Pos_world[2] # let me try to use the world orientation only
        self.Curr_Pos = self.Curr_Pos_localize


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
    
    my_map = Map(100)
    my_robot = Robot()
 
    rospy.init_node("husky_slam_node")
    
    sub = rospy.Subscriber("/gazebo/link_states", LinkStates, callback = my_robot.get_world_pose)
    husky_odom_sub = rospy.Subscriber("/husky_velocity_controller/odom", Odometry, callback = my_robot.get_odometry_pose)
    lidar_sub = rospy.Subscriber("/front/scan", LaserScan, callback = my_robot.update_lidar_measurements)
    
    rospy.loginfo("Node has been started")
    rospy.sleep(2)
    
    while not rospy.is_shutdown():
        my_map.update_map(my_robot)
        my_robot.sample_motion_model_odometry()
        my_robot.sampling_and_localization()

        print(f"Current_Pose_World: {[round(my_robot.Curr_Pos[0],2) , round(my_robot.Curr_Pos[1],2) , round(my_robot.Curr_Pos[2],2)]}")
        print(f"Current_Pose_Local: {[round(my_robot.Curr_Pos_localize[0],2) , round(my_robot.Curr_Pos_localize[1],2) , round(my_robot.Curr_Pos_localize[2],2)]}")

        rospy.sleep(1)
        print('Move')
        rospy.sleep(5)
        print("Stop")
        rospy.sleep(3)
        

    # rospy.spin()