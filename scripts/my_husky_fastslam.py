#!/usr/bin/env python3
import rospy, tf
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import LaserScan

import numpy as np


class Robot:
    def __init__(self):
        self.Curr_Pos = [0,0,0]
        self.Curr_Pos_world = [0,0,0]

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
    
    my_robot = Robot()
    
    rospy.init_node("husky_slam_node")
    
    sub = rospy.Subscriber("/gazebo/link_states", LinkStates, callback = my_robot.get_world_pose)
    lidar_sub = rospy.Subscriber("/front/scan", LaserScan, callback = my_robot.update_lidar_measurements)

    rospy.loginfo("Node has been started")
    rospy.sleep(2)
    
    while not rospy.is_shutdown():
        print(my_robot.Curr_Pos)
        rospy.sleep(3)
