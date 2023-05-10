#!/usr/bin/env python3
import rospy, tf
from gazebo_msgs.msg import LinkStates


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


if __name__ == '__main__':
    
    my_robot = Robot()
    
    rospy.init_node("husky_slam_node")
    
    sub = rospy.Subscriber("/gazebo/link_states", LinkStates, callback = my_robot.get_world_pose)
    
    rospy.loginfo("Node has been started")
    rospy.sleep(2)
    
    while not rospy.is_shutdown():
        print(my_robot.Curr_Pos)
        rospy.sleep(3)
