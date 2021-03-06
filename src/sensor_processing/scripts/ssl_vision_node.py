#!/usr/bin/env python
from sensor_processing.ssl_receiver import *
import rospy
from robocup_msgs.msg import Position, RobotData
from geometry_msgs.msg import Pose, Twist, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion

NUM_ROBOTS = 6
TEAM_COLOR = 1 # yellow

'''
Class for node object for interfacing with SSL Vision
'''
class SSLVisionNode():
    def __init__(self):
        self.vision_sock = SSLReceiver()
        self.pub = rospy.Publisher('locations', Position, queue_size = 10)
        self.rate = None
        self.last_positions = Position()
        self.last_timestamp = 0.0
        self.camera_check = [False for i in range(4)]
 
    '''
    Initialize and start running resources for node
    '''
    def start(self):
        rospy.init_node('ssl_vision', anonymous = True)
        self.vision_sock.open()
        self.rate = rospy.Rate(10)
        self.last_positions, self.last_timestamp = self.get_data()
 
    '''
    Initialize a robocup_msgs/Position message
    '''   
    def init_msg(self):
        data = Position()
        data.friends = [RobotData() for i in range(NUM_ROBOTS)]
        data.enemies = [RobotData() for i in range(NUM_ROBOTS)]
        data.ball_pos = Pose()
        data.ball_speed = Twist()
        return data

    '''
    Receive data from SSL Vision and convert into robocup_msgs/Position message
    '''
    def get_data(self):
        consistent_data = self.init_msg()
        timestamp = 0
        while not self.are_cameras_checked():
            pkt = self.vision_sock.recv_msg()
            if pkt is not None:
                self.camera_check[pkt.detection.camera_id] = True
                consistent_data = self.get_new_positions(pkt, consistent_data)
                timestamp = pkt.detection.t_capture

        self.camera_check = [False] * 4
        return consistent_data, timestamp

    '''
    To ensure all data is received, make sure that all cameras have sent data
    '''
    def are_cameras_checked(self):
        return self.camera_check[0] and self.camera_check[1] and self.camera_check[2] and self.camera_check[3]
        

    '''
    Update the Pose of a robot
    '''
    def update_positions(self, robot_data, robots):
        for robot in robots:
            robot_data[robot.robot_id].id = robot.robot_id
            robot_data[robot.robot_id].r_pose.position.x = robot.x
            robot_data[robot.robot_id].r_pose.position.y = robot.y
            robot_data[robot.robot_id].r_pose.orientation = Quaternion(*quaternion_from_euler(0, 0, robot.orientation))
        return robot_data

    '''
    Update the Pose for all robots
    '''
    def update_robot_positions(self, msg, pkt, color):
        if color:
            if pkt.detection.robots_yellow:
                msg.friends = self.update_positions(msg.friends, pkt.detection.robots_yellow)
            if pkt.detection.robots_blue:
                msg.enemies = self.update_positions(msg.enemies, pkt.detection.robots_blue)
        else:
            if pkt.detection.robots_blue:
                msg.friends = self.update_positions(msg.friend, pkt.detection.robots_blue)
            if pkt.detection.robots_yellow:
                msg.enemies = self.update_positions(msg.enemies, pkt.detection.robots_yellow)
        return msg

    '''
    Update position of the ball
    '''
    def update_ball_position(self, msg, balls):
        if balls:
            if ((abs(balls[0].x) > 0.05) and (abs(balls[0].y) > 0.05)) or self.are_cameras_checked():
                msg.ball_pos.position.x = balls[0].x
                msg.ball_pos.position.y = balls[0].y
        return msg

    '''
    Update positions of all items on field
    '''
    def get_new_positions(self, pkt, msg):
        self.update_robot_positions(msg, pkt, TEAM_COLOR)
        self.update_ball_position(msg, pkt.detection.balls)
        return msg

    '''
    Update the velocities of all items on field
    '''
    def update_velocity(self, new_positions, new_timestamp):
        time_diff = new_timestamp - self.last_timestamp
        
        # update velocities for our team
        for f in self.last_positions.friends:
            old_x = f.r_pose.position.x
            new_x = new_positions.friends[f.id].r_pose.position.x
            
            old_y = f.r_pose.position.y
            new_y = new_positions.friends[f.id].r_pose.position.y
            
            old_quat = f.r_pose.orientation
            new_quat = new_positions.friends[f.id].r_pose.orientation
        
            old_angles = euler_from_quaternion([old_quat.x, old_quat.y, old_quat.z, old_quat.w])
            new_angles = euler_from_quaternion([new_quat.x, new_quat.y, new_quat.z, new_quat.w])

            new_positions.friends[f.id].r_twist.linear.x = (old_x - new_x)/ time_diff
            new_positions.friends[f.id].r_twist.linear.y = (old_y - new_y)/ time_diff
            new_positions.friends[f.id].r_twist.angular.x = (old_x - new_x)/ time_diff
        
        # update velocities for other team
        for e in self.last_positions.enemies:
            old_x = e.r_pose.position.x
            new_x = new_positions.enemies[e.id].r_pose.position.x
            
            old_y = f.r_pose.position.y
            new_y = new_positions.enemies[e.id].r_pose.position.y
            
            old_quat = f.r_pose.orientation
            new_quat = new_positions.enemies[e.id].r_pose.orientation
        
            old_angles = euler_from_quaternion([old_quat.x, old_quat.y, old_quat.z, old_quat.w])
            new_angles = euler_from_quaternion([new_quat.x, new_quat.y, new_quat.z, new_quat.w])

            new_positions.enemies[e.id].r_twist.linear.x = (old_x - new_x)/ time_diff
            new_positions.enemies[e.id].r_twist.linear.y = (old_y - new_y)/ time_diff
            new_positions.enemies[e.id].r_twist.angular.x = (old_x - new_x)/ time_diff

        # update velocity for ball
        old_ball_x = self.last_positions.ball_pos.position.x
        old_ball_y = self.last_positions.ball_pos.position.y
        new_ball_x = new_positions.ball_pos.position.x
        new_ball_y = new_positions.ball_pos.position.y
        
        new_positions.ball_speed.linear.x = (old_ball_x - new_ball_x) / time_diff
        new_positions.ball_speed.linear.y = (old_ball_y - new_ball_y) / time_diff

        # save location data and timestamp for next iteration
        self.last_positions = new_positions
        self.last_timestamp = new_timestamp
        return new_positions

    '''
    Run node
    '''
    def run(self):
        while not rospy.is_shutdown():
            new_data, timestamp = self.get_data()    
            new_data = self.update_velocity(new_data, timestamp)
            self.pub.publish(new_data)
            self.rate.sleep()

if __name__ == '__main__':
    vision_node = SSLVisionNode()
    vision_node.start()
    vision_node.run()
