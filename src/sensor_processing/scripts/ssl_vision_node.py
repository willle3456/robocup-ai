#!/usr/bin/env  python
import rospy
import sys
from sensor_processing.ssl_receiver import *
from sensor_processing.ssl_data_handler import *
from robocup_msgs.msg import Position

class SSLVisionNode(object):

    def __init__(self, params):
        
        team_color, rate, port = self.handle_args(params)

        self.receiver = SSLReceiver(port=port)
        self.data_handler = SSLDataHandler(team_color=team_color)
        
        rospy.init_node('ssl_vision', anonymous=False)
        self.pub = rospy.Publisher('locations', Position, queue_size=10)
        self.rate = rospy.Rate(rate)
        
        self.receiver.open()
    
    def handle_args(self, params):
        team_color = 0
        if params[1] == 'yellow':
            team_color = YELLOW
        elif params[1] == 'blue':
            team_color = BLUE
        else:
            raise ValueError('Team color must be yellow or blue, but was: {0}'.format(params[1]))

        rate = int(params[2])
        port = int(params[3])
        
        return team_color, rate, port

    def run(self):
        while not rospy.is_shutdown():
            pkt = self.receiver.recv_msg()
            
            if pkt is not None:
                self.data_handler.update_all(pkt)
                msg = Position()
                self.data_handler.fill_msg(msg)
                self.pub.publish(msg)
            self.rate.sleep()

if __name__ == '__main__':
    node = SSLVisionNode(sys.argv)
    node.run()
