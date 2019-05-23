#!/usr/bin/env  python
import rospy
from robocup_msgs.msg import Position
from robocup_msgs.msg import Strategy
from strategy_selection.simple_strategy_selector import SimpleStrategySelector
from strategy_selection.selection_metrics import SelectionMetrics
from basic_skills.player_data import Pose2D
from action_selection.selectors import *

class Coach(object):
    def __init__(self):
        
        # publisher for strategies
        self.strat_pub = rospy.Publisher('strategy', Strategy, queue_size=10)
        
        # subscriber to position
        self.pos_sub = rospy.Subscriber('locations', Position, self.update_position_metrics)
        
        #TODO subscriber to ref
        # ref_sub = rospy.Subscriber('ref_msgs', Ref, update_ref_metrics)

        # coach data
        self.metrics = SelectionMetrics()
        self.selector = SimpleStrategySelector(self.metrics)
        self.init_flg = False
        self.current_strat = 0
        self.current_rules = 0

    def update_position_metrics(self, data):
        for f in data.friends:
            tmp_loc = Pose2D(f.r_pose.position.x, f.r_pose.position.y)
            tmp_zone = get_zone(tmp_loc)
            self.metrics.friend_zone[f.id][tmp_zone] += 1
        for e in data.enemies:
            tmp_loc = Pose2D(e.r_pose.position.x, e.r_pose.position.y)
            tmp_zone = get_zone(tmp_loc)
            self.metrics.enemy_zone[e.id][tmp_zone] += 1
 
        if data.ball_pos:
            tmp_loc = Pose2D(data.ball_pos.position.x, data.ball_pos.position.y)
            tmp_zone = get_zone(tmp_loc)
            self.metrics.ball_zone[tmp_zone] += 1
        self.init_flg = True
    
    #TODO need ref box code and msg 1st
    def update_ref_metrics(self):
        pass

    def choose_strat(self):
        return self.selector.select()

    def run(self):
        if self.init_flg:
            new_strat, new_rules = self.choose_strat()
            if (new_strat != self.current_strat) or (new_rules != self.current_rules):
                strat_msg = Strategy()
                strat_msg.strategy = new_strat
                strat_msg.additional_rules = new_rules
                self.strat_pub.publish(strat_msg)
                self.current_strat = new_strat
                self.current_rules = new_rules
        else:
            strat_msg = Strategy()
            strat_msg.strategy = self.current_strat
            strat_msg.additional_rules = self.current_rules
            self.strat_pub.publish(strat_msg)
