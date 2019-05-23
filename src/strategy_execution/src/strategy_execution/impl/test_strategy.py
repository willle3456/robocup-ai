#!/usr/bin/env python
#from skill_execution import *
from basic_skills import *
from strategy_execution import Strategy

class TestStrategy(Strategy):
    def __init__(self):
        super(TestStrategy, self).__init__()
        self.actions = [MoveTo()]

    def common_rule(self, robot):
        return self.actions[0]
   
    def rule_zone_0(self, robot):
        return self.common_rule(robot)

    def rule_zone_1(self, robot):
        return self.common_rule(robot)

    def rule_zone_2(self, robot):
        return self.common_rule(robot)
	
    def rule_zone_3(self, robot):
        return self.common_rule(robot)
	
    def rule_zone_4(self, robot):
        return self.common_rule(robot)
	
    def rule_zone_5(self, robot):
        return self.common_rule(robot)
	
    def rule_zone_6(self, robot):
        return self.common_rule(robot)
	
    def rule_zone_7(self, robot):
        return self.common_rule(robot)
	
    def rule_zone_8(self, robot):
        return self.common_rule(robot)
	
    def rule_zone_9(self, robot):
        return self.common_rule(robot)
	
    def rule_zone_10(self, robot):
        return self.common_rule(robot)
 	
    def rule_zone_11(self, robot):
        return self.common_rule(robot)
	
    def rule_zone_12(self, robot):
        return self.common_rule(robot)
	
    def rule_zone_13(self, robot):
        return self.common_rule(robot)
	
    def rule_zone_14(self, robot):
        return self.common_rule(robot)
	
    def rule_zone_15(self, robot):
        return self.common_rule(robot)
	
    def rule_zone_16(self, robot):
        return self.common_rule(robot)
	
    def rule_zone_17(self, robot):
        return self.common_rule(robot)
	
    def rule_zone_18(self, robot):
        return self.common_rule(robot)
	
    def rule_zone_19(self, robot):
        return self.common_rule(robot)
