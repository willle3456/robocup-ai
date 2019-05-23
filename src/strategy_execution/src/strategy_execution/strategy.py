from abc import ABCMeta, abstractmethod
from basic_skills.robot import Robot

class Strategy:
    __metaclass__ = ABCMeta
    
    def __init__(self):
        self.last_action = None
        self.actions = []
        self.zone_list = [self.rule_zone_0, 
                self.rule_zone_1,
				self.rule_zone_2,
				self.rule_zone_3,
				self.rule_zone_4,
				self.rule_zone_5,
				self.rule_zone_6,
				self.rule_zone_7,
				self.rule_zone_8,
				self.rule_zone_9,
				self.rule_zone_10,
				self.rule_zone_11,
				self.rule_zone_12,
				self.rule_zone_13,
				self.rule_zone_14,
				self.rule_zone_15,
				self.rule_zone_16,
				self.rule_zone_17,
				self.rule_zone_18,
				self.rule_zone_19
			]

    @abstractmethod
    def rule_zone_0(self, robot ):
		pass

    @abstractmethod
    def rule_zone_1(self, robot):
		pass

    @abstractmethod
    def rule_zone_2(self, robot):
		pass

    @abstractmethod
    def rule_zone_3(self, robot):
		pass

    @abstractmethod
    def rule_zone_4(self, robot):
		pass

    @abstractmethod
    def rule_zone_5(self, robot):
		pass

    @abstractmethod
    def rule_zone_6(self, robot):
		pass

    @abstractmethod
    def rule_zone_7(self, robot):
		pass

    @abstractmethod
    def rule_zone_8(self, robot):
		pass

    @abstractmethod
    def rule_zone_9(self, robot):
		pass

    @abstractmethod
    def rule_zone_10(self, robot):
		pass

    @abstractmethod
    def rule_zone_11(self, robot):
		pass

    @abstractmethod
    def rule_zone_12(self, robot):
		pass

    @abstractmethod
    def rule_zone_13(self, robot):
		pass

    @abstractmethod
    def rule_zone_14(self, robot):
		pass

    @abstractmethod
    def rule_zone_15(self, robot):
		pass

    @abstractmethod
    def rule_zone_16(self, robot):
		pass

    @abstractmethod
    def rule_zone_17(self, robot):
		pass

    @abstractmethod
    def rule_zone_18(self, robot):
		pass

    @abstractmethod
    def rule_zone_19(self, robot):
		pass

    def apply(self, zone, robot):
		return self.zone_list[zone](robot)
