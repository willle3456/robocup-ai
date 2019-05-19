#!/usr/bin/env python
from selection_metrics import SelectionMetrics
class SimpleStrategySelector(object):
    def __init__(self, metrics):
        self.metrics = metrics
    
    def select(self):
        # read through the coach metrics and decide which strategy to use or make any adjustments
        # return a, b
        # a: overall strategy
        # b: additional rule to add
        return 0, 0
