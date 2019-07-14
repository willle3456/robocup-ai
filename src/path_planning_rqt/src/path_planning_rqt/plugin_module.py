import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
#from python_qt_binding.QtWidgets import QMainWindow
from robocup_msgs.msg import Position, RobotData, Graph
from path_planning_rqt.planner_widget import PlannerViewerWidget
import networkx as nx

class GraphData(object):
    def __init__(self):
        self.graph = nx.Graph()
        self.path = []

class PlannerViewer(Plugin):
    
    def __init__(self, context):
        super(PlannerViewer, self).__init__(context)
        self.setObjectName('PlannerViewer')

        #args = self._parse_args(context.argv())
        
        self._widget = PlannerViewerWidget()
        
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        context.add_widget(self._widget)

        self.position_sub = rospy.Subscriber('locations', Position, self.update_position)
        self.graph_sub = rospy.Subscriber('graph_data', Graph, self.update_graph)

        self.graph_data = GraphData()
    
    def update_position(self, msg):
        self._widget.position_update.emit(msg)

    def update_graph(self, msg):
    
        # potentially switch this in the future to only render new items and remove old ones
        for node in msg.removed_nodes:
            self.graph_data.graph.remove_node(node)

        for node in msg.new_nodes:
            self.graph_data.graph.add_node(node)

        for edge_from, edge_to in zip(msg.removed_edges[::2], msg.removed_edges[1::2]):
            self.graph_data.graph.remove_edge(edge_from, edge_to)

        for edge_from, edge_to in zip(msg.new_edges[::2], msg.new_edges[1::2]):
            self.graph_data.graph.add_edge(edge_from, edge_to)

        self.graph_data.path = msg.path

        self._widget.graph_update.emit(self.graph_data)
