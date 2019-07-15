#!/usr/bin/env  python

from python_qt_binding.QtWidgets import QWidget, QGraphicsView, QGraphicsScene, QGraphicsEllipseItem, QGraphicsLineItem, QGraphicsItemGroup
from python_qt_binding.QtGui import QPainter, QPen, QBrush, QColor, QTransform
from python_qt_binding.QtCore import Signal
from python_qt_binding import loadUi
import networkx as nx
import os
import rospkg
import time

class PlannerGraphicsView(QGraphicsView):
    
    def __init__(self, parent=None):
        super(PlannerGraphicsView, self).__init__()

class PlannerViewerWidget(QWidget):
    graph_update = Signal(object)
    position_update = Signal(object)

    def __init__(self):
        super(PlannerViewerWidget, self).__init__()

        # load ui file
        ui_file = os.path.join(rospkg.RosPack().get_path('path_planning_rqt'), 'resource', 'planner_plugin.ui')
        loadUi(ui_file, self, {'planner_viewer': PlannerGraphicsView})

        # scene setup
        self.scene = QGraphicsScene()
        self.scene.setBackgroundBrush(QBrush(QColor(0,255,0)))
        #tmp_item = QGraphicsEllipseItem(0, 0, 100, 100)
        #self.scene.addItem(tmp_item)
        self.planner_viewer.setScene(self.scene)
        #self.planner_viewer.show()

        # class objects
        self.graph = nx.Graph()
 
        self.team = {}
        self.obs = {}

        self.path = []
        
        # pens and brushes
        self.team_pen = QPen(QColor(0,0,255))
        self.team_brush = QBrush(QColor(0,0,255))
        self.opp_pen = QPen(QColor(255,255,0))
        self.opp_brush = QBrush(QColor(255,255,0))

        self.graph_pen = QPen(QColor(255, 0, 0))
        self.graph_pen.setWidth(1)
        self.path_pen = QPen(QColor(0, 0, 0))
        self.path_pen.setWidth(3)

        # slots
        self.graph_update.connect(self.draw_graph)
        self.position_update.connect(self.draw_obs)

        # transforms
        self.widget_tf = QTransform()
        self.widget_tf.scale(0.05, 0.05)
        #self.widget_tf.translate(450, 300)

        self.bot_size = 15
        self.last_time = time.time()
        self.avg_buff = [0.0 for i in range(20)]
        self.avg_idx = 0

        # item groups
        self.edge_group = QGraphicsItemGroup()
        self.path_group = QGraphicsItemGroup()

        self.scene.addItem(self.edge_group)
        self.scene.addItem(self.path_group)

    def update_graph(self, graph):
        self.graph = graph

    def get_msg_position(self, robot):
        return robot.r_pose.position.x/20, -robot.r_pose.position.y/20

    def get_item_position(self, item):
        return item.x(), item.y()

    def add_item(self, robot, item_list, brush, pen):
        x, y = self.get_msg_position(robot)

        new_item = QGraphicsEllipseItem(x, y, self.bot_size, self.bot_size)
        new_item.setBrush(brush)
        new_item.setPen(pen)
        #new_item.setTransform(self.widget_tf)

        item_list.update({robot.id: new_item})
        self.scene.addItem(item_list[robot.id])

    def move_item(self, robot, item):
        x, y = self.get_msg_position(robot)
        last_x, last_y = self.get_item_position(item)
        dx = x - last_x
        dy = y - last_y
        
        item.moveBy(dx, dy)

    def update_items(self, data, item_list, brush, pen):
        for robot in data:
            if robot.id in item_list.keys():
                self.move_item(robot, item_list[robot.id])
            else:
                self.add_item(robot, item_list, brush, pen)
            

    def draw_obs(self, position_data):
        self.update_items(position_data.friends, self.team, self.team_brush, self.team_pen)
        self.update_items(position_data.enemies, self.obs, self.opp_brush, self.opp_pen)
        self.scene.update()

        #curr_time = time.time()
        #self.avg_buff[self.avg_idx] = min(100.0, 1/(curr_time - self.last_time))
        #self.avg_idx = (self.avg_idx + 1)%20
        #print 1/(curr_time - self.last_time)
        #self.last_time = curr_time

        #print sum(self.avg_buff)/20.0
        #print self.avg_buff

        #print 'TEAM:\n'
        #print [(t.x(), t.y()) for t in self.team.values()]
        #print 'OBS:\n'
        #print [(o.x(), o.y()) for o in self.obs.values()]

    def draw_nodes(self):
        # remove the old (can find by position)
        # add the new
        for node in list(self.graph.nodes()):
            self.scene.addEllipse(node.x, node.y, 2, 2)

        self.scene.update()
        #self.render(self.painter)

    def draw_graph(self, graph_msg):
        #self.draw_nodes()
        for item in self.path:
            self.scene.removeItem(item)
        self.path = []

        for edge_from, edge_to in zip(graph_msg.new_edges[::2], graph_msg.new_edges[1::2]):
            tmp_line = QGraphicsLineItem(edge_from.position.x/10, -edge_from.position.y/10, edge_to.position.x/10, -edge_to.position.y/10)
            tmp_line.setPen(self.graph_pen)
            self.scene.addItem(tmp_line)
        
        for start_way, end_way in zip(graph_msg.path, graph_msg.path[1::]):
            tmp_line = QGraphicsLineItem(start_way.position.x/10, -start_way.position.y/10, end_way.position.x/10, -end_way.position.y/10)
            tmp_line.setPen(self.path_pen)
            self.path.append(tmp_line)
    
        for item in self.path:
            self.scene.addItem(item)
        
        print len(self.scene.items())
        self.scene.update()
