#!/usr/bin/env  python

from python_qt_binding.QtWidgets import QWidget, QGraphicsView, QGraphicsScene, QGraphicsEllipseItem, QGraphicsLineItem, QGraphicsRectItem
from python_qt_binding.QtGui import QPainter, QPen, QBrush, QColor
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
        self.field_rect = QGraphicsRectItem(0, 0, 900, 600)
        self.field_rect.moveBy(-450, -300)
        self.field_rect.setPen(QPen(QColor(255,255,255)))
        self.scene.addItem(self.field_rect)
        self.planner_viewer.setScene(self.scene)

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

        self.bot_size = 15

    def update_graph(self, graph):
        self.graph = graph

    def get_msg_position(self, robot):
        return (robot.r_pose.position.x/10), (-robot.r_pose.position.y/10)

    def get_item_position(self, item):
        return item.x(), item.y()

    def add_item(self, robot, item_list, brush, pen):
        x, y = self.get_msg_position(robot)
        new_item = QGraphicsEllipseItem(-self.bot_size/2, -self.bot_size/2, self.bot_size, self.bot_size)
        new_item.setBrush(brush)
        new_item.setPen(pen)

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

    def draw_nodes(self):
        for node in list(self.graph.nodes()):
            self.scene.addEllipse(node.x, node.y, 2, 2)

        self.scene.update()

    def draw_graph(self, graph_msg):
        #self.draw_nodes()
        for item in self.path:
            self.scene.removeItem(item)
        self.path = []

        for start, end in zip(graph_msg.new_edges_from, graph_msg.new_edges_to):
            tmp_line = QGraphicsLineItem(start.position.x/10, -start.position.y/10, end.position.x/10, -end.position.y/10)
            tmp_line.setPen(self.graph_pen)
            self.scene.addItem(tmp_line)
        
        for start_way, end_way in zip(graph_msg.path, graph_msg.path[1::]):
            tmp_line = QGraphicsLineItem(start_way.position.x/10, -start_way.position.y/10, end_way.position.x/10, -end_way.position.y/10)
            tmp_line.setPen(self.path_pen)
            self.path.append(tmp_line)
        
        for item in self.path:
            self.scene.addItem(item)
        
        self.scene.update()
