#!/usr/bin/env python3

"""
	Date 24.01.2021
	Author Berke Algul
"""

import rospy
import numpy as np
from math import ceil, sqrt
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped, Pose

class Node:
	def __init__(self, x, y):
		self.x = x
		self.y = y
		self.g_cost = 0
		self.h_cost = 0
		self.cost = 0
		self.parent = None

class Astar:
	def __init__(self):
		self.grid_w = 0
		self.grid_h = 0
		self.grid_res = 1
		self.cost_data = None
		self.origin_x = 0
		self.origin_y = 0

		self.goal_x = None
		self.goal_y = None

		# baslangic noktasi
		self.x = 0
		self.y = 0

		self.nodes = None

	def set_map(self, map):
		print("setting map")
		self.grid_w = map.info.width
		self.grid_h = map.info.height
		self.grid_res = map.info.resolution
		# 2d matrise cevirme
		self.cost_data = np.array(map.data).reshape(self.grid_h, self.grid_w)
		self.origin_x = int(self.grid_w/2)
		self.origin_y = int(self.grid_h/2)

	def set_goal(self, goal):
		x, y = self.d(goal.pose.position.x, goal.pose.position.y)

		if x < 0 or x >= self.grid_w or y < 0 or y >= self.grid_h:
			print("Goal out of map. Disgarded")
			return
		else:
			print("Goal received")
			self.goal_x = x
			self.goal_y = y

	def set_pos(self, odom):
		self.x, self.y = self.d(odom.pose.pose.position.x, odom.pose.pose.position.y)

	def create_nodeset(self):
		print("initializing nodes")
		# bruteforce 2d liste
		self.nodes = []

		for y in range(self.grid_h):
			row = []
			for x in range(self.grid_w):
				# engel olan yuzey
				if self.cost_data[y, x] == 100:
					row.append(None)
				else:
					node = Node(x, y)
					dx = x - self.goal_x
					dy = y - self.goal_y
					node.h_cost = sqrt(dx**2+dy**2)
					row.append(node)
			self.nodes.append(row)

	# metreyi hucre koordinatlarina cevir
	def d(self, x, y):
		x = ceil(x/self.grid_res) + self.origin_x
		y = ceil(y/self.grid_res) + self.origin_y
		return x, y

	# d() nin tam tersi metreye cevirir
	def meter(self, x, y):
		x = (x - self.origin_x) * self.grid_res
		y = (y - self.origin_y) * self.grid_res
		return x, y

	# 2d -> 1d koordinat donusumu icin
	# reshape alternatifi c++ de boyle yaptim
	def I(self, x, y):
		return y * self.grid_w + x

	"""
	Dısarsan cagrilan ana fonksiyon patikayi dondurur
	"""
	def solve(self):
		print("solving")
		# goal yok ise bos patika ver
		if self.goal_x == None:
			print("goal is none")
			return Path()

		goal_node = self.solve_path()

		if goal_node is None:
			return Path()
		else: # unreachable
			return self.create_ros_path(goal_node)
	
	# patikayi olusturan fonksiyon
	def solve_path(self):
		self.create_nodeset()
		openset = []
		closedset = []

		current = self.nodes[self.y][self.x]
		goal = self.nodes[self.goal_y][self.goal_x]

		openset.append(current)

		while len(openset) is not 0:
			# ne az masrafli node u al
			current = openset[0]

			openset.remove(current)
			closedset.append(current)

			# hedefe vardıkmı vardiysak dondur
			if current == goal:
				print("done")
				return current

			neighbors = self.get_neighbors(current)

			for neighbor in neighbors:

				if neighbor in closedset:
					continue
				
				g_cost = current.g_cost + self.cost_data[neighbor.y, neighbor.x]

				if neighbor not in openset:
					neighbor.g_cost = g_cost
					neighbor.cost = neighbor.h_cost + neighbor.g_cost
					neighbor.parent = current
					openset.append(neighbor)

				elif neighbor.cost > g_cost:
					neighbor.g_cost = g_cost
					neighbor.cost = neighbor.h_cost + neighbor.g_cost
					neighbor.parent = current

			# ilk sirada herzaman en az masrafli dugum olmali
			# priority queue veri yapisina bakabilirsin
			openset.sort(key=lambda x: x.cost)

		print("Failed")
		return None

	def get_neighbors(self, node):
		neighbors = []

		# 8 komsu icin
		# çapraz komsular dahildir
		for i in range(-1, 2):
			for j in range(-1, 2):

				# kendisini dahil edemeyiz
				if i == 0 and j == 0:
					continue

				if node is None:
					continue

				x = node.x + i
				y = node.y + j

				# grid disinda ise sal
				if x < 0 or x >= self.grid_w or y < 0 or y >= self.grid_h:
					continue

				n = self.nodes[y][x]

				if n is not None:
					neighbors.append(n)

		return neighbors

	# patikayi ros mesajina cevir
	def create_ros_path(self, goal_node):
		path = Path()
		path.header.frame_id = "map"

		while goal_node is not None:
			pose_stamp = PoseStamped()
			# koordinatlari metre cinsinden girmeliyiz
			x, y = self.meter(goal_node.x, goal_node.y)
			pose_stamp.pose.position.x = x
			pose_stamp.pose.position.y = y
			pose_stamp.header.frame_id = "map"

			path.poses.append(pose_stamp)
			goal_node = goal_node.parent # recursive donguyu sagla
			
		# astar patikayi goal node dan baslar. Bu demo icin problem degil ama ters cevrilmeli
		path.poses.reverse()
		return path


astar = Astar()

def map_cb(map):
	print("map cb")
	astar.set_map(map)

def odom_cb(odom):
	print("odom cb")
	astar.set_pos(odom)

def goal_cb(goal):
	astar.set_goal(goal)

if __name__ == "__main__":
	rospy.init_node("astar_planning_demo")

	rospy.Subscriber("/rover_mapping/inflated_map", OccupancyGrid, map_cb)
	rospy.Subscriber("/odometry/filtered_map", Odometry, odom_cb)
	rospy.Subscriber("move_base_simple/goal", PoseStamped, goal_cb)

	path_pub = rospy.Publisher("/path", Path, queue_size = 1)

	print("initialized")

	while not rospy.is_shutdown():
		print("main loop")
		path = astar.solve()
		path_pub.publish(path)
