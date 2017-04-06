#!/usr/bin/env python

import rospy, tf, numpy, math
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String, Header 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, GridCells, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose, Point, PointStamped, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from Node import Node
import time


def createGrid(event):
	global grid 
	global cell_size
	global hasMap
	global rows, cols
	global offset

	if(not hasMap):
		x = 0 
		y = 0 
		rows = event.info.height
		cols = event.info.width
		offset = event.info.origin.position.x
		print ("map rows = %d, map cols = %d" % (rows,cols))

		cell_size = event.info.resolution
		print ("map resolution = %f" % (cell_size))

		grid = [[Node(0,0,50) for i in range(rows)] for j in range(cols)]

		while (x < cols):
			while (y < rows):
				#asign a node to each grid item of node (x, y, wallchance)
				grid[x][y] = Node(x, y, event.data[x+(y*cols)])
				#print "{} {}".format(x,y)
				y += 1
			y = 0
			x += 1

		print "map loaded"
		hasMap = True

def updateGoal(msg):
	global goal
	global start
	global complete
	global grid
	global hasGoal
	global cell_size
	global offset

	if (complete or not hasGoal):
		goal.x = msg.point.x
		goal.y = msg.point.y
		print goal.x
		print goal.y

		goal = grid[int((goal.x - offset) / cell_size)][int((goal.y) / cell_size)]
		complete = False
		print ("goalX = %d, goalY = %d" % (goal.x, goal.y))
		hasGoal = True

	
def updateStart(msg):
	global start
	global complete
	global grid
	global hasStart
	global cell_size

	if (complete or hasStart):
		start.x = msg.pose.pose.position.x
		start.y = msg.pose.pose.position.y
		print start.x
		print start.y

		start = grid[int((start.x - offset) / cell_size)][int((start.y) / cell_size)]

		complete = False
		print ("StartX = %d, StartY = %d" % (start.x, start.y))
		hasStart = True

	

def AStar(initial, end):
	global grid, complete
	global start, goal
	global rows, cols
	global path

	print "Astar Running"

	notVisited = [] #nodes you havent been to
	visited = [] #nodes that youve been to
	children = []
	path = []

	#set the first current
	current = initial 
	notVisited.append(initial)

	#is each node a point?
	while (notVisited and not rospy.is_shutdown()):

		i = 0
		children = []
		current.calcCosts(start,goal)

		#check if goal reached
		if (current == end):
			print "Great Success!!!"
			#reconstruct_path not written
			complete = True
			path.append(goal)
			reconstruct_path(current) #reconstruct_path returns an array of nodes
			return

		# find the children
		if (current.x + 1 < cols):
			children.append(grid[int(current.x+1)][int(current.y)])
			#print "left child exists"
		if (current.y + 1 < rows):
			children.append(grid[int(current.x)][int(current.y+1)])
			#print "bottom child exists"
		if (current.x - 1 > 0):
			children.append(grid[int(current.x-1)][int(current.y)])
			#print "right child exists"
		if (current.y - 1 > 0):
			children.append(grid[int(current.x)][int(current.y-1)])
			#print "top child exists"

		#print ""
		#print "check children"

		#check if children have been visited or not or if its an obstacle
		k = len(children)-1
		while(k >= 0):
			if(children[k] in visited):
				#print "children remove visited {}, {}".format(children[k].x,children[k].y)
				children.remove(children[k])
			elif(children[k].obstacle):
				#print "children remove obstacle {}, {}".format(children[k].x,children[k].y)
				children.remove(children[k])
			else:
				pass
				#print "good children {}, {}".format(children[k].x,children[k].y)
			k -= 1

		#print "children done"
		#print ""

		if(children):
			#find the successor through the fcost comparison
			children[0].calcCosts(start, goal)
			for Node in children:
				Node.calcCosts(start, goal)
				Node.parent = current
				#print "child {}, {} Gcosts {}, Hcosts {}, Fcosts {}".format(Node.x,Node.y,Node.gCost,Node.hCost,Node.fCost)
		
			#place remaining children into notVisited nodes and publish them as Frontier
			a = 0
			while(a < len(children)):
				notVisited.append(children[a])
				publishFrontierCell(children[a].x, children[a].y)
				a += 1

			#update the sets
			visited.append(current)
			publishVisitedCell(current.x, current.y)
			#print notVisited
			notVisited.remove(current)

			#iterate through notVisited to find lowest cost
			successor = notVisited[0]
			for Node in notVisited:
				if(Node.fCost <= successor.fCost):
					successor = Node

			current = successor

		#else if children is empty go back 
		elif(not children):
			#print "no children"
			notVisited.remove(current)
			visited.append(current)
			publishVisitedCell(current.x, current.y)

			#iterate through notVisited to find lowest cost
			successor = notVisited[0]
			for Node in notVisited:
				if(Node.fCost <= successor.fCost):
					successor = Node

			current = successor

		time.sleep(.25)

	#failed to find the goal
	print "AStar Failed"
	complete = True
	return

def reconstruct_path(step):
	global path

	if(step.parent):
		path.append(step.parent)
		reconstruct_path(step.parent)
	elif(not step.parent):
		publishPath(path)
		wayPoints(path)

def publishPath(steps):
	print "Paths : "
	i = len(steps) - 1
	while (i >= 0):
		print "[{},{}]".format(steps[i].x,steps[i].y)
		publishPathCell(steps[i].x, steps[i].y)
		i -= 1
	print ""

def wayPoints(steps):
	ways = []
	i = len(steps) -1
	ways.append(steps[i])
	while (i > 0):
		if((steps[i].x == steps[i-1].x) and not (steps[i].y == steps[i-1].y)):
			if(not (steps[i-1].x == steps[i-2].x) and (steps[i-1].y == steps[i-2].y)):
				ways.append(steps[i-1])
		elif(not (steps[i].x == steps[i-1].x) and (steps[i].y == steps[i-1].y)):
			if((steps[i-1].x == steps[i-2].x) and not (steps[i-1].y == steps[i-2].y)):
				ways.append(steps[i-1])

		i -= 1
	ways.append(steps[0])

	print "Waypoints  :  "

	i = len(ways) - 1
	while (i >= 0):  
		print "[{},{}]".format(ways[i].x,ways[i].y)
		publishPathCell(ways[i].x, ways[i].y)
		i -= 1
		print ""

def publishVisitedCell(x, y):
	global visited_pub
	global visited_cell_list
	global cell_size
	global offsets

	msg = GridCells()

	grid_height = cell_size
	grid_width = cell_size

	header = Header()
	header.frame_id = "map"

	msg.header = header

	msg.cell_width = grid_width
	msg.cell_height = grid_height

	point = Point()
	point.x = (x * cell_size) + offset + (cell_size/2)
	point.y = (y * cell_size) + (cell_size/2)

	visited_cell_list.append(point)
	msg.cells = visited_cell_list
	visited_pub.publish(msg)


def publishBlockedCell(x, y):
	global block_pub
	global blocked_cell_list
	global cell_size
	global offset

	msg = GridCells()

	grid_height = cell_size
	grid_width = cell_size

	header = Header()
	header.frame_id = "map"

	msg.header = header
	msg.cell_width = grid_width
	msg.cell_height = grid_height

	point = Point()
	point.x = (x * cell_size) + offset + (cell_size/2)
	point.y = (y * cell_size) + (cell_size/2)

	blocked_cell_list.append(point)
	msg.cells = blocked_cell_list
	block_pub.publish(msg)


def publishFrontierCell(x, y):
	global frontier_pub
	global frontier_cell_list
	global cell_size
	global offset

	msg = GridCells()

	grid_height = cell_size
	grid_width = cell_size

	header = Header()
	header.frame_id = "map"

	msg.header = header
	msg.cell_width = grid_width
	msg.cell_height = grid_height

	point = Point()
	point.x = (x * cell_size) + offset + (cell_size/2)
	point.y = (y * cell_size) + (cell_size/2)

	frontier_cell_list.append(point)
	msg.cells = frontier_cell_list
	frontier_pub.publish(msg) 
	

def publishPathCell(x, y):
	global path_pub
	global path_cell_list
	global cell_size
	global offset

	msg = GridCells()

	grid_height = cell_size
	grid_width = cell_size

	header = Header()
	header.frame_id = "map"

	msg.header = header
	msg.cell_width = grid_width
	msg.cell_height = grid_height

	point = Point()
	point.x = (x * cell_size) + offset + (cell_size/2)
	point.y = (y * cell_size) + (cell_size/2)

	path_cell_list.append(point)
	msg.cells = path_cell_list
	path_pub.publish(msg) 


if __name__ == '__main__':

	rospy.init_node('rpcole_lab3_node')

	global visited_pub, block_pub, frontier_pub, unknown_pub, path_pub
	global pose 
	global odom_tf
	global frontier_cell_list
	global path_cell_list
	global visited_cell_list
	global blocked_cell_list
	global goal
	global start
	global complete
	global cell_size
	global hasMap
	global rows, cols
	global offset
	global hasStart
	global hasGoal

	#initialize variables
	visited_cell_list = []
	path_cell_list = []
	frontier_cell_list = []
	blocked_cell_list = []
	pose = PoseStamped()
	start = Node(0,0,50)
	goal = Node(0,0,50)
	complete = True
	cell_size = 0.3
	hasMap = False
	hasGoal = False
	hasStart = False
	rows = 0
	cols = 0
	offset = .5

	#initialize publishers and subscribers
	visited_pub = rospy.Publisher('/visited', GridCells, queue_size = 10)
	block_pub = rospy.Publisher('/blocked', GridCells, queue_size = 10)
	frontier_pub = rospy.Publisher('/frontier', GridCells, queue_size = 10)
	path_pub = rospy.Publisher('/path', GridCells, queue_size = 10)
	map_sub = rospy.Subscriber('/map', OccupancyGrid, createGrid)
	goal_sub = rospy.Subscriber('/clicked_point', PointStamped, updateGoal)
	start_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, updateStart)
	
	

	while(1 and not rospy.is_shutdown()):

		while(not complete and hasStart and hasGoal):
			AStar(start, goal)
		

	print "Lab 3 complete"