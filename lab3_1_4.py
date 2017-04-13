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
from lab4.msg import Waypoint

#creates a background grid of Nodes for us to access based upon the map OccupancyCells
def createGrid(event):
	global grid 
	global cell_size
	global hasMap
	global rows, cols
	global Xoffset
	global Yoffset
	global obstacle_cell_list
	global pad_cell_list

	if(not hasMap):
		x = 0 
		y = 0 
		rows = event.info.height
		cols = event.info.width
		Xoffset = event.info.origin.position.x
		Yoffset = event.info.origin.position.y
		print ("map rows = %d, map cols = %d" % (rows,cols))

		cell_size = event.info.resolution
		print ("map resolution = %f" % (cell_size))

		grid = [[Node(0,0,50) for i in range(rows)] for j in range(cols)]

		while (x < cols):
			while (y < rows):
				#asign a node to each grid item of node (x, y, wallchance)
				grid[x][y] = Node(x, y, event.data[x+(y*cols)])
				#print "{} {}".format(x,y)
				if(grid[x][y].obstacle == True):
					obstacle_cell_list.append(grid[x][y])
				y += 1
			y = 0
			x += 1

		i = 0
		j = 0

		while (i < cols):
			while (j < rows):
				#check to see if cell is obstacle
				#print "obstacle check"
				if(grid[i][j].obstacle == True):
					#print "obstacle true"
					#if cell is obstacle, pad cells around it 
					#if they are not obstacles too
					if((i+1<rows) and (grid[i+1][j].obstacle == False)):
						#print "true true"
						publishPadCell(i+1,j)
						grid[i+1][j].padding = True
						time.sleep(.005)
					if((j+1<cols) and (grid[i][j+1].obstacle == False)):
						#print "true true"
						publishPadCell(i,j+1)
						grid[i][j+1].padding = True
						time.sleep(.005)
					if(grid[i][j-1].obstacle == False):
						#print "true true"
						publishPadCell(i,j-1)
						grid[i][j-1].padding = True
						time.sleep(.005)
					if(grid[i-1][j].obstacle == False):
						#print "true true"
						publishPadCell(i-1,j)
						grid[i-1][j].padding = True
						time.sleep(.005)
				j += 1
			j = 0
			i += 1
		
		start = grid[int(Xoffset / -cell_size)][int(Yoffset / -cell_size)]
		print ("start(createGrid) = {},{}" .format(start.x,start.y))

		print "map loaded"
		hasMap = True

def updateGoal(msg):
	global goal
	global start
	global complete
	global grid
	global hasGoal
	global cell_size
	global Xoffset
	global Yoffset

	if (complete or not hasGoal):
		goal.x = msg.point.x
		goal.y = msg.point.y
		print goal.x
		print goal.y

		goal = grid[int((goal.x - Xoffset) / cell_size)][int((goal.y - Yoffset) / cell_size)]
		complete = False
		print ("goalX = %d, goalY = %d" % (goal.x, goal.y))
		hasGoal = True

	
def updateStart(msg):
	global start
	global complete
	global grid
	global hasStart
	global cell_size
	global Xoffset
	global Yoffset

	if (complete or hasStart):
		start.x = msg.pose.pose.position.x
		start.y = msg.pose.pose.position.y
		print start.x
		print start.y

		#start = grid[int((start.x - Xoffset) / cell_size)][int((start.y - Yoffset) / cell_size)]

		start = grid[int(Xoffset / -cell_size)][int(Yoffset / -cell_size)]
		print ("start(updateStart) = {},{}" .format(start.x,start.y))

		complete = False
		print ("StartX = %d, StartY = %d" % (start.x, start.y))
		hasStart = True

	
def reStar(msg):
	global grid
	global goal
	global cell_size

	AStar(grid[int((msg.x / -cell_size) + start.x)][int((msg.y / -cell_size) + start.y)], goal)


def AStar(initial, end):
	global grid, complete
	global start, goal
	global rows, cols
	global path
	global cell_size

	print "start {},{}".format(start.x,start.y)

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

		print "current {}, {}".format(current.x,current.y)

		#check if goal reached
		if (current == end):
			print "Great Success!!!"
			#reconstruct_path not written
			complete = True
			path.append(goal)

			#reset all visited attributes for next run
			k = len(visited) - 1
			while(k >= 0):
				visited[k].visited = False
				visited[k].inNotVisited = False
				k -= 1

			j = len(notVisited) - 1
			while(j >= 0):
				notVisited[j].visited = False
				notVisited[j].inNotVisited = False
				j -= 1

			#create path and publish path and waypoints	
			reconstruct_path(current) #reconstruct_path returns an array of nodes
			return

		# find the children
		if (current.x + 1 < cols):
			print "left child exists {},{}".format(current.x+1,current.y)
			children.append(grid[int(current.x+1)][int(current.y)])
		if (current.y + 1 < rows):
			print "bottom child exists {},{}".format(current.x,current.y+1)
			children.append(grid[int(current.x)][int(current.y+1)])
		if (current.x - 1 > 0):
			print "right child exists {},{}".format(current.x-1,current.y)
			children.append(grid[int(current.x-1)][int(current.y)])
		if (current.y - 1 > 0):
			print "top child exists {},{}".format(current.x,current.y-1)
			children.append(grid[int(current.x)][int(current.y-1)])

		print ""
		#print "check children"

		#check if children have been visited or not or if its an obstacle
		k = len(children)-1
		while(k >= 0):
			if(children[k].visited):
				#print "children remove visited {}, {}".format(children[k].x,children[k].y)
				children.remove(children[k])
			elif(children[k].obstacle):
				#print "children remove obstacle {}, {}".format(children[k].x,children[k].y)
				children.remove(children[k])
			elif(children[k].padding):
				#print "children remove padding {}, {}".format(children[k].x,children[k].y)
				children.remove(children[k])
			elif(children[k].inNotVisited):
				#print "children remove frontier {}, {}".format(children[k].x,children[k].y)
				children.remove(children[k])
			else:
				pass
				#print "good children {}, {}".format(children[k].x,children[k].y)
			k -= 1

		#print "children done"
		#print ""

		if(children):
			for Node in children:
				Node.parent = current
				Node.calcCosts(start, goal)
				#print "child {}, {} Gcosts {}, Hcosts {}, Fcosts {}".format(Node.x,Node.y,Node.gCost,Node.hCost,Node.fCost)
		
			#place remaining children into notVisited nodes and publish them as Frontier
			a = 0
			while(a < len(children)):
				#print "add {}, {}".format(children[a].x, children[a].y)
				heapPushfCost(notVisited, children[a])
				children[a].inNotVisited = True
				#print "append {}, {}".format(children[a].x, children[a].y)
				publishFrontierCell(children[a].x, children[a].y)
				a += 1

			#update the sets
			#print "rem notVisited"
			#print notVisited
			#print ""
			#print current
			notVisited.remove(current)
			visited.append(current)
			current.visited = True
			publishVisitedCell(current.x, current.y)

			#sort notVisited based on the fCost from low to high
			#notVisited.sort(key=lambda node: node.fCost, reverse = False)
			#pull first in list because has lowest cost
			successor = notVisited[0]

			current = successor

		#else if children is empty go back 
		else:
			print "remove"
			#print notVisited
			#print ""
			#print current
			notVisited.remove(current)
			visited.append(current)
			current.visited = True
			publishVisitedCell(current.x, current.y)

			#sort notVisited based on the fCost from low to high
			#notVisited.sort(key=lambda node: node.fCost, reverse = False)
			#pull first in list because has lowest cost
			successor = notVisited[0]

			current = successor

		print "{}".format(len(notVisited))
		
		time.sleep(.005)	

	#failed to find the goal
	print "AStar Failed"
	complete = True
	return

def heapPushfCost(uselist, item):

	#print "inside"

	if(not uselist):
		#print "did not exist"
		uselist.append(item)
		return

	#print "past"

	i = 0
	while(i < len(uselist) and not rospy.is_shutdown()):
		if(item.fCost <= uselist[i].fCost):
			#print "add list"
			uselist.insert(i,item)
			return
		i += 1

	#print "not smallest"
	uselist.append(item)

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
	global tolerance
	global cell_size

	ways = []
	waypoint = []

	i = len(steps) -1
	ways.append(steps[i])
	point = Point()
	point.x = steps[i].x
	point.y = steps[i].y
	waypoint.append(point)
	print ""
	print "[{},{}]".format(point.x,point.y)

	#start from the end
	while (i > 0):
		if(len(steps) <= 2):
			break
		elif((steps[i].x == steps[i-1].x) and not (steps[i].y == steps[i-1].y)):
			if(not (steps[i-1].x == steps[i-2].x) and (steps[i-1].y == steps[i-2].y)):
				ways.append(steps[i-1])
				point = Point()
				point.x = steps[i-1].x * cell_size
				point.y = steps[i-1].y * cell_size
				waypoint.append(point)
				print "[{},{}]".format(point.x,point.y)
		elif(not (steps[i].x == steps[i-1].x) and (steps[i].y == steps[i-1].y)):
			if((steps[i-1].x == steps[i-2].x) and not (steps[i-1].y == steps[i-2].y)):
				ways.append(steps[i-1])
				point = Point()
				point.x = steps[i-1].x * cell_size
				point.y = steps[i-1].y * cell_size
				waypoint.append(point)
				print "[{},{}]".format(point.x,point.y)

		i -= 1
	ways.append(steps[0])
	point = Point()
	point.x = steps[0].x * cell_size
	point.y = steps[0].y * cell_size
	waypoint.append(point)
	print "[{},{}]".format(point.x,point.y)
	
	print "Waypoints  :  "

	i = len(ways) - 1
	while (i >= 0):
		#print "[{},{}]".format(ways[i].x,ways[i].y)
		publishPathCell(ways[i].x, ways[i].y)
		i -= 1
		print ""


	msg = Waypoint()
	msg.waypoints = waypoint
	waypoint_pub.publish(msg)

def newCost(msg):

	global grid

	newRows = msg.info.height
	newCols = msg.info.width

	costMap = [[Node(0,0,50) for i in range(newRows)] for j in range(newCols)]

	x=0
	y=0
	while (x < newCols):
		while (y < newRows):
			#asign a node to each grid item of node (x, y, wallchance)
			costMap[x][y] = Node(x, y, msg.data[x+(y*cols)])
			#print "{} {}".format(x,y)
			y += 1
		y = 0
		x += 1	

	x=0
	y=0
	while (x < newCols):
		while (y < newRows):
			if(not grid[x][y].padding):			
				if (costMap[x][y].obstacle and (not grid[x][y].obstacle)):
					gird[x][y].obstacle = True
				elif((not costMap[x][y].obstacle) and grid[x][y].obstacle):
					grid[x][y].obstacle = False
		y = 0
		x += 1

	

def publishVisitedCell(x, y):
	global visited_pub
	global visited_cell_list
	global cell_size
	global Xoffset
	global Yoffset
	msg = GridCells()

	grid_height = cell_size
	grid_width = cell_size

	header = Header()
	header.frame_id = "map"

	msg.header = header

	msg.cell_width = grid_width
	msg.cell_height = grid_height

	point = Point()
	point.x = (x * cell_size) + Xoffset + (cell_size/2)
	point.y = (y * cell_size) + Yoffset + (cell_size/2)

	visited_cell_list.append(point)
	msg.cells = visited_cell_list
	visited_pub.publish(msg)

def publishPadCell(x, y):
	global pad_pub
	global padding_cell_list
	global cell_size
	global Xoffset
	global Yoffset

	msg = GridCells()

	grid_height = cell_size
	grid_width = cell_size

	header = Header()
	header.frame_id = "map"

	msg.header = header
	msg.cell_width = grid_width
	msg.cell_height = grid_height

	point = Point()
	point.x = (x * cell_size) + Xoffset + (cell_size/2)
	point.y = (y * cell_size) + Yoffset + (cell_size/2)

	padding_cell_list.append(point)
	msg.cells = padding_cell_list
	pad_pub.publish(msg)

def publishBlockedCell(x, y):
	global block_pub
	global blocked_cell_list
	global cell_size
	global Xoffset
	global Yoffset

	msg = GridCells()

	grid_height = cell_size
	grid_width = cell_size

	header = Header()
	header.frame_id = "map"

	msg.header = header
	msg.cell_width = grid_width
	msg.cell_height = grid_height

	point = Point()
	point.x = (x * cell_size) + Xoffset + (cell_size/2)
	point.y = (y * cell_size) + Yoffset + (cell_size/2)

	blocked_cell_list.append(point)
	msg.cells = blocked_cell_list
	block_pub.publish(msg)


def publishFrontierCell(x, y):
	global frontier_pub
	global frontier_cell_list
	global cell_size
	global Xoffset
	global Yoffset

	msg = GridCells()

	grid_height = cell_size
	grid_width = cell_size

	header = Header()
	header.frame_id = "map"

	msg.header = header
	msg.cell_width = grid_width
	msg.cell_height = grid_height

	point = Point()
	point.x = (x * cell_size) + Xoffset + (cell_size/2)
	point.y = (y * cell_size) + Yoffset + (cell_size/2)

	frontier_cell_list.append(point)
	msg.cells = frontier_cell_list
	frontier_pub.publish(msg) 
	

def publishPathCell(x, y):
	global path_pub
	global path_cell_list
	global cell_size
	global Xoffset
	global Yoffset

	msg = GridCells()

	grid_height = cell_size
	grid_width = cell_size

	header = Header()
	header.frame_id = "map"

	msg.header = header
	msg.cell_width = grid_width
	msg.cell_height = grid_height

	point = Point()
	point.x = (x * cell_size) + Xoffset + (cell_size/2)
	point.y = (y * cell_size) + Yoffset + (cell_size/2)

	path_cell_list.append(point)
	msg.cells = path_cell_list
	path_pub.publish(msg) 

def odomCallback(event):
    global pose
    global xPosition
    global yPosition
    global theta

    odom_list.waitForTransform('map','base_footprint',rospy.Time(0),rospy.Duration(1.0))
    #finds the position and oriention of two objects relative to each other
    (position, orientation) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0)) 
    pose.pose.position.x = position[0]
    pose.pose.position.y = position[1]

    xPosition = position[0]
    yPosition = position[1]

    odomW = orientation
    q = [odomW[0],odomW[1],odomW[2],odomW[3]]
    roll, pitch, yaw = euler_from_quaternion(q)

    #convert yaw to degrees
    pose.pose.orientation.z = yaw
    theta = math.degrees(yaw)


if __name__ == '__main__':

	rospy.init_node('Mag_Men_Astar')

	global visited_pub, block_pub, frontier_pub, unknown_pub, path_pub, pad_pub
	global pose 
	global odom_tf
	global frontier_cell_list
	global path_cell_list
	global visited_cell_list
	global blocked_cell_list #for storing points
	global pad_cell_list #for storing nodes
	global obstacle_cell_list #for storing nodes
	global padding_cell_list #for storing points
	global goal
	global start
	global complete
	global cell_size
	global hasMap
	global rows, cols
	global Xoffset
	global Yoffset
	global hasStart
	global hasGoal
	global tolerance #used for comparing floats


	#initialize variables
	visited_cell_list = []
	path_cell_list = []
	frontier_cell_list = []
	blocked_cell_list = []
	pad_cell_list = []
	obstacle_cell_list = []
	padding_cell_list = []
	pose = PoseStamped()
	cell_size = 0.05
	Xoffset = -15.4
	Yoffset = -12.2
	start = Node(int(Xoffset / -cell_size), int(Yoffset / -cell_size),0)
	print ("start(init) = {},{}" .format(start.x,start.y))
	goal = Node(0,0,0)
	complete = True
	hasMap = False
	hasGoal = False
	#uncomment if want to tell start position
	#hasStart = False
	#comment if want to tell start position
	hasStart = True
	rows = 0
	cols = 0
	tolerance = .01 #used for comparing floats

	#initialize publishers
	pad_pub = rospy.Publisher('/padding', GridCells, queue_size = 25)
	visited_pub = rospy.Publisher('/visited', GridCells, queue_size = 10)
	block_pub = rospy.Publisher('/blocked', GridCells, queue_size = 10)
	frontier_pub = rospy.Publisher('/frontier', GridCells, queue_size = 10)
	path_pub = rospy.Publisher('/path', GridCells, queue_size = 10)
	waypoint_pub = rospy.Publisher('/waypoints', Waypoint, queue_size = 1)
	stop_pub = rospy.Publisher('/STOP' , Twist, queue_size = 1)

	#initialize subscribers
	map_sub = rospy.Subscriber('/map', OccupancyGrid, createGrid)
	roboMap_sub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, newCost)
	goal_sub = rospy.Subscriber('/clicked_point', PointStamped, updateGoal)
	#start_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, updateStart)
	reStar_sub = rospy.Subscriber('/newPoint', Point, reStar)

	
	print "Lab 3 Started"
	print "start(begin) {},{}".format(start.x,start.y)
	while(1 and not rospy.is_shutdown()):
		
		while(not complete and hasStart and hasGoal):
			print "start {},{}".format(start.x,start.y)
			AStar(start, goal)
		

	print "Lab 3 complete"
