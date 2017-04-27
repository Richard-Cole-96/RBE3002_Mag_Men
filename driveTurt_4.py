#!/usr/bin/env python

import rospy, tf, numpy, math
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose, Point
from tf.transformations import euler_from_quaternion
from rpcole_lab2.msg import Waypoint
import time


#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():
	driveStraight(.25,.6)
	rotate(90)
	driveStraight(.25,.45)
	rotate(-135)

wheelSpacing=9.0

#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):
	#drive wheel1 at u1 for time
	#drive wheel2 at u2 for time
	lin_vel = (u1 + u2)/2.0
	ang_vel = (u1 - u2)/wheelSpacing

	twist_msg = Twist();
	stop_msg = Twist();

	twist_msg.linear.x = lin_vel
	twist_msg.angular.z = ang_vel
	stop_msg.linear.x = 0
	stop_msg.angular.z = 0
	now = rospy.Time.now().secs
	while(rospy.Time.now().secs - now  <= time and not rospy.is_shutdown()):
		Twist_pub.publish(twist_msg)
	Twist_pub.publish(stop_msg)

#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
	#power both wheels
	#if distance, measured by encoders, reached - stop
	global pose
	initialx = pose.pose.position.x
	initialy = pose.pose.position.y
	atTarget = False

	#print "RUNNING RUNNIGN RUNNING"

	while (not atTarget and not rospy.is_shutdown()):
		currentX = pose.pose.position.x
		currentY = pose.pose.position.y
		#print "loop"
		#print currentX
		#print speed
		currentDistance = math.sqrt(((currentX - initialx)**2) + ((currentY - initialy)**2))
		publishTwist(speed,0)
		rospy.sleep(rospy.Duration(.005, 0))
		if (currentDistance >= distance):
			atTarget = True

	publishTwist(0,0) 
	rospy.sleep(rospy.Duration(.005, 0))
	print "distance traveled = {}".format(currentDistance)          
	print "ARRIVED!"
	#print "ok next"

#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
	#drive wheel one forwards
	#drive wheel two backwards
	#drive until angle reached via encoder counts (or imu?)
	global odom_list
	global pose 
	global theta
	global originTheta

	vel = 0

	#set starting angle for referance later
	originTheta = theta

	if(angle > 180):
		print "angle is too large"
		excess = angle - 180
		angle = -180 + excess
	if(angle < -180):
		print "angle is too small"
		excess = angle + 180
		angle = -180 + excess

	error = angle-theta
	error2 = 720

	##determine which way to turn based on angle

	while((abs(error) >= 2) and not rospy.is_shutdown()): 

		if (angle > originTheta):
			vel = 0.3
		elif (angle < originTheta):
			vel = -0.3
		#recalc error and publish twist to move turtlebot
		publishTwist(0.0,vel)
		rospy.sleep(rospy.Duration(.02, 0))
		
		#error 
		error2 = error
		error = angle-theta

		#print "theta = {}, error = {}".format(theta,error)
		#print ""

		# in case we are looking for 180 and -180 and overshoot
		if (abs(abs(error2) - abs(error)) > 90):
			print "range flip"
			vel = 0
			publishTwist(0.0, vel)
			rospy.sleep(rospy.Duration(.02, 0))
			error = 0

	#rotation complete so stop movement
	vel = 0
	publishTwist(0.0,vel)
	rospy.sleep(rospy.Duration(.02, 0))

#navigates to a pose
def navToPose(goal):
	global yPosition
	global theta
	global xPosition
	global pose

	#calc initial position for referance later
	iX = xPosition
	iY = yPosition

	#get all info about the pose
	goalX = goal.pose.position.x
	goalY = goal.pose.position.y

	quat = goal.pose.orientation
	quat = [quat.x, quat.y, quat.z, quat.w]

	roll, pitch, yaw = euler_from_quaternion(quat)

	#convert yaw to degrees
	goalAng = math.degrees(yaw)

	print "spin!"
	firstAng = math.degrees(math.atan2(goalY-yPosition,goalX-xPosition))
	if (firstAng < -180):
		print "TOO SMALL!!!!!"
		firstAng = firstAng + 360
	if (firstAng > 180):
		print "TOO LARGE!!!!!"
		firstAng = firstAng - 360
	rotate(firstAng-theta)

	print "move!"
	curDist = math.sqrt(((iX - goalX) ** 2)+ ((iY - goalY) ** 2))
	driveStraight(.25,curDist)

	print "spin!"
	rotate(goalAng - theta)

	print "done"

#This function works the same as rotate how ever it does publish linear velocities.
def driveArc(radius, speed, angle):
	global pose
	global xPosition
	global yPosition
	global theta

	#calc desited angular velocity
	angVel = speed/radius

	#calc needed destination angle
	goalAng = theta + angle

	while ((theta <= goalAng) and not rospy.is_shutdown()):
		
		publishTwist(speed * .25,angVel * .25)
		#print "dest=%d, angle=%d" % (goalAng,theta)

#Bumper Event Callback function
def readBumper(msg):
	if (msg.state == 1):
		print "bumper"
		#executeTrajectory()
		driveStraight(0.1,0.5)

# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
# Start the timer with the following line of code: 
#   rospy.Timer(rospy.Duration(.01), timerCallback)
def timerCallback(event):

	pass # Delete this 'pass' once implemented

#publish Twist message using the Twist_pub publisher 
#based upon the given x velocity and angular velocity around the z-axis
def publishTwist(linearVelocity, angularVelocity):
	global Twist_pub
	msg = Twist()
	msg.linear.x = linearVelocity
	msg.angular.z = angularVelocity
	Twist_pub.publish(msg)

def odomCallback(event):
	global pose
	global xPosition
	global yPosition
	global theta
	global hasPos

	odom_list.waitForTransform('map','base_footprint',rospy.Time(0),rospy.Duration(1.0))
	#finds the position and oriention of two objects relative to each other
	(position, orientation) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0)) 
	pose.pose.position.x = position[0]
	pose.pose.position.y = position[1]

	xPosition = position[0]
	yPosition = position[1]
	#print "position {},{}".format(xPosition,yPosition)

	odomW = orientation
	q = [odomW[0],odomW[1],odomW[2],odomW[3]]
	roll, pitch, yaw = euler_from_quaternion(q)

	#convert yaw to degrees
	pose.pose.orientation.z = yaw
	theta = math.degrees(yaw)

	hasPos = True    

def sendLocation():
	global xPosition
	global yPosition
	global loc_pub

	newMsg = Point()

	newMsg.x = xPosition
	newMsg.y = yPosition

	loc_pub.publish(newMsg)

def sevenTwenty(msg):

	sendMsg = Point()

	print "rotate to 180"
	rotate(180)
	rospy.sleep(rospy.Duration(2, 0))
	print "rotate to 0"
	rotate(0)
	rospy.sleep(rospy.Duration(2, 0))
	print "rotate to -180"
	rotate(-180)
	rospy.sleep(rospy.Duration(2, 0))
	#print "rotate to 0"
	#rotate(0)
	#print "rotate to 180"
	#rotate(180)
	print "need to update map"

	rospy.sleep(rospy.Duration(3, 0))

	sendMsg.x = xPosition
	sendMsg.y = yPosition
	print "SEVENTWENTY"

	begin_pub.publish(sendMsg)

	rospy.sleep(rospy.Duration(3, 0))

	print "rotate to 0 to force update"
	rotate(0)

#drive to the second to last waypoint in the list, spins twice, then forces update of map
def driveWaypoints(msg):
	sendMsg = Point()
	
	global findFront_pub
	global xPosition
	global yPosition
	global tolerance
	print "in drive"

	a = 0

	while ((a < len(msg.waypoints) - 2) and not rospy.is_shutdown()):   
		if(abs(msg.waypoints[a+1].x - msg.waypoints[a].x) >= 0.001):
			if(msg.waypoints[a].x > msg.waypoints[a+1].x):
				print "rotate to +x"
				rotate(180)
			elif(msg.waypoints[a].x < msg.waypoints[a+1].x):
				print "rotate to -x"
				rotate(0)
			print "drive1 = {}, drive2 = {}".format(msg.waypoints[a].x,msg.waypoints[a+1].x)
			print "drive x {}".format(msg.waypoints[a].x-msg.waypoints[a+1].x)
			driveStraight(0.25, msg.waypoints[a].x-msg.waypoints[a+1].x)
		elif(abs(msg.waypoints[a+1].y - msg.waypoints[a].y) >= 0.001):
			if (msg.waypoints[a].y < msg.waypoints[a+1].y):
				print "rotate to +y"
				rotate(90)
			if (msg.waypoints[a].y > msg.waypoints[a+1].y):
				print "rotate to -y"
				rotate (-90) 
			print "drive1 = {}, drive2 = {}".format(msg.waypoints[a].y,msg.waypoints[a+1].y)
			print "drive y {}".format(msg.waypoints[a].y-msg.waypoints[a+1].y)
			driveStraight(0.25, msg.waypoints[a].y-msg.waypoints[a+1].y)
		#sends the new location after it drives to the waypoint
		a += 1
	#if the message list is only two, drive half the distance to the final position
	if(len(msg.waypoints) == 2):
		if(abs(msg.waypoints[a+1].x - msg.waypoints[a].x) >= 0.001):
			if(msg.waypoints[a].x > msg.waypoints[a+1].x):
				print "rotate to +x"
				rotate(180)
			elif(msg.waypoints[a].x < msg.waypoints[a+1].x):
				print "rotate to -x"
				rotate(0)
			print "drive1 = {}, drive2 = {}".format(msg.waypoints[a].x,msg.waypoints[a+1].x)
			print "drive x {}".format((msg.waypoints[a].x-msg.waypoints[a+1].x)/2)
			driveStraight(0.25, (msg.waypoints[a].x-msg.waypoints[a+1].x)/2)
		elif(abs(msg.waypoints[a+1].y - msg.waypoints[a].y) >= 0.001):
			if (msg.waypoints[a].y < msg.waypoints[a+1].y):
				print "rotate to +y"
				rotate(90)
			if (msg.waypoints[a].y > msg.waypoints[a+1].y):
				print "rotate to -y"
				rotate (-90) 
			print "drive1 = {}, drive2 = {}".format(msg.waypoints[a].y,msg.waypoints[a+1].y)
			print "drive y {}".format((msg.waypoints[a].y-msg.waypoints[a+1].y)/2)
			driveStraight(0.25, (msg.waypoints[a].y-msg.waypoints[a+1].y)/2)

	#rotates twice and then publishes message telling map to update and find the next frontier
	print "rotate to update gMapping"
	print "rotate to 180"
	rotate(180)
	rospy.sleep(rospy.Duration(2, 0))
	print "rotate to 0"
	rotate(0)
	rospy.sleep(rospy.Duration(2, 0))
	print "rotate to -180"
	rotate(-180)
	rospy.sleep(rospy.Duration(2, 0))
	print "need to update map"

	rospy.sleep(rospy.Duration(2, 0))

	sendMsg.x = xPosition
	sendMsg.y = yPosition
	print "newLocation {} {}".format(sendMsg.x, sendMsg.y)
	findFront_pub.publish(sendMsg)
	
	rospy.sleep(rospy.Duration(3, 0))

	print "rotate to 0 to force map update"
	rotate(0)

def stop_Robot(msg):
	print "Search Complete, stopping Turtlebot"
	publishTwist(0,0)

# This is the program's main function
if __name__ == '__main__':

	# Change this node name to include your username
	rospy.init_node('Mag_Men_drive_node')

	# These are global variables. Write "global <variable_name>" in any other function to gain access to these global variables 
	global Twist_pub
	global newFrontierGoal_pub
	global pose
	global odom_tf
	global odom_list
	global tolerance #used for comparing floats
	global hasPos

	tolerance = .01 #used for comparing floats
	pose=PoseStamped()
	hasPos = False

	theta = 0
   
	Twist_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size = 10) # Publisher for commanding robot motion
	bumper_sub = rospy.Subscriber('mobile_base/events/Bumper', BumperEvent, readBumper, queue_size=1)
	goal_sub = rospy.Subscriber('move_base_simple/goal',PoseStamped, navToPose,queue_size=1)
	stop_sub = rospy.Publisher('/STOP', Twist, stop_Robot, queue_size = 10)

	sevenTwenty_sub = rospy.Subscriber('/sevenTwenty', Point, sevenTwenty)

	loc_pub = rospy.Publisher('/newPoint', Point, queue_size = 1)
	findFront_pub = rospy.Publisher('/findNextFront',Point, queue_size = 1)
	begin_pub = rospy.Publisher('/begin', Point, queue_size = 1)

	rospy.Timer(rospy.Duration(1), odomCallback)

	# Use this object to get the robot's Odometry
	odom_list = tf.TransformListener()
	odom_sub = rospy.Subscriber("/odom", Odometry, odomCallback)

	# Use this command to make the program wait for some seconds
	#rospy.sleep(rospy.Duration(1, 0))
	print "Starting drive"

	while(not hasPos):
		pass

	print "has position"
	#start subscribers taht need position to work
	waypoint_sub = rospy.Subscriber('/waypoints',Waypoint,driveWaypoints,queue_size = 10)


	#driveStraight(0.25,0.5)
	#rotate(90)
	#driveStraight(0.25,0.5)

	while(1 and not rospy.is_shutdown()):
		pass

	print "Lab 2 complete!"
