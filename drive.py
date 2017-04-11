#!/usr/bin/env python

import rospy, tf, numpy, math
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion
from rpcole_lab2.msg import Waypoint


#drive to a goal subscribed as /move_base_simple/goal
def navToPose(goal):
    driveStraight(10, 60)
    print "spin!"
    #rotate to direction
    rotate(90)
    print "move!"
    #move set distance
    driveStraight(10,45)
    print "spin!"
    #rotate back to home
    rotate(90)
    print "done"


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
        if (currentDistance == distance):
        	atTarget = True

	publishTwist(0,0)           
      
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

    ##determine which way to turn based on angle

    while((abs(error) >= 2) and not rospy.is_shutdown()):
        
        if (angle > originTheta):
            vel = 0.35
        elif (angle < originTheta):
            vel = -0.35
        #recalc error and publish twist to move turtlebot
     	publishTwist(0.0,vel)
        error = angle-theta

    #rotation complete so stop movement
    vel = 0
    publishTwist(0.0,vel)


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
        executeTrajectory()


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

#takes the list of waypoints (start ==> goal) and drives the turtlebot along the path
def driveWaypoints(msg):
    global tolerance
    print "in drive"
    a = 1
    while ((a < len(msg.waypoints)) and not rospy.is_shutdown()):
        print "{}".format(a)
        if(msg.waypoints[a].x - msg.waypoints[a-1].x > tolerance):
            if(msg.waypoints[a].x > msg.waypoints[a-1].x):
                print "rotate to +x"
                rotate(90)
            elif(msg.waypoints[a].x < msg.waypoints[a-1].x):
                print "rotate to -x"
                rotate(-90)
            print "drive x"
            driveStraight(0.25, msg.waypoints[a].x-msg.waypoints[a-1].x)
        elif(msg.waypoints[a].y - msg.waypoints[a-1].y > tolerance):
            if (msg.waypoints[a].y < msg.waypoints[a-1].y):
                print "rotate to +y"
                rotate(0)
            if (msg.waypoints[a].y > msg.waypoints[a-1].y):
                print "rotate to -y"
                rotate (180)
            print "drive y"
            driveStraight(0.25, msg.waypoints[a].y-msg.waypoints[a-1].y)
        a += 1


# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('Mag_Men_drive_node')

    # These are global variables. Write "global <variable_name>" in any other function to gain access to these global variables 
    global Twist_pub
    global pose
    global odom_tf
    global odom_list
    global tolerance #used for comparing floats

    tolerance = .01 #used for comparing floats
    pose=PoseStamped()

    theta = 0
    # Replace the elipses '...' in the following lines to set up the publishers and subscribers the lab requires
    Twist_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size = 10) # Publisher for commanding robot motion
    bumper_sub = rospy.Subscriber('mobile_base/events/Bumper', BumperEvent, readBumper, queue_size=1)
    goal_sub = rospy.Subscriber('move_base_simple/goal',PoseStamped,navToPose,queue_size=1)
    waypoint_sub = rospy.Subscriber('/waypoints',Waypoint,driveWaypoints,queue_size = 10)
    rospy.Timer(rospy.Duration(1), odomCallback)

    # Use this object to get the robot's Odometry 
    odom_list = tf.TransformListener()
    odom_sub = rospy.Subscriber("/odom", Odometry, odomCallback)

    # Use this command to make the program wait for some seconds
    #rospy.sleep(rospy.Duration(1, 0))
    print "Starting Lab 2"

    #driveStraight(3,3)
    #rotate(90)

    while(1 and not rospy.is_shutdown()):
    	pass

    print "Lab 2 complete!"