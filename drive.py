#!/usr/bin/env python

import rospy, tf, numpy, math
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion
# Add additional imports for each of the message types used


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
    #trajector methods

    pass  # Delete this 'pass' once implemented

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
      # Delete this 'pass' once implemented



#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    #power both wheels
    #if distance, measured by encoders, reached - stop
    global pose
    initialx = pose.position.x
    initialy = pose.position.y
    atTarget = False

    print "RUNNING RUNNIGN RUNNING"

    while (not atTarget and not rospy.is_shutdown()):
        currentX = pose.position.x
        currentY = pose.position.y
        #print "loop"
        #print currentX
        #print speed
        currentDistance = math.sqrt(((currentX - initialx)**2) + ((currentY - initialy)**2))
        publishTwist(speed,0)
        if (currentDistance == distance):
        	atTarget = True

	publishTwist(0,0)           
      
    print "ok next"

    
#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
    #drive wheel one forwards
    #drive wheel two backwards
    #drive until angle reached via encoder counts (or imu?)
    global odom_list
    global pose 
    global theta
    global originTheta

    originTheta = 0

    if (angle > 180 or  angle <- 180):
        print "angle too large or small"
    vel = Twist();
    done = True

    originTheta = originTheta + theta

    #set rotation direction
    angle = angle + originTheta
    error = angle-theta

    ##determine which way to turn based on angle

    while((abs(error) >= 2) and not rospy.is_shutdown()):
        
         if (angle > originTheta):
            vel.angular.z = 0.5
         elif (angle < originTheta):
            vel.angular.z = -0.5
     
     	 error = angle-theta
         Twist_pub.publish(vel)

    vel.angular.z = 0
    Twist_pub.publish(vel)

    # Delete this 'pass' once implemented

def navToPose(goal):
	pass


#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
    #use forward kinematics here to determine wheelindividual wheel speeds
    #use speed, to find time to traverse arc angle
    #run for said time
    pass  # Delete this 'pass' once implemented





#Bumper Event Callback function
def readBumper(msg):
    if (msg.state == 1):
        executeTrajectory()
         # Delete this 'pass' once implemented



# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
# Start the timer with the following line of code: 
#   rospy.Timer(rospy.Duration(.01), timerCallback)
def timerCallback(event):

    pass # Delete this 'pass' once implemented

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
    print "hello"
    odom_list.waitForTransform('odom', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
    print "bye"
    (position, orientation) = odom_list.lookupTransform('odom','base_footprint',rospy.Time(0))
    pose.position.x=position[0]
    pose.position.y=position[1]

    odomW = orientation
    q = [odomW[0], odomW[1], odomW[2], odomW[3]]
    roll, pitch, yaw = euler_from_quaternion(q)
    print pose.position.x
    pose.orientation.z = yaw
    theta = math.degrees(yaw)
    


# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('sample_Lab_2__node_jltai')

    # These are global variables. Write "global <variable_name>" in any other function to gain access to these global variables 
    global Twist_pub
    global pose
    global odom_tf
    global odom_list
   
    pose=Pose()

    theta = 0
    # Replace the elipses '...' in the following lines to set up the publishers and subscribers the lab requires
    Twist_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size = 10) # Publisher for commanding robot motion
    bumper_sub = rospy.Subscriber('mobile_base/events/Bumper', BumperEvent, readBumper, queue_size=1)
    #Odom_sub = rospy.Subscriber("/odom", Odometry, odomCallback)
    rospy.Timer(rospy.Duration(1), odomCallback)

    # Use this object to get the robot's Odometry 
    odom_list = tf.TransformListener()
    
    # Use this command to make the program wait for some seconds
    #rospy.sleep(rospy.Duration(1, 0))
    print "Starting Lab 2"

    #make the robot keep doing something...
    #while(1):
    #	pass
    # Make the robot do stuff...

    #driveStraight(3,3)
    rotate(90)

    run_t = 300
    start_t = rospy.Time.now().secs
    while(rospy.Time.now().secs - start_t <= run_t and not rospy.is_shutdown()):
    	print "running running running"
    print "Lab 2 complete!"