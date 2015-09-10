#!/usr/bin/env python

import rospy
from gabe_ricky_sarah_proj1.srv import*
from gabe_ricky_sarah_proj1.msg import*
from std_msgs.msg import String

#Reacts to command topic
def ControlMode(data):
	return 1
	#Recalculate command list iff command changes

#Reacts to worldstate topic
def StateKeeper(data):
	return 1
	#Change locally held state variables


def controller():
	rospy.init_node('controller')

	#Subscribe to /command
	rospy.Subscriber("command", String, ControlMode)

	#Subscribe to /worldstate
	rospy.Subscriber('State', WorldState, StateKeeper)

	#Call Service worldstate
	try:
	    get_state = rospy.ServiceProxy('get_state', WorldState_Request)
		#get state
		#change locally held state variables
	except rospy.ServiceException, e:	
		return 1

	#Call Service moverobot 
	try:
	    move_robot = rospy.ServiceProxy('move_robot', MoveRobot)
	    return 1
		#place move requests
		#if true, continue
		#if false, recalculate commands
	except rospy.ServiceException, e:	
		return 1

	rospy.spin()	

if __name__ == '__main__':
	controller()
	print "Doing something I swear"