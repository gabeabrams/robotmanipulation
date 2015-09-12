#!/usr/bin/env python

import rospy
from gabe_ricky_sarah_proj1.srv import*
from gabe_ricky_sarah_proj1.msg import*
from std_msgs.msg import String

#########################GLOBAL DATA###########################

requestWorldState_Connection = None

######################NETWORKING FUNCTS########################
def worldStateReceived(data):
	"Receives the world state from robot_interface"
	# TODO: record world state
	return 1

def commandReceived(data):
	"Handles custom commands from terminal"
	# Receives commands from terminal
	return 1
	
def requestWorldState():
	"Requests the world state from robot_interface"
	try:
		if requestWorldState_Connection == None:
			rospy.wait_for_service('get_state')
			requestWorldState_Connection = rospy.ServiceProxy("get_state",WorldState_Request)
		return requestWorldState_Connection() # TODO: NOTHING TO PASS. WHAT DO WE DO?
	catch rospy.ServiceException, e:
		return None

######################INIT FUNCTIONS########################
def initNetwork():
	"Initializes controller node networks
	# Subscribe to world state publisher
	rospy.Subscriber("world_state_connection", WorldState, worldStateReceived)
	
	# Subscribe to terminal commands
	rospy.Subscriber("command", String, commandReceived)
	
def initController():
	"First function to initialize the controller node"
	# Create controller node
	rospy.init_node("controller")
	
	# Create the network
	initNetwork()

if __name__ == '__main__':
	initController()

######################OLD FUNCTIONS########################

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
	rospy.Subscriber("command", String, ControlMode) # ???? WHAT IS THIS?
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

