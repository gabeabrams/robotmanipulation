#!/usr/bin/env python

import rospy
from gabe_ricky_sarah_proj1.srv import*
from gabe_ricky_sarah_proj1.msg import*
from std_msgs.msg import String

#########################GLOBAL DATA###########################

requestWorldState_Connection = None
worldState = WorldState()

######################NETWORKING FUNCTS########################
def worldStateReceived(data):
	"Receives the world state from robot_interface"
	global worldState
	worldState = data

def commandReceived(data):
	"Handles custom commands from terminal"
	# Receives commands from terminal
	return 1

def moveRobotRequest():
	return 1

def requestWorldState():
	"Requests the world state from robot_interface"
	rospy.wait_for_service('get_state')
	global requestWorldState_Connection
	global worldState
	try:
		if requestWorldState_Connection == None:
		   requestWorldState_Connection = rospy.ServiceProxy("get_state",WorldState_Request)
		worldState = requestWorldState_Connection()
	except rospy.ServiceException, e:
		return None

######################INIT FUNCTIONS########################
def initNetwork():
	"Initializes controller node networks"
	# Subscribe to world state publisher
	rospy.Subscriber("world_state_connection", WorldState, worldStateReceived)
	
	# Subscribe to terminal commands
	rospy.Subscriber("command", String, commandReceived)

def readParams():
	# Reads ROS Parameters from launch file
	global gridRows
	global gridCols
	global numBlocks
	global blockLocaleRow
	global blockLocaleCol
	global configuration
	global goalState
	global isOneArmSolution
	
	gridRows = rospy.get_param("gridRows")
	gridCols = rospy.get_param("gridCols")
	numBlocks = rospy.get_param("numBlocks")
	blockLocaleRow = rospy.get_param("blockLocaleRow")
	blockLocaleCol = rospy.get_param("blockLocaleCol")
	configuration = rospy.get_param("configuration")
	goalState = rospy.get_param("goalState")
	isOneArmSolution = rospy.get_param("isOneArmSolution")	
	
def initController(gridRows, gridCols, numBlocks, blockLocaleRow, blockLocaleCol, configuration, goalState, isOneArmSpolution):
	"First function to initialize the controller node"
	# Create controller node
	rospy.init_node("controller")
	
	# Create the network
	initNetwork()

	requestWorldState()

	rospy.spin()

if __name__ == '__main__':
	ParamsBeingRead = 0
	#readParams(); ParamsBeingRead = 1
	if ParamsBeingRead == 0:
		gridRows = 5
		gridCols = 5
		numBlocks = 3
		blockLocaleRow = 3
		blockLocaleCol = 3
		configuration = "ascending"
		goalState = "descending"
		isOneArmSolution = False
	initController(gridRows,gridRows,numBlocks,blockLocaleRow,blockLocaleCol,configuration,goalState,isOneArmSolution)
