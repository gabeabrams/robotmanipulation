#!/usr/bin/env python

import rospy
from gabe_ricky_sarah_proj1.srv import*
from gabe_ricky_sarah_proj1.msg import*
from std_msgs.msg import String

import ai

#########################GLOBAL DATA###########################

worldState = WorldState()
goalState = ""

blockLocaleRow = 0
blockLocaleCol = 0

kill = False
######################NETWORKING FUNCTS########################
def worldStateReceived(data):
	#Receives the world state from robot_interface
	global worldState
	worldState = data

def commandReceived(data):
	#Handles custom commands from terminal
	global goalState
	if data != goalState:
		global kill
		goalState = data.data
		requestWorldState()
		kill = True
		rospy.sleep(10)
		MakeAIControlRobot()	

def requestWorldState():
	#Requests the world state from robot_interface
	rospy.wait_for_service('get_state')
	global worldState
	try:
		WorldStateRequest = rospy.ServiceProxy("get_state",WorldState_Request)
		StateResponse = WorldStateRequest('Whirled steight pls')
		worldState = StateResponse.state
	except rospy.ServiceException, e:
		return None

######################INIT FUNCTIONS########################
def initNetwork():
	"Initializes controller node networks"
	# Subscribe to world state publisher
	rospy.Subscriber("world_state_connection", WorldState, worldStateReceived)
	
	# Subscribe to terminal commands
	rospy.Subscriber("command", String, commandReceived)

def MakeAIControlRobot():
	global worldState
	global goalState
	global rightActions
	global leftActions
	global kill
	kill = False

	print worldState	
	print ""	

	rightActions = []
	leftActions = []

	# Setup times
	secondsBetweenActions = 1.5

	if isOneArmSolution:
		((rightActions,r),(leftActions, l)) = ai.heyAIWhatsNext(worldState, goalState, 1)
		
		while (r < len(rightActions) - 1) and kill == False:
			(r, dataBack) = ai.heyAIDoNext(((rightActions,r), (leftActions, l)), 1)
			rospy.sleep(secondsBetweenActions)
			if dataBack == False:
				print "Action failed! Recalculating 1 Arm Solution."
				((rightActions,r)) = ai.heyAIWhatsNext(worldState, goalState, 1)
		if kill == True:
			kill = False	
	else:
		((rightActions,r),(leftActions,l)) = ai.heyAIWhatsNext(worldState, goalState, 2)

		while (r < len(rightActions) - 1) and kill == False:
			(((rightActions,r),(leftActions,l)), dataBack) = ai.heyAIDoNext(((rightActions,r),(leftActions,l)), 2)
			rospy.sleep(secondsBetweenActions)
			if dataBack == False:
				print "Action failed! Recalculating 2 Arm Solution."
				((rightActions,r),(leftActions,l)) = ai.heyAIWhatsNext(worldState, goalState, 2)
		if kill == True:
			kill == False
	rospy.sleep(1)	
	
	print "I did it!"
	print ""
	# gridRows, gridCols, numBlocks, localeRow, localeCol, configuration, goalState, isOneArmSpolution

	
def readParams():
	print "Reading Parameters:"
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
	print "- Grid dimensions: (" + str(gridRows) + ", " + str(gridCols) + ")"
	numBlocks = rospy.get_param("numBlocks")
	print "- Number of blocks: " + str(numBlocks)
	blockLocaleRow = rospy.get_param("blockLocaleRow")
	blockLocaleCol = rospy.get_param("blockLocaleCol")
	print "- Home location: (" + str(blockLocaleRow) + ", " + str(blockLocaleCol) + ")"
	configuration = rospy.get_param("configuration")
	print "- Initial configuration: " + configuration
	goalState = rospy.get_param("goalState")
	print "- Final configuration: " + goalState
	isOneArmSolution = rospy.get_param("isOneArmSolution")
	if isOneArmSolution:
		print "- We will be using one arm"
	else:
		print "- We will be using two arms"

	

def initController():
	# Create controller node
	rospy.init_node("controller")
	print "\n\n\n\n\n\n\n\n\n\nController Node Initialized!\n-----------------------------------------"
	readParams()
 	print "-----------------------------------------"   

    # Get values from robot_interface globals
	global blockLocaleRow
	global blockLocaleCol

	# Create the network
	print "Waiting for network..."
	initNetwork()
	requestWorldState()
	ai.sendHomeLoc(blockLocaleRow, blockLocaleCol)
	print "-----------------------------------------"
	print "Brilliant. Let's get started!"
	MakeAIControlRobot()

	rospy.spin()

if __name__ == '__main__':
	initController()
