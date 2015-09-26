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
		rospy.sleep(2)
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

	print ""

	rightActions = []
	leftActions = []

	if isOneArmSolution:
		((rightActions,r),(leftActions, l)) = ai.heyAIWhatsNext(worldState, goalState, 1)
		
		while (r < len(rightActions) - 1) and kill == False:
			(r, dataBack) = ai.heyAIDoNext(((rightActions,r), (leftActions, l)), 1)
			rospy.sleep(1)
			if dataBack == False:
				print "Action failed! Recalculating 1 Arm Solution."
				((rightActions,r)) = ai.heyAIWhatsNext(worldState, goalState, 1)
		if kill == True:
			kill = False	
	else:
		((rightActions,r),(leftActions,l)) = ai.heyAIWhatsNext(worldState, goalState, 2)

		while (r < len(rightActions) - 1) and kill == False:
			(((rightActions,r),(leftActions,l)), dataBack) = ai.heyAIDoNext(((rightActions,r),(leftActions,l)), 2)
			rospy.sleep(3)
			if dataBack == False:
				print "Action failed! Recalculating 2 Arm Solution."
				((rightActions,r),(leftActions,l)) = ai.heyAIWhatsNext(worldState, goalState, 2)
		if kill == True:
			kill == False
	rospy.sleep(1)	
	
	print "Starting from:"
	print worldState	
	print ""	
	
def initController(gridRows, gridCols, numBlocks, localeRow, localeCol, configuration, goalState, isOneArmSpolution):
	# Create controller node
	rospy.init_node("controller")

	global blockLocaleRow
	global blockLocaleCol
	blockLocaleRow = localeRow
	blockLocaleCol = localeCol

	# Create the network
	initNetwork()
	requestWorldState()
	ai.sendHomeLoc(blockLocaleRow, blockLocaleCol)

	MakeAIControlRobot()

	rospy.spin()
	
def readParams():
	print "Reading Parameters!"

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
	
	initController(gridRows,gridRows,numBlocks,blockLocaleRow,blockLocaleCol,configuration,goalState,isOneArmSolution)

if __name__ == '__main__':
	ParamsBeingRead = True
	if ParamsBeingRead:
		readParams()
	else:
		gridRows = 3
		gridCols = 3
		numBlocks = 3
		blockLocaleRow = 2
		blockLocaleCol = 2
		configuration = "stacked_ascending"
		goalState = "stacked_descending"
		isOneArmSolution = False
		initController(gridRows,gridRows,numBlocks,blockLocaleRow,blockLocaleCol,configuration,goalState,isOneArmSolution)
