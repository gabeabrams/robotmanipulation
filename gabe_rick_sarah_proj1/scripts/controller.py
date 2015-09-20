#!/usr/bin/env python

import rospy
from gabe_ricky_sarah_proj1.srv import*
from gabe_ricky_sarah_proj1.msg import*
from std_msgs.msg import String

import move_planner

#########################GLOBAL DATA###########################

worldState = WorldState()
goalState = ""
stepNumber = 0
totalSteps = 0
finishedState = ""


######################NETWORKING FUNCTS########################
def worldStateReceived(data):
	#Receives the world state from robot_interface
	global worldState
	worldState = data

def commandReceived(data):
	#Handles custom commands from terminal
	if data != goalState:
		goalState = data
		requestworldState()
		startAnActionList(True)

def moveRobotRequest(req):
	rospy.wait_for_service('move_robot')
	try:
		MakeMove = rospy.ServiceProxy("move_robot",MoveRobot)
		robotMoved = MakeMove(req.rightArmAction, req.leftArmAction, req.rightArmTarget, req.leftArmTarget)
		if robotMoved != True:
			requestWorldState()
			startAnActionList(True)
		else:
			stepNumber += 1
	except rospy.ServiceException, e:
		"Nothing happens"

def requestWorldState():
	#Requests the world state from robot_interface
	rospy.wait_for_service('get_state')
	global requestWorldState_Connection
	global worldState
	try:
		WorldStateRequest = rospy.ServiceProxy("get_state",WorldState_Request)
		worldState = WorldStateRequest('Whirled steight pls')
	except rospy.ServiceException, e:
		return None

def startAnActionList(isFallback):
	global stepNumber
	global totalSteps
	global finishedState

	if isFallback:
		if finishedState != "":
			if isOneArmSolution:
				ServiceCallList = move_planner.oneArm(finishedState, goalState, blockLocaleRow, blockLocaleCol, numBlocks)
			else:
				ServiceCallList = move_planner.twoArms(finishedState, goalState, blockLocaleRow, blockLocaleCol, numBlocks)	
		else:
			ServiceCallList = move_planner.fallback(worldState, goalState, blockLocaleRow, blockLocaleCol, numBlocks)

	else: 
		if isOneArmSolution:
			ServiceCallList = move_planner.oneArm(finishedState, goalState, blockLocaleRow, blockLocaleCol, numBlocks)
		else:
			ServiceCallList = move_planner.twoArms(finishedState, goalState, blockLocaleRow, blockLocaleCol, numBlocks)

	stepNumber = 1
	totalSteps = len(ServiceCallList)

	while stepNumber <= totalSteps:
		finishedState = ""
		moveRobotRequest(ServiceCallList[stepNumber-1])
		stepNumber += 1

	if stepNumber == totalSteps:
		finishedState = goalState			


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
	# Create controller node
	rospy.init_node("controller")
	
	# Create the network
	initNetwork()
	requestWorldState()

	global serviceCallList
	global totalSteps
	global stepNumber

	if isOneArmSolution:
		ServiceCallList = startAnActionList(False)
	else:
		ServiceCallList = startAnActionList(False)

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
		configuration = "stacked_ascending"
		goalState = "stacked_descending"
		isOneArmSolution = False
	initController(gridRows,gridRows,numBlocks,blockLocaleRow,blockLocaleCol,configuration,goalState,isOneArmSolution)
