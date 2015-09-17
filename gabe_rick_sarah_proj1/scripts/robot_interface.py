#!/usr/bin/env python

import rospy
import random
from gabe_ricky_sarah_proj1.srv import*
from gabe_ricky_sarah_proj1.msg import*

#########################GLOBAL DATA###########################

#World State
worldState = WorldState()

#Save the last action performed
lastAction = 5; #1st action was action.MOVE_TO_BLOCK
holding = None;
lastHeld = None;
lastTarget = 0;

leftLastAction = None;
leftHolding = None;
leftLastHeld = None;
leftLastTarget = None;
home = [0,0];

#######################WORLD STATE FUNCTIONS########################

def getWorldState():
	"Get the world state object"
	return worldState
	
def updateGripperState(isOpen):
	"Change the gripper state in worldState"
	global worldState
	worldState.gripperOpen = isOpen

def getGripperIsOpen():
	global worldState
	return worldState.gripperOpen
	
def initWorldState(rows,cols):
	"Initialize the world state with an empty grid"
	global worldState
	worldState.grid = Grid()
	worldState.grid.stacks = []
	
	worldState.grid.dimensions = Coord()
	worldState.grid.dimensions.row = rows
	worldState.grid.dimensions.col = cols
	
	worldState.gripperOpen = True
	

def initBlocksInStack(configuration,numBlocks,row,col):
	"Put a stack of numBlocks blocks in row,col"
	# Ascending    Descending
	#   [3]           [1]
	#   [2]           [2]
	#   [1]           [3]
	#"""""""""""""""""""""""""
	global home
	home = [row,col];

	newStack = Stack()
	newStack.row = row
	newStack.col = col

	global lastTarget 
	
	if configuration == "stacked_ascending":
		newStack.blocks = range(1,numBlocks+1)
		lastTarget = numBlocks
		worldState.grid.stacks.append(newStack)

	# Reverse the stack if descending
	if configuration == "stacked_descending":
		newStack.blocks = range(1,numBlocks+1)
		newStack.blocks.reverse() 
		lastTarget = 1
		worldState.grid.stacks.append(newStack)

	if configuration == "scattered":
		for i in range (1,numBlocks+1):
			(row, col) = getRandomTableLocation(0)
			addBlockToWS(i, row, col)
		lastTarget = numBlocks	
			
def addBlockToWS(blockID, row, col):
	"Add a block to world state on top of stack"
	added = False
	for stack in worldState.grid.stacks:
		if stack.row == row and stack.col == col:
			# Found an existing stack!
			stack.blocks.append(blockID)
			added = True
	
	if not added:
		newStack = Stack()
		newStack.row = row
		newStack.col = col
		newStack.blocks = [blockID]
		worldState.grid.stacks.append(newStack)
	return worldState

def removeBlockFromWS(blockID):
	"Remove block from world state"
	for stack in worldState.grid.stacks:
		if blockID in stack.blocks:
			stack.blocks.remove(blockID)

	for stack in worldState.grid.stacks:
		if stack.blocks == []:
			worldState.grid.stacks.remove(stack)

	return worldState

def moveBlockInWS(blockID, row, col):
	"Updates worldState with blockID moved from its current location to (row,col). Does not send commands to baxter"
	removeBlockFromWS(blockID)
	addBlockToWS(blockID,row,col)
	return worldState

def getBlockInfo(blockID, targetType):
	"Returns block info including depth: 1=top, len=bottom"
	found = False
	row = 0
	col = 0
	depth = 0

	if targetType == -1:
		for stack in worldState.grid.stacks:
			if blockID in stack.blocks:
				row = stack.row
				col = stack.col
				depth = len(stack.blocks) - stack.blocks.index(blockID)
				found = True

	elif targetType == 3:	
		(row,col) = home
		depth = 1
		for stack in worldState.grid.stacks:
			if stack.row == row and stack.col == col:
				newBlock = stack.blocks[-1]
				(found, row, col, depth) = getBlockInfo(newBlock, -1) 
		found = True		

	else:
		(row,col) = getRandomTableLocation(targetType)
		depth = 1
		found = True	

	return (found,row,col,depth)

def getStackInWS(row,col):
	for stack in worldState.grid.stacks:
		if stack.row == row and stack.col == col:
			return stack
	return None

def getRandomTableLocation(targetType):
	"Returns row and column of random open table location"
	succeeded = False
	taken = False
	row = 0
	col = 0
	global home
	homerow = home[0]
	homecol = home[1]
	
	while not succeeded:
		if targetType == 0:
			row = random.randint(0, worldState.grid.dimensions.row)
			col = random.randint(0, worldState.grid.dimensions.col)
		elif targetType == 1:	
			row = random.randint(0, homerow)
			col = random.randint(0, worldState.grid.dimensions.col)
		elif targetType == 2:	
			row = random.randint(homerow, worldState.grid.dimensions.row)
			col = random.randint(0, worldState.grid.dimensions.col)
		for stack in worldState.grid.stacks:
			if row == stack.row and col == stack.col:
				taken = True
		succeeded = not taken

	return (row,col)	
	
######################NETWORKING FUNCTIONS########################

# Network listeners
def moveRobotRequested(req):
	"Handles requested move from controller. Returns True only if the move is valid. Executes action"
	
	# Read in values from request
	action = req.rightArmAction
	target = req.rightArmTarget

	leftAction = req.leftArmAction
	leftTarget = req.leftArmTarget
	
	targetType = target.targetType
	blockID = target.blockID

	gripperIsOpen = getGripperIsOpen()
	(blockFound, blockRow, blockCol, blockDepth) = getBlockInfo(blockID, targetType)

	global lastAction	#Previous Command
	global lastTarget	#Last Targetted blockID
	global holding		#Block currently held
	global lastHeld		#Last held block

	#Assume action cannot be done
	valid = False
	
	if targetType == -1:
		# Target is a block
		if action.type == action.OPEN_GRIPPER:
			valid = True
			updateGripperState(True)
			if holding != None:		# Prevents errors from unnecessary opening
				lastHeld = holding 	# Update last held block
			holding = None 			# No longer holding block

		elif action.type == action.CLOSE_GRIPPER:
			valid = True
			updateGripperState(False)
			if lastAction == action.OPEN_GRIPPER:
				holding = lastHeld
			if lastAction == action.MOVE_TO_BLOCK:
				holding = lastTarget

		elif action.type == action.MOVE_TO_BLOCK:
			top = (blockDepth == 1)
			gOpen = gripperIsOpen
			valid = (top and gOpen)
			if valid:
				lastAction = action.MOVE_TO_BLOCK
				lastTarget = blockID

		elif action.type == action.MOVE_OVER_BLOCK:
			top = (blockDepth == 1)
			notSelf = (holding != blockID)
			valid = (top and notSelf)		
			if valid:
				lastAction = action.MOVE_OVER_BLOCK
				if (holding != None):
					removeBlockFromWS(holding)
					addBlockToWS(holding, blockRow, blockCol)	
	
	else:
		if action.type == action.OPEN_GRIPPER:
			valid = True
			updateGripperState(True)
			if holding != None:		# Prevents errors from unnecessary opening
				lastHeld = holding 	# Update last held block
			holding = None 			# No longer holding block

		elif action.type == action.CLOSE_GRIPPER:
			valid = True
			updateGripperState(False)
			if lastAction == action.OPEN_GRIPPER:
				holding = lastHeld
			if lastAction == action.MOVE_TO_BLOCK:
				holding == lastTarget

		elif action.type == action.MOVE_TO_BLOCK:
			valid = False

		elif action.type == action.MOVE_OVER_BLOCK:
			valid = True
			lastAction = action.MOVE_OVER_BLOCK
			if (holding != None):
				removeBlockFromWS(holding)
				addBlockToWS(holding, blockRow, blockCol)			
	
	#Respond
	return valid
		
def getStateRequested(req):
	"Returns world state upon request"
	requestString = req.request
	return worldState

######################INIT FUNCTIONS########################

#Setup network
def initNetwork():
	"Initializes networking functionality. Returns all server and publisher info"
	# Setup service responses:
	moveRobotServer = rospy.Service('move_robot',MoveRobot,moveRobotRequested)
	getStateServer = rospy.Service('get_state',WorldState_Request,getStateRequested)
	
	# Publisher connection setup:
	worldStatePublisher = rospy.Publisher('world_state_connection',WorldState,queue_size = 10)
	publishRate = rospy.Rate(1) # Set publishing rate to 1Hz
	
	return (moveRobotServer,getStateServer,worldStatePublisher,publishRate)

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

# Full setup
def initRobotInterface(gridRows, gridCols, numBlocks, blockLocaleRow, blockLocaleCol, configuration, goalState, isOneArmSolution):
	"First function to call. Initializes robot_interface node."
	# Create node
	rospy.init_node('robot_interface')
	
	# Initialize world state
	initWorldState(gridRows,gridCols) 
	initBlocksInStack(configuration,numBlocks,blockLocaleRow,blockLocaleCol)
	
	#runTests() # TODO: remove

	# Initialize network
	(moveRobotServer,getStateServer,worldStatePublisher,publishRate) = initNetwork()
	
	# Continually network
	while not rospy.is_shutdown():
		rospy.loginfo(worldState)
		worldStatePublisher.publish(worldState)
		publishRate.sleep()

# INITIALIZE VIA MAIN
if __name__ == "__main__":
	try:
		ParamsBeingRead = 0
		#readParams(); ParamsBeingRead = 1
		if ParamsBeingRead == 0:
			gridRows = 5
			gridCols = 5
			numBlocks = 3
			blockLocaleRow = 3
			blockLocaleCol = 3
			configuration = "scattered"
			goalState = "stacked_descending"
			isOneArmSolution = False
		initRobotInterface(gridRows,gridCols,numBlocks,blockLocaleRow,blockLocaleCol,configuration,goalState,isOneArmSolution)
	except rospy.ROSInterruptException:
		pass

######################TESTING FUNCTIONS########################
def runTests():
	global worldState
	# Assuming initialized with: initBlocksInStack(True,3,0,0)
	updateGripperState(False)
	assert(getGripperIsOpen()==False)
	print worldState

	updateGripperState(True)
	assert(getGripperIsOpen()==True)
	print worldState
	
	stack = getStackInWS(0,0)
	assert(stack.blocks[0] == 1)
	assert(stack.blocks[1] == 2)
	assert(stack.blocks[2] == 3)
	
	stack = getStackInWS(5,5)
	assert(stack == None)
	
	(found,row,col,depth) = getBlockInfo(1)
	assert(found==True)
	assert(row==0)
	assert(col==0)
	assert(depth==3)
	
	(found,row,col,depth) = getBlockInfo(2)
	assert(found==True)
	assert(row==0)
	assert(col==0)
	assert(depth==2)
	
	(found,row,col,depth) = getBlockInfo(3)
	assert(found==True)
	assert(row==0)
	assert(col==0)
	assert(depth==1)
	
	removeBlockFromWS(3)
	(found,row,col,depth) = getBlockInfo(3)
	assert(found==False)
	
	addBlockToWS(3,1,1)
	(found,row,col,depth) = getBlockInfo(3)
	assert(found)
	assert(row==1)
	assert(col==1)
	assert(depth==1)
	
	moveBlockInWS(3,2,2)
	(found,row,col,depth) = getBlockInfo(3)
	assert(found)
	assert(row==2)
	assert(col==2)
	assert(depth==1)