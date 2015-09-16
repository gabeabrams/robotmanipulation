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
lastTarget = 3;

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
	

def initBlocksInStack(ascending,numBlocks,row,col):
	"Put a stack of numBlocks blocks in row,col"
	# Ascending    Descending
	#   [3]           [1]
	#   [2]           [2]
	#   [1]           [3]
	#"""""""""""""""""""""""""

	newStack = Stack()
	newStack.row = row
	newStack.col = col
	
	newStack.blocks = range(1,numBlocks+1)

	global lastTarget 
	lastTarget = numBlocks
	
	# Reverse the stack if descending
	if not ascending:
		newStack.blocks.reverse() 
		lastTarget = 1
		print lastTarget
		
	# Add stack to blocks
	worldState.grid.stacks.append(newStack)
	
	return newStack
	
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

def getBlockInfo(blockID):
	"Returns block info including depth: 1=top, len=bottom"
	found = False
	row = 0
	col = 0
	depth = 0

	for stack in worldState.grid.stacks:
		if blockID in stack.blocks:
			row = stack.row
			col = stack.col
			depth = len(stack.blocks) - stack.blocks.index(blockID)
			found = True

	if blockID == 0:
		(row,col) = getRandomTableLocation()
		depth = 1
		found = True	
	
	return (found,row,col,depth)

def getStackInWS(row,col):
	for stack in worldState.grid.stacks:
		if stack.row == row and stack.col == col:
			return stack
	return None

def getRandomTableLocation():
	"Returns row and column of random open table location"
	succeeded = False
	taken = False
	row = 0
	col = 0
	
	while not succeeded:
		row = random.randint(0, worldState.grid.dimensions.row)
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
	action = req.action
	target = req.target
	
	isblock = target.isblock
	blockID = target.block
	row = target.loc.row
	col = target.loc.col
	
	gripperIsOpen = getGripperIsOpen()
	(blockFound, blockRow, blockCol, blockDepth) = getBlockInfo(blockID)
	global lastAction	#Previous Command
	global lastTarget	#Last Targetted blockID
	global holding		#Block currently held
	global lastHeld		#Last held block

	#Assume action cannot be done
	valid = False
	
	if isblock:
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
	
	elif isblock and blockID == 0:
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
	
	else:
		# Row and column specified
		valid = True # TODO
	
	#Respond
	return valid # TODO	
		
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
	global isAscending
	global goalState
	global isOneArmSolution

	gridRows = rospy.get_param("gridRows")
	gridCols = rospy.get_param("gridCols")
	numBlocks = rospy.get_param("numBlocks")
	blockLocaleRow = rospy.get_param("blockLocaleRow")
	blockLocaleCol = rospy.get_param("blockLocaleCol")
	isAscending = rospy.get_param("isAscending")
	goalState = rospy.get_param("goalState")
	isOneArmSolution = rospy.get_param("isOneArmSolution")	

# Full setup
def initRobotInterface(gridRows, gridCols, numBlocks, blockLocaleRow, blockLocaleCol, isAscending, goalState, isOneArmSolution):
	"First function to call. Initializes robot_interface node."
	# Create node
	rospy.init_node('robot_interface')
	
	# Initialize world state
	initWorldState(gridRows,gridCols) 
	initBlocksInStack(isAscending,numBlocks,blockLocaleRow,blockLocaleCol)
	
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
		readParams(); ParamsBeingRead = 1
		if ParamsBeingRead == 0:
			gridRows = 5
			gridCols = 5
			numBlocks = 3
			blockLocaleRow = 0
			blockLocaleCol = 0
			isAscending = True
			goalState = "descending"
			isOneArmSolution = False
		initRobotInterface(gridRows,gridCols,numBlocks,blockLocaleRow,blockLocaleCol,isAscending,goalState,isOneArmSolution)
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

######################OLD CODE########################

#Answers move_robot requests
def move_robot(req):
	print "Checking validity" # TODO
	return true

#Answers get_state requests
def get_state(req):
	print "Retrieving state" # TODO
	return "THIS SHOULD BE THE STATE"	

#Maintains state and publishes
def robot_interface():
	rospy.init_node('robot_interface')

	#Pass to move_robot
	s1 = rospy.Service('move_robot', MoveRobot, move_robot)
	print "Ready to move robot"

	#Pass to get_state
	s2 = rospy.Service('get_state', WorldState_Request, get_state)
	print "Ready to get state"

	#Publisher
	pub = rospy.Publisher('State', WorldState, queue_size = 10)
	rate = rospy.Rate(1)
	#WorldState setup
	msg = WorldState()
	msg.grid = Grid()
	msg.grid.stacks = [Stack() for _ in range(2)]
	msg.grid.stacks[0].blocks = [Block() for _ in range(1)]
	msg.grid.stacks[0].blocks[0].id = 1
	msg.grid.stacks[0].row = 2
	msg.grid.stacks[0].col = 2
	msg.grid.stacks[1].blocks = [Block() for _ in range(1)]
	msg.grid.stacks[1].blocks[0].id = 3
	msg.grid.stacks[1].row = 1
	msg.grid.stacks[1].col = 0
	msg.grid.dimensions = Coord()
	msg.grid.dimensions.row = 5
	msg.grid.dimensions.col = 5
	while not rospy.is_shutdown():
		rospy.loginfo(msg)
		pub.publish(msg)
		rate.sleep()


	
