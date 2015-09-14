#!/usr/bin/env python

import rospy
from gabe_ricky_sarah_proj1.srv import*
from gabe_ricky_sarah_proj1.msg import*

#########################GLOBAL DATA###########################

#World State
worldState = WorldState() # TODO: add position of end effector
# State: [bottom, ..., top]

#######################WORLD STATE FUNCTIONS########################

def getWorldState():
	"Get the world state object"
	return worldState
	
def updateGripperState(isOpen):
	"Change the gripper state in worldState"
	worldState.gripperOpen = isOpen

def getGripperState():
	return worldState.gripperOpen
	
def initWorldState(rows,cols):
	"Initialize the world state with an empty grid"
	worldState.grid = Grid()
	worldState.grid.stacks = []
	
	worldState.grid.dimensions = Coord()
	worldState.grid.dimensions.row = rows
	worldState.grid.dimensions.col = cols
	
	worldState.gripperOpen = True
	
	return worldState

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
	
	# Reverse the stack if descending
	if not ascending:
		newStack.blocks.reverse()
		
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
	
	return (found,row,col,depth)

def getStackInWS(row,col):
	for stack in worldState.grid.stacks:
		if stack.row == row and stack.col == col:
			return stack
	return None

def getRandomTableLocation(blockID):
	"Returns row and column of random open table location"
	succeeded = False
	row = 0
	col = 0
	
	while not succeeded:
		row = randrange(worldState.grid.dimensions.rows)
		col = randrange(worldState.grid.dimensions.cols)
	
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
	
	
	# Calculate final location
	if isblock and blockID > 0:
		# Target is a block
		return True # TODO
	
	elif isblock and blockID == 0:
		# Random location on the table
		return True # TODO
	
	else:
		# Row and column specified
		return True # TODO
	
	# TODO: CHANGE WORLD STATE + BAXTER (later)
	
	succeeded = True # TODO
		
	# Handle action cases
	if action.type == action.OPEN_GRIPPER:
		succeeded = False
	
	if action.type == action.CLOSE_GRIPPER:
		succeeded = False
		
	if action.type == action.MOVE_TO_BLOCK:
		succeeded = False
		
	if action.type == action.MOVE_OVER_BLOCK:
		succeeded = False
	
	
	# Prepare response
	return succeeded
		

def getStateRequested(req):
	"Returns world state upon request"
	requestString = req.request
	return worldState
	
######################TESTING FUNCTIONS########################
def runTests():
	# Assuming initialized with: initBlocksInStack(True,3,0,0)
	updateGripperState(False)
	assert(getGripperState()==False)
	
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
def initRobotInterface():
	"First function to call. Initializes robot_interface node."
	# Create node
	rospy.init_node('robot_interface')

	# Get parameters
	readParams()
	
	# Initialize world state
	initWorldState(gridRows,gridCols) 
	initBlocksInStack(isAscending,numBlocks,blockLocaleRow,blockLocaleCol)
	
	runTests() # TODO: remove
	
	# Initialize network
	(moveRobotServer,getStateServer,worldStatePublisher,publishRate) = initNetwork()
	
	# Continually network
	while not rospy.is_shutdown():
		rospy.loginfo(worldState)
		worldStatePublisher.publish(worldState)
		publishRate.sleep()

# INITIALIZE VIA MAIN
if __name__ == "__main__":
	initRobotInterface()	
	

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


	
