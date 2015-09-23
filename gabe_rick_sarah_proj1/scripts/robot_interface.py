#!/usr/bin/env python

import rospy
import random
from gabe_ricky_sarah_proj1.srv import*
from gabe_ricky_sarah_proj1.msg import*
from baxter_interface import Gripper
from baxter_interface import Limb

#########################GLOBAL DATA###########################

#World State
worldState = WorldState()

#Save the last action performed
rightLastAction = 5; #1st action was action.MOVE_TO_BLOCK
rightHolding = None;
rightLastHeld = None;
rightLastTarget = 0;
rightLastOver = -3;

leftLastAction = None;
leftHolding = None;
leftLastHeld = None;
leftLastTarget = None;
leftLastOver = 0;

home = [0,0];

ClosedPercent = 50;
BLOCK_SIDE = 1.75*.00254
#######################WORLD STATE FUNCTIONS########################

def getWorldState():
	"Get the world state object"
	return worldState
	
def updateGripperState(isOpen, side):
	"Change the gripper state in worldState"
	global worldState
	if side == 'right':
		worldState.rightGripperOpen = isOpen
	if side == 'left':
		worldState.leftGripperOpen = isOpen

def getGripperIsOpen(side):
	global worldState
	if side == 'right':
		return worldState.rightGripperOpen
	if side == 'left':
		return worldState.leftGripperOpen
	else:
		return True	
	
def initWorldState(rows,cols):
	"Initialize the world state with an empty grid"
	global worldState
	worldState.grid = Grid()
	worldState.grid.stacks = []
	
	worldState.grid.dimensions = Coord()
	worldState.grid.dimensions.row = rows
	worldState.grid.dimensions.col = cols
	
	worldState.rightGripperOpen = True
	worldState.leftGripperOpen = True
	

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

	global rightLastTarget 
	
	if configuration == "stacked_ascending":
		newStack.blocks = range(1,numBlocks+1)
		rightLastTarget = numBlocks
		worldState.grid.stacks.append(newStack)

	# Reverse the stack if descending
	if configuration == "stacked_descending":
		newStack.blocks = range(1,numBlocks+1)
		newStack.blocks.reverse() 
		rightLastTarget = 1
		worldState.grid.stacks.append(newStack)

	if configuration == "scattered":
		for i in range (1,numBlocks+1):
			(row, col) = getRandomTableLocation(-3)
			addBlockToWS(i, row, col)
		rightLastTarget = numBlocks	
			
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
	height = 0

	if blockID > 0:
		for stack in worldState.grid.stacks:
			if blockID in stack.blocks:
				row = stack.row
				col = stack.col
				depth = len(stack.blocks) - stack.blocks.index(blockID)
				height = stack.blocks.index(blockID)
				found = True

	elif blockID == 0:	
		(row,col) = home
		depth = 1
		for stack in worldState.grid.stacks:
			if stack.row == row and stack.col == col:
				newBlock = stack.blocks[-1]
				(found, row, col, depth, height) = getBlockInfo(newBlock) 
		found = True		

	else:
		(row,col) = getRandomTableLocation(blockID)
		depth = 1
		height = 0
		found = True	

	return (found,row,col,depth)

def getStackInWS(row,col):
	for stack in worldState.grid.stacks:
		if stack.row == row and stack.col == col:
			return stack
	return None

def getHomeLoc():
	global blockLocaleCol
	global blockLocaleRow
	return (blockLocaleRow, blockLocaleCol)	

def getNumBlocks():
	global numBlocks
	return numBlocks	

def getRandomTableLocation(blockID):
	"Returns row and column of random open table location"
	succeeded = False
	taken = False
	row = 0
	col = 0
	global home
	homerow = home[0]
	homecol = home[1]
	
	while not succeeded:
		if blockID == -3:
			row = random.randint(0, worldState.grid.dimensions.row)
			col = random.randint(0, worldState.grid.dimensions.col)
		elif blockID == -1:	
			row = random.randint(0, homerow)
			col = random.randint(0, worldState.grid.dimensions.col)
		elif blockID == -2:	
			row = random.randint(homerow, worldState.grid.dimensions.row)
			col = random.randint(0, worldState.grid.dimensions.col)
		for stack in worldState.grid.stacks:
			if row == stack.row and col == stack.col:
				taken = True
			if row == homerow and col == homecol:
				taken = True	
		succeeded = not taken

	return (row,col)	

######################MOVING FUNCTIONS########################
def Mover(action, target, arm):

	blockID = target.blockID

	gripperIsOpen = getGripperIsOpen(arm)
	(blockFound, blockRow, blockCol, blockDepth, blockHeight) = getBlockInfo(blockID)

	global rightLastTarget
	global rightLastAction
	global rightHolding
	global rightLastHeld
	global rightLastOver

	global leftLastTarget
	global leftLastAction
	global leftHolding
	global leftLastHeld
	global leftLastOver	

	if arm == 'right':
		holding = rightHolding
		lastHeld = rightLastHeld
		lastAction = rightLastAction
		lastTarget = rightLastTarget
		lastOver = rightLastOver

	elif arm == 'left':
		holding = leftHolding
		lastHeld = leftLastHeld
		lastAction = leftLastAction
		lastTarget = leftLastTarget
		lastOver = leftLastOver

	#Assume action cannot be done
	if blockID > target.TABLE_HOME:
		# Target is a block
		if action.type == action.OPEN_GRIPPER:
			updateGripperState(True, arm)
			if holding != None:				# Prevents errors from unnecessary opening
				lastHeld = holding 			# Update last held block
			holding = None 	
			baxterOpen(arm)					# No longer holding block	

		elif action.type == action.CLOSE_GRIPPER:
			updateGripperState(False, arm)
			if lastAction == action.OPEN_GRIPPER:
				holding = lastHeld
			if lastAction == action.MOVE_TO_BLOCK:
				holding = lastTarget
			lastAction = action.CLOSE_GRIPPER
			baxterClose(arm)	

		elif action.type == action.MOVE_TO_BLOCK:
			lastAction = action.MOVE_TO_BLOCK
			lastTarget = blockID
			baxterMoveTo(arm)

		elif action.type == action.MOVE_OVER_BLOCK:
			lastAction = action.MOVE_OVER_BLOCK
			if (holding != None):
				removeBlockFromWS(holding)
				addBlockToWS(holding, blockRow, blockCol)
			lastOver = blockID	
			baxterMoveOver(arm, blockRow, blockCol, blockDepth)	
	
	else:
		if action.type == action.MOVE_OVER_BLOCK:
			lastAction = action.MOVE_OVER_BLOCK
			if (holding != None):
				removeBlockFromWS(holding)
				addBlockToWS(holding, blockRow, blockCol)
			lastOver = blockID	
			baxterMoveOver(arm, blockRow, blockCol, blockDepth)			
	
	if arm == 'right':
		rightHolding = holding
		rightLastHeld = lastHeld
		rightLastAction = lastAction
		rightLastTarget = lastTarget
		rightLastOver = lastOver

	if arm == 'left':
		leftHolding = holding
		leftLastHeld = lastHeld
		leftLastAction = lastAction
		leftLastTarget = lastTarget
		leftLastOver = lastOver	

def baxterOpen(arm):
	if arm == 'right':
		rightGripper.open(block = False, timeout = 5)
	elif arm == 'left':
		leftGripper.open(block = False, timeout = 5)		

def baxterClose(arm):
	global ClosedPercent
	if arm == 'right':
		rightGripper.command_position(ClosedPercent, block = False, timeout = 5)
	elif arm == 'left':
		leftGripper.command_position(ClosedPercent, block = False, timeout = 5)

def baxterMoveTo(arm):
	global BLOCK_SIDE
	if arm == 'right':
		currentPose = rightLimb.endpoint_pose()
		currentx = currentPose.position.x
		currenty = currentPose.position.y
		currentz = currentPose.position.z
		currento = currentPose.orientation
		new z = currentz + BLOCK_SIDE   #TODO: is this the right direction??????

		Request = pose_stamp()
		Request.header = std_msgs/Header()
		Request.header.seq = 0
		Request.header.stamp = 
		try:
			#IK = rospy.ServiceProxy("get_state",WorldState_Request)
			StateResponse = WorldStateRequest('Whirled steight pls')
			worldState = StateResponse.state
		except rospy.ServiceException, e:
			return None

def baxterMoveOver(arm, blockRow, blockCol, blockDepth):
	print 1			

######################CHECKING FUNCTIONS########################
def oneArmChecker(action, target, arm):

	blockID = target.blockID

	gripperIsOpen = getGripperIsOpen(arm)
	(blockFound, blockRow, blockCol, blockDepth, blockHeight) = getBlockInfo(blockID)

	global rightLastTarget
	global rightLastAction
	global rightHolding
	global rightLastHeld
	global rightLastOver

	global leftLastTarget
	global leftLastAction
	global leftHolding
	global leftLastHeld
	global leftLastOver

	if arm == 'right':
		holding = rightHolding
		lastHeld = rightLastHeld
		lastAction = rightLastAction
		lastTarget = rightLastTarget

	elif arm == 'left':
		holding = leftHolding
		lastHeld = leftLastHeld
		lastAction = leftLastAction
		lastTarget = leftLastTarget	

	#Assume action cannot be done
	valid = False

	if action.type == action.STILL:
		valid = True
		return valid
	
	if blockID > target.TABLE_HOME:
		# Target is a block
		if action.type == action.OPEN_GRIPPER:
			valid = True

		elif action.type == action.CLOSE_GRIPPER:
			valid = True

		elif action.type == action.MOVE_TO_BLOCK:
			top = (blockDepth == 1)
			gOpen = gripperIsOpen
			alreadyOver = (lastAction == action.MOVE_OVER_BLOCK)
			valid = (top and gOpen and alreadyOver)

		elif action.type == action.MOVE_OVER_BLOCK:
			top = (blockDepth == 1)
			notSelf = (rightHolding != blockID)
			valid = (top and notSelf)			
	
	else:
		if action.type == action.MOVE_OVER_BLOCK:
			valid = True			

	return valid

def twoArmChecker(rightAction, rightTarget, leftAction, leftTarget):

	rightBlockID = rightTarget.blockID
	leftBlockID = leftTarget.blockID

	rightGripperIsOpen = getGripperIsOpen('right')
	leftGripperIsOpen = getGripperIsOpen('left')

	(rightBlockFound, rightBlockRow, rightBlockCol, rightblockDepth, rightBlockHeight) = getBlockInfo(rightBlockID)
	(leftBlockFound, leftBlockRow, leftBlockCol, leftBlockDepth, leftBlockHeight) = getBlockInfo(leftBlockID)	

	global rightLastAction
	global rightLastTarget
	global rightLastHeld
	global rightHolding

	global leftLastAction
	global leftLastTarget
	global rightLastHeld
	global rightHolding

	valid = False

	rightValid = oneArmChecker(rightAction, rightTarget, 'right')
	leftValid = oneArmChecker(leftAction, leftTarget, 'left')
	if (rightValid == False) or (leftValid == False):
		#One of the actions is invalid on its own
		valid = False
		return valid

	if (rightAction.type == rightAction.STILL) or (leftAction.type == leftAction.STILL):
		valid = True
		return valid	

	if (rightAction.type == rightAction.OPEN_GRIPPER) or \
	(rightAction.type == rightAction.CLOSE_GRIPPER) or \
	(rightAction.type == rightAction.MOVE_TO_BLOCK):
		
		if leftAction.type == leftAction.OPEN_GRIPPER:
			valid = True

		if leftAction.type == leftAction.CLOSE_GRIPPER:
			valid = True

		if leftAction.type == leftAction.MOVE_TO_BLOCK:
			valid = True

		if leftAction.type == leftAction.MOVE_OVER_BLOCK:
			if (leftBlockID != rightLastOver) or (leftBlockID < leftTarget.TABLE_HOME):
				#Left is not moving to where right is
				valid = True

		
	if rightAction.type == rightAction.MOVE_OVER_BLOCK:

		if leftAction.type == leftAction.OPEN_GRIPPER:
			if (rightBlockID != leftLastOver) or (rightBlockID < rightTarget.TABLE_HOME):
				#Right is not moving to where left is
				valid = True

		if leftAction.type == leftAction.CLOSE_GRIPPER:
			if (rightBlockID != leftLastOver) or (rightBlockID < rightTarget.TABLE_HOME):
				#Right is not moving to where left is
				valid = True

		if leftAction.type == leftAction.MOVE_TO_BLOCK:
			if (rightBlockID != leftLastOver) or (rightBlockID < rightTarget.TABLE_HOME):
				#Right is not moving to where left is
				valid = True

		if leftAction.type == leftAction.MOVE_OVER_BLOCK:
			if (rightBlockID != leftLastOver) or (rightBlockID < rightTarget.TABLE_HOME):
				#Right is not moving to where left is
				if (leftBlockID != rightBlockID) or (rightBlockID < rightTarget.TABLE_HOME):
				#Left is not moving to where right will be post-move
					valid = True

	return valid	


######################NETWORKING FUNCTIONS########################

# Network listeners
def moveRobotRequested(req):
	"Handles requested move from controller. Returns True only if the move is valid. Executes action"
	
	if isOneArmSolution:
		valid = oneArmChecker(req.rightArmAction, req.rightArmTarget, 'right')
		if valid:
			Mover(req.rightArmAction, req.rightArmTarget, 'right')

	else:
		valid = twoArmChecker(req.rightArmAction, req.rightArmTarget, req.leftArmAction, req.leftArmTarget)	
		if valid:
			Mover(req.rightArmAction, req.rightArmTarget, 'right')
			Mover(req.leftArmAction, req.leftArmTarget, 'left')

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

def initBaxterObjects():
	global rightGripper
	global leftGripper
	global rightLimb
	global leftLimb

	rightGripper = Gripper('right', versioned = False)
	leftGripper = Gripper('left', versioned = False)
	rightMover = Limb('right')
	leftMover = Limb('left')

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

	#initBaxterObjects()
	
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
	
	(found,row,col,depth, height) = getBlockInfo(1)
	assert(found==True)
	assert(row==0)
	assert(col==0)
	assert(depth==3)
	
	(found,row,col,depth, height) = getBlockInfo(2)
	assert(found==True)
	assert(row==0)
	assert(col==0)
	assert(depth==2)
	
	(found,row,col,depth, height) = getBlockInfo(3)
	assert(found==True)
	assert(row==0)
	assert(col==0)
	assert(depth==1)
	
	removeBlockFromWS(3)
	(found,row,col,depth, height) = getBlockInfo(3)
	assert(found==False)
	
	addBlockToWS(3,1,1)
	(found,row,col,depth, height) = getBlockInfo(3)
	assert(found)
	assert(row==1)
	assert(col==1)
	assert(depth==1)
	
	moveBlockInWS(3,2,2)
	(found,row,col,depth, height) = getBlockInfo(3)
	assert(found)
	assert(row==2)
	assert(col==2)
	assert(depth==1)
