#!/usr/bin/env python

BLOCK_TARGET = 1 # Any positive number = blockID
TABLE_TARGET = -3
TABLE_LEFT_TARGET = -1
TABLE_RIGHT_TARGET = -2
HOME_TARGET = 0
GRIPPER_TARGET = -4
NO_TARGET = -5

OPEN_OPERATION = 1
CLOSE_OPERATION = 2
MOVE_OVER_OPERATION = 3
MOVE_TO_OPERATION = 4
NO_OPERATION = 5
MOVE_OUT_OPERATION = 6

homeRow = 0
homeCol = 0

import rospy
from gabe_ricky_sarah_proj1.srv import*
from gabe_ricky_sarah_proj1.msg import*
from std_msgs.msg import String
import controller
##################################ACTION HANDLER########################################

# SIMPLE ACTIONS
def openGripper():
	return createDormantAction(GRIPPER_TARGET,OPEN_OPERATION, "open gripper       ")

def closeGripper():
	return createDormantAction(GRIPPER_TARGET,CLOSE_OPERATION, "close gripper      ")

def moveToBlock(blockID):
	return createDormantAction(blockID, MOVE_TO_OPERATION, "move to block " + str(blockID) + "    ")

def moveOverBlock(blockID):
	return createDormantAction(blockID, MOVE_OVER_OPERATION, "move over block " + str(blockID) + "  ")

def moveToHome():
	return createDormantAction(HOME_TARGET, MOVE_OVER_OPERATION, "move over home       ")

def moveAboveTable(area):
	return createDormantAction(area*-1, MOVE_OVER_OPERATION, "move over table a " + str(area))

def moveOut(side):
	return createDormantAction(side, MOVE_OUT_OPERATION, "move out of the way   ")

def still():
	return createDormantAction(NO_TARGET, NO_OPERATION, "still              ")

def createDormantAction(target, operation):
	createDormantAction(target,operation,"")
	
def createDormantAction(target, operation, text):
	act = Action()
	tar = Target()
	if operation == OPEN_OPERATION: # Open
		act.type = 2
		tar.blockID = 1
	if operation == CLOSE_OPERATION: # Close
		act.type = 3
		tar.blockID = 1
	if operation == MOVE_OVER_OPERATION: # Move Over
		act.type = 7
		tar.blockID = target
	if operation == MOVE_TO_OPERATION: # Move To
		act.type = 5
		tar.blockID = target
	if operation == NO_OPERATION: # Still
		act.type = 11
		tar.blockID = 1
	if operation == MOVE_OUT_OPERATION:
		act.type = 7
		tar.blockID = target	

	return (act, tar, text)

##################################MOVE PLANNER########################################

# COMPOSITE ACTIONS
# area: 1=left, 2=right, 0=either

def padGeneral(rightActions,leftActions):
	rPlus = [] 
	lPlus = []
	while((len(leftActions)+len(lPlus)) > (len(rPlus) + len(rightActions))):
		rPlus += [still()]
	while((len(rPlus) + len(rightActions)) > (len(lPlus) + len(leftActions))):
		lPlus += [still()]
	return (rPlus,lPlus)

# During this phase, tuple order is: (CENTER, OUTSKIRTS)
# 3 THEN 3
def padScatterBlock():
	return [still(),still(),still(),still()]
	
def scatterBlock(blockID,area):
	return [still(),moveOverBlock(blockID),moveToBlock(blockID),closeGripper(),     moveAboveTable(area),openGripper(), still(),still()]
	
def scatterOnto(blockID,blockIDBelow):
	return [still(),moveOverBlock(blockID),moveToBlock(blockID),closeGripper(),     moveOverBlock(blockIDBelow),openGripper(),still(),still()]

# During this phase, tuple order is: (OUTSKIRTS, CENTER)
# 4 THEN 4
def padStackBlock():
	return [still(),still(),still()]	

def stackBlock(moveID,destID):
	if destID == None:
		centerAction = moveToHome()
	else:
		centerAction = moveOverBlock(destID)
	return [moveOverBlock(moveID),moveToBlock(moveID),closeGripper(),     still(),centerAction,openGripper()]
	

# FULL ROUTINES
SCATTERED = 3
STACKED_ASCENDING = 1
STACKED_DESCENDING = 2
SORTED_ODD_EVEN = 4
MESSED_UP = 5
def translateStateInfo(param):
	endOut = STACKED_ASCENDING
	if param == "scattered":
		endOut = SCATTERED
	if param == "stacked_descending":
		endOut = STACKED_DESCENDING
	if param == "sorted_odd_even":
		endOut = SORTED_ODD_EVEN
	return endOut
	
def oneArm(startI,endI,numBlocks):
	
	# If already done, return
	if startI == endI:
		return []

	rightActions = []
	rightActions += [openGripper()]
	
	# Scatter blocks
	if not startI == SCATTERED:
		scatterOrder = range(1,numBlocks+1)
		if(startI == STACKED_ASCENDING):
			scatterOrder.reverse()
	
		rightActions = []
		rightActions += [(openGripper())]	
		for blockID in scatterOrder:
			rightActions += scatterBlock(blockID,3)
			
	# If final state is scatter, we're done!
	if endI == SCATTERED:
		return rightActions
	
	# Re-stack blocks
	stackOrder = range(1,numBlocks+1)
	if(endI == STACKED_DESCENDING):
		stackOrder.reverse()
	
	prevID = None
	for blockID in stackOrder:
		rightActions += stackBlock(blockID,prevID)
		prevID = blockID
	
	return (rightActions)

def twoArms(startI,endI,numBlocks):
	
	# If already done, return
	if startI == endI:
		return ([],[])
	
	# Scatter blocks
	scatterOrder = range(1,numBlocks+1)
	if(startI == STACKED_ASCENDING):
		scatterOrder.reverse()
	if (startI == SCATTERED):
		scatterOrder = []	
	
	# Prepare for synchronization
	if numBlocks%2 == 0:
		leftActions = []
		rightActions = padScatterBlock()
	else:	
		rightActions = []
		leftActions = padScatterBlock()

	rightActions += [openGripper()]
	leftActions += [openGripper()]
	
	# Odd/Even Sorting
	leftBottomID = -1
	rightBottomID = -1
	
	for blockID in scatterOrder:
		if len(rightActions) < len(leftActions):
			# Moving blockID to right side
			if endI == SORTED_ODD_EVEN:
				if rightBottomID == -1:
					rightBottomID = blockID
					rightActions += scatterBlock(blockID,2)
				else:
					rightActions += scatterOnto(blockID,rightBottomID)
					rightBottomID = blockID
			else:
				rightActions += scatterBlock(blockID,2)
			
		else:
			# Moving blockID to left side
			if endI == SORTED_ODD_EVEN:
				if leftBottomID == -1:
					leftBottomID = blockID
					leftActions += scatterBlock(blockID,1)
				else:
					leftActions += scatterOnto(blockID,leftBottomID)
					leftBottomID = blockID
			else:
				leftActions += scatterBlock(blockID,1)
	
	# Synchronize
	if len(rightActions) > len(leftActions):
		leftActions = leftActions + padScatterBlock()
	else:
		rightActions = rightActions + padScatterBlock()
	
	# If final state is scatter, we're done!
	if endI > 2:
		return (rightActions,leftActions)
		
	# Stagger
	leftActions += padStackBlock() # right goes first
	
	# Re-stack blocks
	stackOrder = range(1,numBlocks+1)
	if(endI == STACKED_DESCENDING):
		stackOrder.reverse()
	
	prevID = None
	for blockID in stackOrder:
		if len(rightActions) < len(leftActions):
			rightActions += stackBlock(blockID,prevID)
		else:
			leftActions += stackBlock(blockID,prevID)
		prevID = blockID
	
	# Synchronize arms
	if len(rightActions) < len(leftActions):
		rightActions += [moveOut(-4)]
		rightActions += padStackBlock()
	else:
		leftActions += [moveOut(-5)]
		leftActions += padStackBlock()
			
	return (rightActions,leftActions)

# FALLBACK

# Uses one arm to finish operation
def fallback(worldState, numArmsToUse):
	print "Fallback activated."
	
	# Open first
	rightActions = []
	rightActions += [(openGripper())]

	global homeRow
	global homeCol	

	# Find all stacks
	stacks = worldState.grid.stacks
	tallStacks = []
	shortStacks = []
	for stack in stacks:
		if len(stack.blocks) > 1:
			tallStacks.append(stack)
		else:
			shortStacks.append(stack)
	
	# Now we have all stacks that need to be deconstructed
	# Scatter these
	for stack in tallStacks:
		blockList = list(stack.blocks)
		blockListReversed = blockList.reverse()
		blocks = tuple(blockList)
		for blockID in blocks:
			if (blockID%2) == 0:
				rightActions += scatterBlock(blockID,2)
			else:
				rightActions += scatterBlock(blockID,1)	

	for stack in shortStacks:
		blockID = stack.blocks[0]
		if blockID%2 == 0:
			rightActions += scatterBlock(blockID, 1)
		else:
			rightActions += scatterBlock(blockID, 2)				
	
	return (rightActions)
	
#################################TESTING#########################################

#(l,r) = twoArms("stacked_ascending", "sorted_odd_even",5)
#for i in range(len(l)):
#	(_,_,left) = l[i]
#	(_,_,right) = r[i]
#	print(left + "\t" + right)
	
##################################AI EASY########################################
def sendHomeLoc(blockLocaleRow, blockLocaleCol):
	global homeRow
	global homeCol
	homeRow = blockLocaleRow
	homeCol = blockLocaleCol

def detectConfig(worldState):
	# Get stack height
	maxStackHeight = 1
	global homeRow
	global homeCol
	clogging = False

	for stack in worldState.grid.stacks:
		if maxStackHeight < len(stack.blocks):
			maxStackHeight = len(stack.blocks)
		if stack.row == homeRow and stack.col == homeCol:
			clogging = True
	
	# Get number of blocks
	numBlocks = 0
	for stack in worldState.grid.stacks:
		for blockID in stack.blocks:
			numBlocks += 1
	
	# Get number of stacks
	numStacks = len(worldState.grid.stacks)
	
	# Is first stack ascending?
	isAscending = (worldState.grid.stacks[0].blocks == tuple(range(1,numBlocks+1)))
	desc = range(1,numBlocks+1)
	desc.reverse()
	isDescending = (worldState.grid.stacks[0].blocks == tuple(desc))

	if maxStackHeight == 1 and clogging == False:
		return (SCATTERED,numBlocks)
	elif numStacks == 1 and isAscending:
		return (STACKED_ASCENDING,numBlocks)
	if numStacks == 1 and isDescending:
		return (STACKED_DESCENDING,numBlocks)
	else:
		return (MESSED_UP,numBlocks)

def runActions(dormantActionRight, dormantActionLeft):
	print "Right arm action: " 
	print dormantActionRight
	print "Left arm action:  " 
	print dormantActionLeft
	(action1,target1,text1) = dormantActionRight
	(action2,target2,text2) = dormantActionLeft
   	rospy.wait_for_service('move_robot', timeout = 2)
   	try:
   		move_robot_handle = rospy.ServiceProxy('move_robot', MoveRobot)
   		data = move_robot_handle(action1, action2, target1, target2)
   		print data
   		print ""
   		return data.success
   	except rospy.ServiceException, e:
   		print "Service call failed: %s"%e

##### USE THESE FUNCTIONS ONLY #####

# Usage: pass in world state, goal string from params, number of arms
# ex: actionPackage = heyAIWhatsNext(worldState,"scattered",2)
def heyAIWhatsNext(worldState, goalStateString, numArmsToUse):
	(currentState, numBlocks) = detectConfig(worldState)
	goalState = translateStateInfo(goalStateString)
	
	(rightActions,leftActions) = ([],[])
	
	# RECOVER WITH FALLBACK IF NEEDED
	if currentState == MESSED_UP:
		(rPlus) = fallback(worldState, numArmsToUse)
		rightActions += rPlus
		leftActions += [openGripper()]
		leftActions += [moveOut(-5)]
		currentState = SCATTERED
	
	# PAD TO START FRESH
	(rPlus, lPlus) = padGeneral(rightActions,leftActions)
	rightActions += rPlus
	leftActions += lPlus

	if numArmsToUse == 1:
		(rPlus) = oneArm(currentState,goalState,numBlocks)
		rightActions += rPlus
	
	if numArmsToUse == 2:
		(rPlus,lPlus) = twoArms(currentState,goalState,numBlocks)
		leftActions += lPlus
		rightActions += rPlus
	
	# DONE. RETURN ACTIONPsACKAGE
	(rPlus,lPlus) = padGeneral(rightActions,leftActions)
	leftActions += lPlus
	rightActions += rPlus

	return ((rightActions,0),(leftActions,0))

# Usage: pass in actionPackage to perform next action
# ex: actionPackage,response = heyAIDoNext(actionPackage)
def heyAIDoNext(actionPackage, numArmsToUse):
	if numArmsToUse == 2:
		((rightActions,r),(leftActions,l)) = actionPackage
		
		leftAction = leftActions[l]
		rightAction = rightActions[r]
		dataBack = runActions(rightAction,leftAction)
		
		l += 1
		r += 1
		return (((rightActions,r),(leftActions,l)),dataBack)

	else:
		((rightActions,r), (leftActions,l)) = actionPackage
		rightAction = rightActions[r]
		dataBack = runActions(rightAction, still())
		
		r += 1
		
		return (r,dataBack)	

# Usage: call this function to alert AI that the previous action needs to be retried
# Still need to call heyAIDoNext
# ex: actionPackage = heyAIThatFailed(actionPackage)
def heyAIThatFailed(actionPackage):
	((rightActions,r),(leftActions,l)) = actionPackage
	l -= 1
	r -= 1
	return (((rightActions,r),(leftActions,l)), False)
