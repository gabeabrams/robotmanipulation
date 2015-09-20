#!/usr/bin/env python

#import action_handler

# COMPOSITE ACTIONS
# area: 1=left, 2=right, 0=either

# During this phase, tuple order is: (CENTER, OUTSKIRTS)
# 3 THEN 3
def padScatterBlock():
	return [still(),still(),still(),still()]
	
def scatterBlock(blockID,area):
	return [still(),moveOverBlock(blockID),moveToBlock(blockID),closeGripper(),     moveAboveTable(area),openGripper(),still(),still()]
	
def scatterOnto(blockID,blockIDBelow):
	return [still(),moveOverBlock(blockID),moveToBlock(blockID),closeGripper(),     moveOverBlock(blockIDBelow),openGripper(),still(),still()]

# During this phase, tuple order is: (OUTSKIRTS, CENTER)
# 4 THEN 4
def padStackBlock():
	return [still(),still(),still()]

def stackBlock(moveID,destID):
	if destID == None:
		centerAction = moveToCenterStackLocation()
	else:
		centerAction = moveOverBlock(destID)
	return [moveOverBlock(moveID),moveToBlock(moveID),closeGripper(),     still(),centerAction,openGripper()]
	

# FULL ROUTINES
def translateStateInfo(startIn, endIn):
	startOut = 1
	if startIn == "scattered":
		startOut = 3
	if startIn == "stacked_descending":
		startOut = 2
		
	endOut = 1
	if endIn == "scattered":
		endOut = 3
	if endIn == "stacked_descending":
		endOut = 2
	if endIn == "sorted_odd_even":
		endOut = 4
	return (startOut,endOut)
	
def oneArm(startState,endState,numBlocks):
	startI,endI = translateStateInfo(startState,endState) # 1=ascending, 2=descending
	
	# If already done, return
	if startI == endI:
		return []
	
	# Scatter blocks
	if not startI == 3:
		scatterOrder = range(1,numBlocks+1)
		if(startI == 1):
			scatterOrder.reverse()
	
		rightActions = []
		for blockID in scatterOrder:
			rightActions += scatterBlock(blockID,0)
			
	# If final state is scatter, we're done!
	if endI == 3:
		return rightActions
	
	# Re-stack blocks
	stackOrder = range(1,numBlocks+1)
	if(endI == 2):
		stackOrder.reverse()
	
	prevID = None
	for blockID in scatterOrder:
		rightActions += stackBlock(blockID,prevID)
		prevID = blockID
	
	return rightActions

def twoArms(startState,endState,numBlocks):
	startI,endI = translateStateInfo(startState,endState) # 1=ascending, 2=descending
	
	# If already done, return
	if startI == endI:
		return []
	
	# Scatter blocks
	scatterOrder = range(1,numBlocks+1)
	if(startI == 1):
		scatterOrder.reverse()
	
	# Prepare for synchronization
	rightActions = []
	leftActions = padScatterBlock()
	
	# Odd/Even Sorting
	leftBottomID = -1
	rightBottomID = -1
	
	for blockID in scatterOrder:
		if len(rightActions) < len(leftActions):
			# Moving blockID to right side
			if endI == 4:
				if rightBottomID == -1:
					rightBottomID = blockID
					rightActions += scatterBlock(blockID,2)
				else:
					rightActions += scatterOnto(blockID,rightBottomID)
			else:
				rightActions += scatterBlock(blockID,2)
			
		else:
			# Moving blockID to left side
			if endI == 4:
				if leftBottomID == -1:
					leftBottomID = blockID
					leftActions += scatterBlock(blockID,1)
				else:
					leftActions += scatterOnto(blockID,leftBottomID)
			else:
				leftActions += scatterBlock(blockID,1)
			
	
	# If final state is scatter, we're done!
	if endI == 3 or endI == 4:
		return rightActions
	
	# Synchronize
	if len(rightActions) > len(leftActions):
		leftActions = leftActions + padScatterBlock()
	else:
		rightActions = rightActions + padScatterBlock()
		
	# Stagger
	leftActions += padStackBlock() # right goes first
	
	# Re-stack blocks
	stackOrder = range(1,numBlocks+1)
	if(endI == 2):
		stackOrder.reverse()
	
	prevID = None
	for blockID in scatterOrder:
		if len(rightActions) < len(leftActions):
			rightActions += stackBlock(blockID,prevID)
		else:
			leftActions += stackBlock(blockID,prevID)
		prevID = blockID
	
	# Synchronize arms
	if len(rightActions) < len(leftActions):
		rightActions += padStackBlock()
	else:
		leftActions += padStackBlock()
			
	return (leftActions,rightActions)

# FALLBACK

# Uses one arm to finish operation
def fallback(worldState,finalState,numBlocks):
	endState,_ = translateStateInfo(finalState,finalState)
	
	# Scatter first
	rightActions = []
	
	# Find all stacks
	stacks = worldState.grid.stacks
	tallStacks = []
	for stack in stacks:
		if len(stack.blocks) > 1:
			tallStacks.append(stack)
	
	# Now we have all stacks that need to be deconstructed
	# Scatter these
	for stack in tallStacks:
		blocks = stack.blocks.reverse()
		for blockID in blocks:
			rightActions += scatterBlock(blockID,0)
	
	# If final state is scatter, we're done
	if endState == 3:
		return rightActions
	
	# Re-stack blocks
	stackOrder = range(1,numBlocks+1)
	if(endState == 2):
		stackOrder.reverse()
	
	prevID = None
	for blockID in scatterOrder:
		rightActions += stackBlock(blockID,prevID)
		prevID = blockID
	
	return rightActions
	# TODO: if block is already on center square, issue! Maybe add to scatter list later?

# TESTS

#print ("\nOne arm solution:\n")

#for item in oneArm("stacked_ascending","scattered",0,0,3):
#	print(item)

#print ("\n\nTwo arm solution:\n")
#
#(leftActions,rightActions) = twoArms("stacked_descending","stacked_ascending",0,0,4)
#for i in range(len(leftActions)):
#	left = leftActions[i]
#	right = rightActions[i]
#	print(left + "\t" + right)
