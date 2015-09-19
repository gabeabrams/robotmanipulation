#!/usr/bin/env python

import action_handler

# COMPOSITE ACTIONS
# area: 1=left, 2=right, 0=either

# During this phase, tuple order is: (CENTER, OUTSKIRTS)
# 3 THEN 3
def padScatterBlock():
	return [still(),still(),still(),still()]
	
def scatterBlock(blockID,area):
	return [still(),moveOverBlock(blockID),moveToBlock(blockID),closeGripper(),     moveAboveTable(area),openGripper(),still(),still()]

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
	return (startOut,endOut)
	
def oneArm(startState,endState,stackRow,stackCol,numBlocks):
	startI,endI = translateStateInfo(startState,endState) # 1=ascending, 2=descending
	
	# If already done, return
	if startI == endI:
		return []
	
	# Scatter blocks
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

def twoArms(startState,endState,stackRow,stackCol,numBlocks):
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
	
	for blockID in scatterOrder:
		if len(rightActions) < len(leftActions):
			rightActions += scatterBlock(blockID,2)
		else:
			leftActions += scatterBlock(blockID,1)
	
	# If final state is scatter, we're done!
	if endI == 3:
		return rightActions
	
	# Synchronize
	if len(rightActions) > len(leftActions):
		leftActions = leftActions + padScatterBlock()
	else:
		rightActions = rightActions + padScatterBlock()
	# Stagger TODO: right always go first okay?
	leftActions += padStackBlock() # right goes first
	#if numBlocks % 2 == 1:
	#	# If odd number of blocks, right has to start both times
	#	leftActions = leftActions + padStackBlock()
	#else:
	#	rightActions = rightActions + padStackBlock()
	
	
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
def fallback(worldState,finalState,stackRow,stackCol,numBlocks):
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

print ("\nOne arm solution:\n")

for item in oneArm("stacked_ascending","scattered",0,0,3):
	print(item)

print ("\n\nTwo arm solution:\n")

(leftActions,rightActions) = twoArms("stacked_descending","stacked_ascending",0,0,4)
for i in range(len(leftActions)):
	left = leftActions[i]
	right = rightActions[i]
	print(left + "\t" + right)
