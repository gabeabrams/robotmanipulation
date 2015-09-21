#!/usr/bin/env python

#import rospy
#from gabe_ricky_sarah_proj1.srv import*
#from gabe_ricky_sarah_proj1.msg import*
#from std_msgs.msg import String

# TODO: First few actions already done

# SIMPLE ACTIONS
def openGripper():
	return createAction(GRIPPER_TARGET,OPEN_OPERATION, "open gripper       ")


def closeGripper():
	return createAction(GRIPPER_TARGET,CLOSE_OPERATION, "close gripper      ")

def moveToBlock(blockID):
	return createAction(blockID, MOVE_TO_OPERATION, "move to block     ")

def moveOverBlock(blockID):
	return createAction(blockID, MOVE_OVER_OPERATION, "move over block   ")

def moveToHome():
	return createAction(HOME_TARGET, MOVE_OVER_OPERATION, "move to home       ")

def moveAboveTable(area):
	return createAction(area*-1, MOVE_OVER_OPERATION, "move over table a ")

def still():
	return createAction(NO_TARGET, NO_OPERATION, "still              ")

#BLOCK_TARGET = 1 # Any positive number = blockID
#TABLE_TARGET = 0
#TABLE_LEFT_TARGET = -1
#TABLE_RIGHT_TARGET = -2
#HOME_TARGET = -3
#GRIPPER_TARGET = -4
#NO_TARGET = -5

#OPEN_OPERATION = 1
#CLOSE_OPERATION = 2
#MOVE_OVER_OPERATION = 3
#MOVE_TO_OPERATION = 4
#NO_OPERATION = 5

def createDormantAction(target, operation):
	createDormantAction(target,operation,"")
	
def createDormantAction(target, operation, text):
	return ("","",text)
	#act = Action()
	#tar = Target()
	#if operation == 1: # Open
	#	act.type = 2
	#	tar.blockID = 1
	#if operation == 2: # Close
	#	act.type = 3
	#	tar.blockID = 1
	#if operation == 3: # Move Over
	#	act.type = 7
	#	tar.blockID = target
	#if operation == 4: # Move To
	#	act.type = 5
	#	tar.blockID = target
	#if operation == 5: # Still
	#	act.type = 11
	#	tar.blockID = 1
		
	#return (act,target,text)

def runAction(dormantAction):
	runAction(dormantActionLeft, createDormantAction(NO_TARGET,NO_OPERATION))

def runAction(dormantActionLeft, dormantActionRight):
	(action1,target1,text1) = dormantActionLeft
	(action2,target2,text2) = dormantActionRight
   	rospy.wait_for_service('move_robot')
   	try:
   		move_robot_handle = rospy.ServiceProxy('move_robot', MoveRobot)
   		return move_robot_handle(action1, target1, action2, target2)
   	except rospy.ServiceException, e:
   		print "Service call failed: %s"%e
	