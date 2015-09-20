#!/usr/bin/env python

import rospy
from gabe_ricky_sarah_proj1.srv import*
from gabe_ricky_sarah_proj1.msg import*
from std_msgs.msg import String

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

def moveToHome(): # TODO: MAKE SURE ROBOT_INTERFACE CALCULATES ROW,COL or BLOCK.
	return createAction(HOME_TARGET, MOVE_OVER_OPERATION, "move to home       ")

def moveAboveTable(area):
	return createAction(area*-1, MOVE_OVER_OPERATION, "move over table a ")

def still():
	return createAction(NO_TARGET, NO_OPERATION, "still              ")

BLOCK_TARGET = 1 # Any positive number = blockID
TABLE_TARGET = 0
TABLE_LEFT_TARGET = -1
TABLE_RIGHT_TARGET = -2
HOME_TARGET = -3
GRIPPER_TARGET = -4
NO_TARGET = -5

OPEN_OPERATION = 1
CLOSE_OPERATION = 2
MOVE_OVER_OPERATION = 3
MOVE_TO_OPERATION = 4
NO_OPERATION = 5

def createDormantAction(target, operation):
	return (target, operation)

def runAction(dormantAction):
	runAction(dormantActionLeft, createDormantAction(NO_TARGET,NO_OPERATION))

# WHEN CHANGING GRIPPER, TARGET=BLOCK

def runAction(dormantActionLeft, dormantActionRight):
	# TODO: Create actions
   	rospy.wait_for_service('move_robot')
   	try:
    	move_robot_handle = rospy.ServiceProxy('move_robot', MoveRobot) # TODO: DO WE HAVE TO DO THIS EVERY TIME? OR SHOULD WE PASS THIS IN
        return move_robot_handle(action1, target1, action2, target2)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
	