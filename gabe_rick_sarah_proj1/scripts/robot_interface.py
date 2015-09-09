#!/usr/bin/env python

import rospy
from gabe_ricky_sarah_proj1.srv import*
from gabe_ricky_sarah_proj1.msg import*

#Answers move_robot requests
def move_robot(req):
	print "Checking validity"
	return true

#Answers get_state requests
def get_state(req):
	print "Retrieving state"
	return state	

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

if __name__ == "__main__":
	robot_interface()	
	
