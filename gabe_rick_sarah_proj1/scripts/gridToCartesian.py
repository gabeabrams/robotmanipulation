import robot_interface

gridWidth = 0
gridHeight = 0
gridDepth = 0

homeRow = 0
homeCol = 0

BaxX0 = 0
BaxY0 = 0
BaxZ0 = 0
BaxOrient = None

BLOCK_SIDE = 1.75*.00254 # meters

# Usage: initGridToCartesian((x,y,z),(rows,cols),(width,height),numBlocks)
def initGridToCartesian(tableRowCols, tableDimensions):
	global homeRow
	global homeCol
	(homeRow, homeCol) = robot_interface.getHomeLoc()
	
	(rows,cols) = tableRowCols
	(width,height) = tableDimensions
	
	global gridWidth
	global gridHeight
	global gridDepth
	gridWidth = width/cols
	gridHeight = height/rows
	gridDepth = BLOCK_SIDE

def toCartesian(row,col,height):
	global gridWidth
	global gridHeight
	global gridDepth
	
	rowDiff = row-homeRow
	colDiff = col-homeCol
	
	x = colDiff*gridWidth
	y = rowDiff*gridHeight
	
	z = (height-1)*gridDepth
	return (x,y,z)

def initToBaxter(rightMover):
	global BaxX0
	global BaxY0
	global BaxZ0
	global BaxOrient

	numBlocks = robot_interface.getNumBlocks()

	pose = rightMover.endpoint_pose()
	position = pose['position']
	orientation = pose['orientation']
	BaxX0 = position[0]
	BaxY0 = position[1]
	BaxZ0 = position[2] - (numBlocks-1)*BLOCK_SIDE
	BaxOrient = orientation

def toBaxter(x,y,z):
	global BaxX0
	global BaxY0
	global BaxZ0

	newx = y+BaxX0
	newy = (-1*x)+BaxY0
	newz = z+BaxZ0
	return (newx, newy, newz)

def getBaxOrient():
	global BaxOrient
	return BaxOrient	