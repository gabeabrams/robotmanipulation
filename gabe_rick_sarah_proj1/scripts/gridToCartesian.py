import robot_interface

x0 = 0
y0 = 0
z0 = 0

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
	
	xDiff = colDiff*gridWidth
	yDiff = rowDiff*gridHeight
	
	x = x0 + xDiff
	y = y0 + yDiff
	z = (height-1)*gridDepth
	return (x,y,z)

def initToBaxter(rightMover):
	global BaxX0
	global BaxY0
	global BaxZ0
	global BaxOrient

	pose = rightMover.endpoint_pose()
	BaxX0 = pose.position.x
	BaxY0 = pose.position.y
	BaxZ0 = pose.position.z - (numBlocks-1)*BLOCK_SIDE
	BaxOrient = pose.orientation

def toBaxter(x,y,z):
	newx = y+Y0
	newy = (-1*x)+Y0
	newz = z+Z0
	return (newx, newy, newz)