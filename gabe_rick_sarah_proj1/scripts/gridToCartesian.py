gridWidth = 0.0
gridHeight = 0.0
gridDepth = 0.0

homeRow = 0
homeCol = 0
numBlocks = 0

BaxX0 = 0.0
BaxY0 = 0.0
BaxZ0 = 0.0
BaxOrient = None

BLOCK_SIDE = 1.75*.0254 # meters

# Usage: initGridToCartesian((x,y,z),(rows,cols),(width,height),numBlocks)
def initGridToCartesian(tableRowCols, tableDimensions, numB, home):
	global homeRow
	global homeCol
	global numBlocks
	(homeRow, homeCol) = home
	numBlocks = numB
	
	(rows,cols) = tableRowCols
	(width,height) = tableDimensions
	
	global gridWidth
	global gridHeight
	global gridDepth
	
	gridWidth = float(width)/float(cols)
	gridHeight = float(height)/float(rows)
	gridDepth = BLOCK_SIDE

def toCartesian(row,col,height):
	global gridWidth
	global gridHeight
	global gridDepth
	
	print gridWidth
	print gridWidth
	
	rowDiff = row-homeRow
	colDiff = col-homeCol
	
	x = float(colDiff)*gridWidth
	y = float(rowDiff)*gridHeight
	
	z = (height)*gridDepth
	# print "Translating rch to xyz: rowDiff=" + str(rowDiff) + ",colDiff=" + str(colDiff) + " (" + str(row) + "," + str(col) + "," + str(height) + ") > (" + str(x) + "," + str(y) + "," + str(z) + ")"
	return (x,y,z)

def initToBaxter(mover):
	global BaxX0
	global BaxY0
	global BaxZ0
	global BaxOrient
	global numBlocks

	pose = mover.endpoint_pose()
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
