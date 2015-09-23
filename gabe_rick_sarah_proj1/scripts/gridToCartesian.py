import robot_interface

x0 = 0
y0 = 0
z0 = 0

gridWidth = 0
gridHeight = 0
gridDepth = 0

homeRow = 0
homeCol = 0

BLOCK_SIDE = 1.75*.00254 # meters

# Usage: initGridToCartesian((x,y,z),(homeRow,homeCol),(rows,cols),(width,height),numBlocks)
def initGridToCartesian(baxterCoords, tableRowCols, tableDimensions):
	(x,y,z) = baxterCoords

	global homeRow
	global homeCol
	(homeRow, homeCol) = robot_interface.getHomeLoc()

	global x0
	global y0
	global z0
	x0 = x
	y0 = y
	z0 = z - numBlocks + 1 * BLOCK_SIDE
	
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