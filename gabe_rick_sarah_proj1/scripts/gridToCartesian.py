x0 = 0
y0 = 0
z0 = 0

gridWidth = 0
gridHeight = 0
gridDepth = 0

homeRow = 0
homeCol = 0

BLOCK_SIDE = 1.75 # WHAT UNITS ARE WE USING?

# Usage: initGridToCartesian((x,y,z),(homeRow,homeCol),(rows,cols),(width,height),numBlocks)
def initGridToCartesian(baxterCoords,homeRowCol,tableRowCols, tableDimensions, numBlocks):
	(x,y,z) = baxterCoords
	x0 = x
	y0 = y
	z0 = z - numBlocks * BLOCK_SIDE
	
	(rows,cols) = tableRowCols
	(homeRowA,homeColA) = homeRowCol
	homeRow = homeRowA
	homeCol = homeColA
	(width,height) = tableDimensions
	
	gridWidth = width/cols
	gridHeight = height/rows
	
	gridDepth = BLOCK_SIDE

def toCartesian(row,col,height):
	rowDiff = row-homeRow
	colDiff = col-homeCol
	
	xDiff = colDiff*gridWidth
	yDiff = rowDiff*gridHeight
	
	x = x0 + xDiff
	y = y0 + yDiff
	z = height*gridDepth
	
	return (x,y,z)