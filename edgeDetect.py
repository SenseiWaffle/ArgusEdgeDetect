
"""
Finding the Area of an Artery through Image Manipulation
Written by Tianpeng ( Tony ) Feng
"""

import numpy as np
import cv2 as cv

imgName = "pig2.png"         # Image to be read
imgOG = cv.imread( imgName ) # Copy of original image for comparison purposes
img = cv.imread( imgName )

area = 0
pixLen = 0.0078       # The length that 1 pixel represents in millimeters
xLen = img.shape[ 0 ] # Number of rows
yLen = img.shape[ 1 ] # Number of cols
scannerWidth = 120    # Pixel width of scanner footprint.
pixThreshold = 100    # Brightness value
avgThreshold = 150    # Average pixel value
if len( img.shape ) < 2:
    channel = img.shape[ 2 ]

# Get rid of left most side of image.
for i in range( xLen ):
    for j in range( scannerWidth ):
        #img[ i, j ] = [ 0, 0, 0 ]
        img.itemset( ( i, j, 0 ), 0 )
        img.itemset( ( i, j, 1 ), 0 )
        img.itemset( ( i, j, 2 ), 0 )

# Replace all pixels with brightness over the pixel threshhold
for i in range( xLen ):
    for j in range( yLen ):
        if img.item( i, j, 1 ) > pixThreshold :
            #img[ i, j ] = [ 0, 0, 0 ]
            img.itemset( ( i, j, 0 ), 0 )
            img.itemset( ( i, j, 1 ), 0 )
            img.itemset( ( i, j, 2 ), 0 )
        else:
            img[ i, j ] = [ 255, 255, 255 ] # For ease of seeing what is left behind

# Replace black pixels that are surrounded by mostly white with white pixels ( noise reduction )
average = 0
for i in range( 5, xLen - 5 ):
    for j in range( 5, yLen - 100 ):
        if( img.item( i, j, 0 ) == 0 ): # All pixels are black or white, therefore if the blue value is 0, it is black
            for a in range( i - 4, i + 4 ): # Iterate through a 9 by 9 area around the pixel...
                for b in range( j - 4, j + 4 ):
                    average += img.item( a, b, 0 )

            average /= 81 # Average pixel value
            if average >= avgThreshold:
                img[ i, j ] = [ 255, 255, 255 ] # Set center pixel value to white
            average = 0 # Reset average

# Boundary finder. If the pixel to the left is white and 80% of pixels to the right are black, it is a boundary pixel
count = 0
for i in range( xLen ):
    for j in range( scannerWidth, yLen ):
        if( img.item( (i, j, 0) ) == 0 ) and ( img.item( (i, j-2, 0) ) == 255 ):
            for t in range( 25 ):
                if img.item( i, j + t, 0 ) == 0:
                    count += 1
        if count >= 20:
            img.itemset( (i, j, 2), 255 )
            count = 0

# Dialate and erode the image
kernel = np.ones ((3,3), np.uint8 )
d_im = cv.dilate( img, kernel, iterations=1 )
img = cv.erode( d_im, kernel )

# Delete all black pixels
img[ np.where( ( img == [ 0, 0, 0 ] ).all( axis = 2 ) ) ] = [ 255, 255, 255 ]


"""
Finds a starting point based on parameters.

( Starting x point, stopping x point, starting y point, stopping y point, step direction for x, step direction for y )
Returns the point determined as the starting coordinate
"""
def findStartPoint( xStart, xStop, yStart, yStop, xStep, yStep ): # Returns startPoint in form [x,y]
    point = [0,0]
    i = 0
    running = 1
    while( running ):
        for i in range( xStart, xStop, xStep ):
            for j in range( yStart, yStop, yStep ):
                if( img.item( i, j, 0 ) == 0 ): # Check to see if the blue value is 0
                    point = [i,j]
                    running = 0
                    break
                if j == yStop-1:
                    i += 1
            if running == 0:
                break
    return point


"""
Draw the boundary based off of the location of nearby RED pixels. In this case, red is the color of a boundary pixel.

( Starting coordinate, min x search range, max x search range, x step direction,
  min y search range, max y search range, y step direction,
  image, x length of image, y length of image )
Returns the point at which no more pixels were in reach of the searching algorithm
"""
def drawBoundary( startPoint, xIncStart, xIncStop, xStep, yIncStart, yIncStop, yStep, img, imgXLen, imgYLen ):
    global area         # Import these variables into local scope
    global pixLen
    global scannerWidth

    # Defining local variables needed
    currentPoint = startPoint
    nextPoint = [0,0]
    found = 0
    done = 0
    running = 1
    cx, cy = currentPoint[ 0 ], currentPoint[ 1 ]
    nx, ny = currentPoint[ 0 ], currentPoint[ 1 ]

    while( running ):
        # This will make sure the the pixel is always within search boundary
        if cx < 0 or cx >= imgXLen:
            break
        if cy < 0 or cy >= imgYLen:
            break

        #print( currentPoint )

        for i in range( xIncStart, xIncStop, xStep ):         # Iterate through a small rectangle whose
            if done == 0:                                     # dimensions are specified by the given
                for j in range( yIncStart, yIncStop, yStep ): # range variables
                    nx = cx + i
                    ny = cy + j

                    if nx == cx and ny == cy:
                        ny += 1
                    #print( "Next point's position: {}, {}".format( nx, ny ) )
                    #print( "   Point's color: {}, {}, {}".format( img[ nx, ny, 0 ],img[ nx, ny, 1 ],img[ nx, ny, 2 ] ) )

                    if img[ nx, ny, 1 ] == 0: # If the pixel's green value is 0...
                        area += ( ( ny - scannerWidth ) * pixLen**2 ) # Find the area represented by the row
                        found = 1
                        done = 1
                        break
                if done:
                    break
        if found:
            found = 0
            done = 0
            #print( "Drawing a line from: ( {}, {} ) to ( {}, {} )".format( cx, cy, nx, ny ) )
            cv.line( img, (cy,cx), (ny,nx), (0,255,0), 3 ) # NOTE: cv.line uses (y,x) as opposed to our system of (x,y)
            #cv.imshow( "Updating...", img )
            #cv.waitKey( 0 )

            holder = cx
            cx = nx
            nx = holder
            holder = cy
            cy = ny
            ny = holder
            currentPoint = [ cx, cy ] # Switch values of the currentPoint with nextPoint
            #print( "Found!\n" )
            pass
        else:
            stopPoint = currentPoint # Save this coordinate for returning
            #print( "\nDid not find a red point.\n" )
            running = 0
            break

    return stopPoint


stopPoint1 = [0,0]
stopPoint2 = [0,0]

# Find initial start point starting from the top
startPoint = findStartPoint( 0, xLen, scannerWidth+1, yLen, 1, 1 )
stopPoint1 = drawBoundary( startPoint, 1, 8, 1, -6, 6, 1, img, xLen, yLen )

# Now run the above code from bottom to top
startPoint = findStartPoint( xLen-1, -1, scannerWidth+1, yLen, -1, 1 )
stopPoint2 = drawBoundary( startPoint, -8, -1, 1, -6, 6, 1, img, xLen, yLen )

# Bridge the gap linearly
cv.line( img, (stopPoint1[1],stopPoint1[0]), (stopPoint2[1],stopPoint2[0]), (0,255,0), 3 )
averageWidth = ( stopPoint1[1] + stopPoint2[1] ) / 2
area += ( ( averageWidth - scannerWidth ) * ( stopPoint2[0] - stopPoint1[0] ) * ( pixLen ) )

# Show the original and resulting image side by side and save the result as a .png
cv.imshow( "Original", imgOG )
cv.imshow( "Result", img )
cv.imwrite( "result.png", img )
print( "\nArea: {} mm^2".format( area ) )
cv.waitKey( 0 )
quit() # Finished!
