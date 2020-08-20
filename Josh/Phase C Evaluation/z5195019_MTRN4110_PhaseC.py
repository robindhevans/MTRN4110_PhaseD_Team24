#!/usr/bin/env python
# coding: utf-8

# In[3]:


import cv2 # OpenCV library
import numpy as np # Numpy library for scientific computing
import matplotlib.pyplot as plt # Matplotlib library for plotting
import math # math library for carrying out circle plot

ROBOT_FILE_NAME = 'PHASEC_TEST_IMAGES/Robot_3.png'
MAZE_FILE_NAME = 'PHASEC_TEST_IMAGES/Maze_3.png'
MAP_FILE_NAME = 'MapFound.txt'


# # Task 1 - Read in an image and display it.

# In[4]:


#read image
maze_img = cv2.imread(MAZE_FILE_NAME)
maze_RGB = cv2.cvtColor(maze_img, cv2.COLOR_BGR2RGB)
#display image
#plt.figure(figsize = (18, 10))
#plt.imshow(maze_RGB)
#plt.show()


# # Threshold for corner detection

# In[71]:


#convert image to HSV
maze_HSV = cv2.cvtColor(maze_img, cv2.COLOR_BGR2HSV)
#set threhold values for hue, saturation, value
corLower = np.array([140, 200, 105])
corUpper = np.array([160, 255, 255])
#apply threshold and convert to grayscale
mask = cv2.inRange(maze_HSV, corLower, corUpper)
maskBW = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
#create kernel
kernel = np.ones((3,3), np.uint8)
#apply erosion
erosion = cv2.erode(maskBW, kernel, iterations = 1)
#isolate the different channels
H, S, V = cv2.split(erosion)
#display corners
#plt.figure(figsize = (18, 10))
#plt.imshow(erosion)
#plt.show()


# # Find Corners

# In[72]:


#function to detect the location of the corners of the maze
def corner_location (yMin, yMax, xMin, xMax, V):
    block_x = []
    block_y = []
    x = 0
    y = 0
    for x in range(xMin, xMax):
        for y in range(yMin, yMax):
            if(V[y,x] != 0):
                block_x.append(x)
                block_y.append(y)
    
    coordx = sum(block_x)/len(block_x)
    coordy = sum(block_y)/len(block_y)
    
    return [int(coordx), int(coordy)]
            
#define maze width etc
maze_width = 1350
maze_half_height = 375
maze_height = 750
left_limit = 338
right_limit = 1013


#top left
top_left = corner_location(0, maze_half_height, 0, left_limit, V)
#top right
top_right = corner_location(0, maze_half_height, right_limit, maze_width, V)
#bottom left
bottom_left = corner_location(maze_half_height, maze_height, 0, left_limit, V)
#bottom right
bottom_right = corner_location(maze_half_height, maze_height, right_limit, maze_width, V)


#print(top_left, top_right, bottom_left, bottom_right)


# # Task 2 - Find four cornerstiones of maze

# In[73]:


#function to draw the corners of the maze in
def corner_indicator(cor_x, cor_y, image_RGB):
    for x in range(cor_x-10, cor_x+10):
        for y in range(cor_y-2, cor_y+2):
            if(x > 0 and x < 1350 and y > 0 and y < 750):
                image_RGB[y,x] = [0,205,212]
        
    for y in range(cor_y-10, cor_y+10):
        for x in range(cor_x-2, cor_x+2):
            if(x > 0 and x < 1350 and y > 0 and y < 750):
                image_RGB[y,x] = [0,205,212]
    
    return image_RGB

#make a coy of the immage
cor_RGB = cv2.imread(MAZE_FILE_NAME)
cor_RGB = cv2.cvtColor(cor_RGB, cv2.COLOR_BGR2RGB)

#top left
cor_RGB = corner_indicator(top_left[0], top_left[1], cor_RGB)
#top right
cor_RGB = corner_indicator(top_right[0], top_right[1], cor_RGB)
#bottom left
cor_RGB = corner_indicator(bottom_left[0], bottom_left[1], cor_RGB)
#bottom right
cor_RGB = corner_indicator(bottom_right[0], bottom_right[1], cor_RGB)
#display marked up image
#plt.figure(figsize = (18, 10))
#plt.imshow(cor_RGB)
#plt.show()


# # Task 3 - Perspective transform
# 

# In[74]:


pts1 = np.float32([top_left, top_right, bottom_left, bottom_right]) # four points on the first image
pts2 = np.float32([[0,0],[1350,0],[0,750],[1350,750]]) # four points on the second image
#get transformation matrix
H = cv2.getPerspectiveTransform(pts1,pts2) # homography matrix
#print(H)

#apply image trasnformation
maze_warp = cv2.warpPerspective(maze_RGB, H, (1350,750))

#plt.figure(figsize = (18, 10))
#plt.imshow(maze_warp)
#plt.show()


# # Threshold for wall detection

# In[75]:


#create thresholds for the 
wallLower = np.array([15, 40, 215])
wallUpper = np.array([20, 130, 255])

maze_HSV = cv2.cvtColor(maze_warp, cv2.COLOR_RGB2HSV)
#apply thresholding to the image
wall_mask = cv2.inRange(maze_HSV, wallLower, wallUpper)
wall_maskBW = cv2.cvtColor(wall_mask, cv2.COLOR_GRAY2BGR)

kernel = np.ones((3,3), np.uint8)
#apply closing to the threshold
erosion_wall =  cv2.morphologyEx(wall_maskBW, cv2.MORPH_CLOSE, kernel)

H1, S1, V1 = cv2.split(erosion_wall)

#plt.figure(figsize = (18, 10))
#plt.imshow(erosion_wall)
#plt.show()


# # Detecting walls and storing in array

# In[76]:


def check_horz_wall (row, col, wall_mask):
    pix_test = [25,50,75,100,125]
    wall_inc = 150
    count = 0
    
    for index in pix_test:
        if(V1[row*wall_inc,(col*wall_inc)+index] > 0):
            count += 1
            
    if (count > 3):
        return 1
    else:
        return 0
    
def check_vert_wall (row, col, wall_mask):
    pix_test = [25,50,75,100,125]
    wall_inc = 150
    count = 0
    
    for index in pix_test:
        if(V1[(row*wall_inc)+index,col*wall_inc] > 0):
            count += 1
            
    if (count > 3):
        return 1
    else:
        return 0
    
#set up storage arrays
h_walls = np.zeros((4,9))
v_walls = np.zeros((5,8))

#detect horizontal walls
for row in range(1,5):
    for col in range(0,9):
        h_walls[row-1,col] = check_horz_wall(row, col, wall_mask)
        
#print('horizontal')
#print(h_walls)

#detect vertical walls
for row in range(0,5):
    for col in range(1,9):
        v_walls[row,col-1] = check_vert_wall(row, col, wall_mask)
        
#print('vertical_walls')
#print(v_walls)
            


# # Task 4 - Detect all the internal walls

# In[77]:


def paint_Hwall(col, row, maze_walls):
    wall_inc = 150
    col_index = (col*wall_inc)+150
    row_index = row*wall_inc
    for x in range(row_index, row_index+150):
        for y in range(col_index-4, col_index+4):
            maze_walls[y,x] = [0,205,212] 
            
    return maze_walls

def paint_Vwall(col, row, maze_walls):
    wall_inc = 150
    col_index = col*wall_inc
    row_index = (row*wall_inc)+150
    for x in range(row_index-4, row_index+4):
        for y in range(col_index, col_index+150):
            maze_walls[y,x] = [0,205,212] 
            
    return maze_walls

#make a copy of the warped image
maze_walls = maze_warp.copy()            
    
#iterate through arrays and paint walls    
for row in range(0,4):
    for col in range(0,9):
        if(h_walls[row,col]==1):
            maze_walls = paint_Hwall(row, col, maze_walls)
            
for row in range(0,5):
    for col in range(0,8):
        if(v_walls[row,col]==1):
            maze_walls = paint_Vwall(row, col, maze_walls)

        
#plt.figure(figsize = (18, 10))
#plt.imshow(maze_walls)
#plt.show()


# In[78]:


#read in image of robot
rbt_bgr = cv2.imread(ROBOT_FILE_NAME)
rbt_rgb = cv2.cvtColor(rbt_bgr,cv2.COLOR_BGR2RGB)

rbt_warp = cv2.warpPerspective(rbt_rgb, H, (750,750))
# Load the predefined dictionary
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
# Initialize the detector parameters using default values
parameters =  cv2.aruco.DetectorParameters_create()
#detect the Aruco makrer
markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(rbt_warp, dictionary, parameters=parameters)

#test = cv2.aruco.drawDetectedMarkers(rbt_warp, markerCorners, markerIds)
#plt.figure(figsize = (10, 10))
#plt.imshow(test)
#plt.show()

#find centre of symbol
av_x = 0
av_y = 0
center = [0,0]

for x in range(0,4):
    av_x += markerCorners[0][0][x][0]
    av_y += markerCorners[0][0][x][1]
    
    
center[0] = int(av_x/4)
center[1] = int(av_y/4)


#determine orientation
      
south = markerCorners[0][0][0][1]
direction = 0

for x in range(1,4):
    if (markerCorners[0][0][x][1] > south):
        #print('new south',south)
        south = markerCorners[0][0][x][1]
        direction = x

#print(direction)


# # Task 5 - Detect location and heading of robot

# In[79]:


#draw indicator on robot
def robot_indicator(center, direction, maze_rgb):
    robot_char = ' '
    #circle
    omega = np.arange(0.0, 6.3, 0.02)
    for om in omega:
        for rad in range(30,35):
            xplot = center[1] + (rad * math.sin(om))
            yplot = center[0] + (rad * math.cos(om))
            maze_rgb[int(xplot),int(yplot)] = [240,21,244]
    
    
    # South
    if(direction == 0):
        robot_char = 'V'
        tip = center[1] + 20
        for thick in range(-2,3):
            for rad in range(0,35):
                y = tip - (rad*math.cos(0.5))
                xL = center[0] + (rad*math.sin(0.5)) + thick
                xR = center[0] - (rad*math.sin(0.5)) + thick
                maze_rgb[int(y), int(xR)] = [240,21,244]
                maze_rgb[int(y), int(xL)] = [240,21,244]
    # North
    elif(direction == 2):
        robot_char = '>'
        tip = center[1] - 20
        for thick in range(-2,3):
            for rad in range(0,35):
                y = tip + (rad*math.cos(0.5))
                xL = center[0] + (rad*math.sin(0.5)) + thick
                xR = center[0] - (rad*math.sin(0.5)) + thick
                maze_rgb[int(y), int(xR)] = [240,21,244]
                maze_rgb[int(y), int(xL)] = [240,21,244]
    
    # East
    elif(direction == 1):
        robot_char = '^'
        tip = center[0] + 20
        for thick in range(-2,3):
            for rad in range(0,35):
                x = tip - (rad*math.cos(0.5))
                yup = center[1] + (rad*math.sin(0.5)) + thick
                ydwn = center[1] - (rad*math.sin(0.5)) + thick
                maze_rgb[int(yup), int(x)] = [240,21,244]
                maze_rgb[int(ydwn), int(x)] = [240,21,244]

    # West
    elif(direction == 3):
        robot_char = '<'
        tip = center[0] - 20
        for thick in range(-2,3):
            for rad in range(0,35):
                x = tip + (rad*math.cos(0.5))
                yup = center[1] + (rad*math.sin(0.5)) + thick
                ydwn = center[1] - (rad*math.sin(0.5)) + thick
                maze_rgb[int(yup), int(x)] = [240,21,244]
                maze_rgb[int(ydwn), int(x)] = [240,21,244]
                
    return robot_char
#Return centre of robot by averaging thresholded image of maze
def robot_location (V):
    block_x = []
    block_y = []
    x = 0
    y = 0
    corner = 0
    for x in range(0, 1350):
        for y in range(0, 750):
            if(V[y,x]):
                block_x.append(x)
                block_y.append(y)
                
    
    coordx = sum(block_x)/len(block_x)
    coordy = sum(block_y)/len(block_y)
    
    if (coordx < 675):
        if (coordy < 375):
            marker = (1,2)
        else:
            marker = (9,2)
    else:
        if (coordy < 375):
            marker = (1,34)
        else:
            marker = [9,34]
                
    return (marker, [int(coordx), int(coordy)])

#Convert image to HSV
rbt_walls = cv2.cvtColor(maze_walls, cv2.COLOR_RGB2HSV)

rbtLower = np.array([25, 30, 0])
rbtUpper = np.array([110, 145, 215])

maskrbt = cv2.inRange(rbt_walls, rbtLower, rbtUpper)
maskrbtBW = cv2.cvtColor(maskrbt, cv2.COLOR_GRAY2BGR)

kernel = np.ones((3,3),np.uint8)

erosionrbt = cv2.morphologyEx(maskrbtBW, cv2.MORPH_OPEN, kernel)

H, S, V = cv2.split(erosionrbt)

#plt.figure(figsize = (18, 10))
#plt.imshow(erosionrbt)
#plt.show()

marker, av_pos = robot_location(V)

#print(marker, av_pos)

robot_char = robot_indicator(av_pos, direction, maze_walls)

#plt.figure(figsize = (18, 10))
#plt.imshow(maze_walls)
#plt.show()
      


# # Task 6 - Generate map and write it to a text file

# In[80]:


#Functions to insert walls into txt file
def vertical_row(wall_value):
    map_row = ""
    map_row += "|   "
    for x in wall_value:
        if(x):
            map_row += "|   "
        else:
            map_row += "    "
            
    map_row += "|"
    return map_row

def horizontal_row(wall_value):
    map_row = ""
    map_row += ' '
    
    for x in wall_value:
        if(x):
            map_row += "--- "
        else:
            map_row += "    "
            
    map_row += " "
    return map_row
#Places arrow indicator for robot position and direction
def insert_robot(map_array, direction, marker):
    if(direction == 0):
        text = map_array[marker[0]]
        text = text[:marker[1]] + 'v' + text[(marker[1]+1):]
        map_array[marker[0]] = text
    elif(direction == 1):
        text = map_array[marker[0]]
        text = text[:marker[1]] + '>' + text[(marker[1]+1):]
        map_array[marker[0]] = text
    elif(direction == 2):
        text = map_array[marker[0]]
        text = text[:marker[1]] + '^' + text[(marker[1]+1):]
        map_array[marker[0]] = text
    else:
        text = map_array[marker[0]]
        text = text[:marker[1]] + '<' + text[(marker[1]+1):]
        map_array[marker[0]] = text
        
    return 0
            

map_array = []

map_array.append(" --- --- --- --- --- --- --- --- --- ")

for y in range(1,10):
    if (y % 2):
        map_array.append(vertical_row(v_walls[int(y/2)]))
    else:
        map_array.append(horizontal_row(h_walls[int(y/2) - 1]))
                         
map_array.append(" --- --- --- --- --- --- --- --- --- ")

insert_robot(map_array, direction, marker)
    

output = open(MAP_FILE_NAME, "w")
for line in map_array:
    # write line to output file
    output.write(line)
    output.write("\n")
output.close()
        


# In[ ]:




