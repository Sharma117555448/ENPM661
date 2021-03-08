'''
author @Charu Sharma
        117555448
        charu107@umd.edu
'''
import cv2
import numpy as np
from datetime import datetime

# Defining the map
robot_world = np.zeros((300,400), dtype= np.uint8)

# Plotting the obstacles in the map
cv2.circle(robot_world, (90, 229), 35, 255, -1)
cv2.ellipse(robot_world, (246, 154), (60,30), 0, 0, 360, 255, -1)

pts_poly= np.array([[327, 236], [380, 183], [380, 128], [354, 161], [324, 156], 
                    [285, 196]], np.int32)
pts_poly = pts_poly.reshape((-1,1,2))
cv2.fillPoly(robot_world, [pts_poly], (255, 255, 255), 1)

pts_rect = np.array([[48, 191], [171, 105], [160, 90], [37, 175]], np.int32)
pts_rect = pts_rect.reshape((-1, 1, 2))
#cv2.polylines(robot_world, [pts_rect], True, 255, 1)
cv2.fillPoly(robot_world, [pts_rect], (255, 255, 255), 1)

pts_fig = np.array([[200,20], [200, 70], [230, 70], [230, 60], [210, 60], 
                    [210, 30], [230, 30], [230, 20]], np.int32)
pts_fig = pts_fig.reshape((-1, 1, 2))
cv2.fillPoly(robot_world, [pts_fig], (255, 255, 255), 1)


# Start and end point
initial_coordinate = [100, 100]
final_coordinate = [150, 150]

start_time= datetime.now() # To calculate time

# To check if the move should be made
def is_valid(c):  
    i=c[0]
    j=c[1]   
    if robot_world[i][j]== 0:
        return True
    else: 
        return False

    
# Functions for movements    
def move_up(cc):
    i=cc[0]
    j=cc[1] 
    if i > 0 and i <= 299:
        temp1 = i-1
        temp2 = j
        new_c = [temp1,temp2]
        return new_c
    return None

def move_down(cc):
    i=cc[0]
    j=cc[1]
    if i >= 0 and i < 299:
        temp1 = i+1
        temp2 = j
        new_c = [temp1, temp2]
        return new_c
    return None

def move_left(cc):
    i=cc[0]
    j=cc[1]
    if j > 0 and j <= 399:
        temp1 = i
        temp2 = j-1
        new_c = [temp1, temp2]
        return new_c
    return None

def move_right(cc):
    i=cc[0]
    j=cc[1]
    if j >= 0 and j < 399:
        temp1= i
        temp2= j+1
        new_c = [temp1,temp2]
        return new_c
    return None

def move_up_left(cc):
    i=cc[0]
    j=cc[1]
    if i > 0 and i <=299 and j > 0 and j <=399:
        temp1= i-1
        temp2= j-1
        new_c = [temp1,temp2]
        return new_c
    return None

def move_up_right(cc):
    i=cc[0]
    j=cc[1]
    if i > 0 and i <=299 and j >= 0 and j < 399:
        temp1= i-1
        temp2= j+1
        new_c = [temp1,temp2]
        return new_c
    return None

def move_down_left(cc):
    i=cc[0]
    j=cc[1]
    if i >= 0 and i < 299 and  j > 0 and j <= 399:
        temp1= i+1
        temp2= j-1
        new_c = [temp1,temp2]
        #print('down left', new_c)
        return new_c
    return None

def move_down_right(cc):
    i=cc[0]
    j=cc[1]
    if i >= 0 and i < 299 and j >= 0 and j < 399 :        
        temp1= i+1
        temp2= j+1
        new_c = [temp1,temp2]
        return new_c
    return None

child_node = []
final_node = []
robot_location = []

def end_game(n):

    my_node = Node(n)
    robot_location = n
    my_dummy_parent = Node(n)
    my_dummy_parent.add_child(my_node)
    final_node.append(my_node)
    child_node.insert(0, my_node)
    
    for item in final_node:
        
        def is_goal(location):
            if location == final_coordinate:
                print(' ')
                print('Reached the goal!!') 
                return True 
        
        robot_location = item.config
        if move_up(robot_location) != None:
            if is_valid(move_up(robot_location)) == True:
                robot_location = move_up(robot_location)
                i = robot_location[0]
                j = robot_location[1]
                robot_world[i][j] = 150
                #print('Location: ', robot_world[i][j])
                #print('loc up: ', robot_location)
                child_node.insert(0, Node(robot_location))
                final_node.append(child_node[0])
                item.add_child(child_node[0])
                
                cv2.imshow('Visual', robot_world)
                cv2.waitKey(1) 
                if is_goal(robot_location) == True:
                    return child_node[0]        
            
        robot_location = item.config
        if move_down(robot_location) != None:
            if is_valid(move_down(robot_location)) == True:
                robot_location = move_down(robot_location)
                i = robot_location[0]
                j = robot_location[1]
                robot_world[i][j] = 150
                child_node.insert(0, Node(robot_location))
                item.add_child(child_node[0])
                #robot_location = child_node[1].config
                
                cv2.imshow('Visual', robot_world)
                cv2.waitKey(1) 
                if is_goal(robot_location) == True:
                    return child_node[0]
                
        robot_location = item.config        
        if move_left(robot_location) != None:
            if is_valid(move_left(robot_location)) == True:
                robot_location = move_left(robot_location)
                i = robot_location[0]
                j = robot_location[1]
                robot_world[i][j] = 150
                child_node.insert(0, Node(robot_location))
                final_node.append(child_node[0])
                item.add_child(child_node[0])
                
                cv2.imshow('Visual', robot_world)
                cv2.waitKey(1) 
                if is_goal(robot_location) == True:
                    return child_node[0]
        
        robot_location = item.config
        if move_right(robot_location) != None:
            if is_valid(move_right(robot_location)) == True:
                robot_location = move_right(robot_location)
                i = robot_location[0]
                j = robot_location[1]
                robot_world[i][j] = 150
                child_node.insert(0, Node(robot_location))
                final_node.append(child_node[0])
                item.add_child(child_node[0])
                
                cv2.imshow('Visual', robot_world)
                cv2.waitKey(1) 
                if is_goal(robot_location) == True:
                    return child_node[0]
                
        robot_location = item.config
        if move_up_left(robot_location) != None:
            if is_valid(move_up_left(robot_location)) == True:
                robot_location = move_up_left(robot_location)
                i = robot_location[0]
                j = robot_location[1]
                robot_world[i][j] = 150
                child_node.insert(0, Node(robot_location))
                final_node.append(child_node[0])
                item.add_child(child_node[0])
                
                cv2.imshow('Visual', robot_world)
                cv2.waitKey(1) 
                if is_goal(robot_location) == True:
                    return child_node[0]
        
        robot_location = item.config
        if move_up_right(robot_location) != None:
            if is_valid(move_up_right(robot_location)) == True:
                robot_location = move_up_right(robot_location)
                i = robot_location[0]
                j = robot_location[1]
                robot_world[i][j] = 150
                child_node.insert(0, Node(robot_location))
                final_node.append(child_node[0])
                item.add_child(child_node[0])
                
                cv2.imshow('Visual', robot_world)
                cv2.waitKey(1) 
                if is_goal(robot_location) == True:
                    return child_node[0]
                
        robot_location = item.config
        if move_down_left(robot_location) != None:
            if is_valid(move_down_left(robot_location)) == True:
                robot_location = move_down_left(robot_location)
                i = robot_location[0]
                j = robot_location[1]
                robot_world[i][j] = 150
                child_node.insert(0, Node(robot_location))
                final_node.append(child_node[0])
                item.add_child(child_node[0])
                
                cv2.imshow('Visual', robot_world)
                cv2.waitKey(1)                
                if is_goal(robot_location) == True:
                    return child_node[0]
        
        robot_location = item.config        
        if move_down_right(robot_location) != None:
            #print(robot_location)
            if is_valid(move_down_right(robot_location)) == True:
                robot_location = move_down_right(robot_location)
                i = robot_location[0]
                j = robot_location[1]
                robot_world[i][j] = 150
                child_node.insert(0, Node(robot_location))
                final_node.append(child_node[0])
                item.add_child(child_node[0])  
                
                cv2.imshow('Visual', robot_world)
                cv2.waitKey(1)
                if is_goal(robot_location) == True:
                    return child_node[0]
        
 
# Saving the parent-child details         
class Node(object):
    def __init__(self, coordinate):
        self.config= coordinate
        self.children = []
        self.parent = None

    def add_child(self, obj):
        self.children.append(obj)
        obj.parent = self

goal_node = None
# To check if the Start point and the End point are not in obstacle space
if is_valid(initial_coordinate) == False or is_valid(final_coordinate)  == False:
    print("Try again")
else:
    # main function call        
    goal_node = end_game(initial_coordinate)


# Show the optimal path
temp_goal = goal_node
while temp_goal.config != initial_coordinate:
    robot_world[temp_goal.config[0]][temp_goal.config[1]] = 50
    temp_goal = temp_goal.parent
    cv2.imshow('Visual', robot_world)
    cv2.waitKey(1)
robot_world[initial_coordinate[0]][initial_coordinate[1]] = 50
  

print("Total number of nodes possible: ", len(final_node)) 
end_time = datetime.now()
time_taken = end_time - start_time
print("Time taken: ", time_taken)   

cv2.imshow('Image', robot_world)
cv2.waitKey(0)
cv2.destroyAllWindows()
