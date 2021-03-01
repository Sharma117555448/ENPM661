# -*- coding: utf-8 -*-
"""
Created on Sun Feb 18 12:54:54 2021

@author: sharma
"""

from datetime import datetime

#Declaring the Initial State of the puzzle
start_config = [['01', '06', '02', '03'], 
                ['09', '05', '07', '04'], 
                ['  ', '10', '11', '08'], 
                ['13', '14', '15', '12']]    


#Declaring the Final State of the puzzle
goal_config = [['01', '02', '03', '04'], 
               ['05', '06', '07', '08'], 
               ['09', '10', '11', '12'], 
               ['13', '14', '15', '  ']] 

print('The initial state of the puzzle is:', file=open("nodePath_5.txt", "w"))
for a in start_config:
    print(*a, file=open("nodePath_5.txt", "a"))

start_time= datetime.now() # To calculate time

#To find the location of the blank/empty tile
def find(config, number):
    loc=[[0],[0]]
    for i in range(len(config)):
        temp=config[i]
        for j in range(len(temp)):
            if temp[j]== number:
                loc[0]=i
                loc[1]=j
                return loc
            else: continue
    
location = find(start_config, '  ')
print('Location of the blank tile is:', location, file=open("nodePath_5.txt", "a"))
print(' ', file=open("nodePath_5.txt", "a"))
     
# Comparing the two matirces to see it they are equal
def compare(config1, config2):
    for i in range(0,4):
        for j in range(0,4):
            if config1[i][j] != config2[i][j]:
                return False
    return True

# Movements for displacing the blank tile
def move_up(swapped_config):    # Upward Movement
        location = find(swapped_config, '  ')
        i=location[0]
        j=location[1]
        if i-1>=0: #in range(0,3)
            swapped_config[i][j] = swapped_config[i-1][j]
            swapped_config[i-1][j]= '  '
            return swapped_config
        else: return None    

def move_down(swapped_config):  # Downward Movement
        location = find(swapped_config, '  ')
        i=location[0]
        j=location[1]
        if i+1<=3: #in range(1,4)
            swapped_config[i][j] = swapped_config[i+1][j]
            swapped_config[i+1][j]= '  '
            return swapped_config
        else: return 
       
def move_right(swapped_config):     # Movement to the Right
        location = find(swapped_config, '  ')
        i=location[0]
        j=location[1]
        if j+1 <= 3: #in range(0,3)
            swapped_config[i][j] = swapped_config[i][j+1]
            swapped_config[i][j+1]= '  '
            return swapped_config
        else: return None

def move_left(swapped_config):      # Movement to the Left
        location = find(swapped_config, '  ')
        i=location[0]
        j=location[1]
        if j-1 >= 0: #in range(1,4)
            swapped_config[i][j] = swapped_config[i][j-1]
            swapped_config[i][j-1]= '  '
            return swapped_config
        else: return None


def copy_test(stc): # Copy function to avoid deep copy
    con= [['', '', '', ''],
          ['', '', '', ''], 
          ['', '', '', ''],
          ['', '', '', '']]
    for row in range (0,4):
        for col in range(0,4):
            con[row][col]= stc[row][col]
    return con  

final_node = []
child_node = []


def endgame(s):         # Performing the main function
    
    my_node= Node(s)
    my_dummy_parent = Node(s)
    my_dummy_parent.add_child(my_node)
    final_node.append(my_node)
    child_node.insert(0, my_node)
        
    for item in final_node:
        if move_up(copy_test(item.config)) != None:
            #print (sep='\n', *move_up(copy_test(item.config)))
            if compare(item.parent.config , move_up(copy_test(item.config))) == False: 
                if check_if_exists(move_up(copy_test(item.config))) == False:
                    child_node.insert(0, Node(move_up(copy_test(item.config))))
                    final_node.append(child_node[0])
                    item.add_child(child_node[0])
                    if compare(move_up(copy_test(item.config)), goal_config) == True:
                        return child_node[0]
        
        if move_left(copy_test(item.config)) != None:
            #print (sep='\n', *move_left(copy_test(item.config)))
            if compare(item.parent.config , move_left(copy_test(item.config))) == False:
                if check_if_exists(move_left(copy_test(item.config))) == False:
                    child_node.insert(0, Node(move_left(copy_test(item.config))))
                    final_node.append(child_node[0])
                    item.add_child(child_node[0])
                    if compare(move_left(copy_test(item.config)), goal_config) == True:
                        return child_node[0]
          
        if move_right(copy_test(item.config)) != None:
            #print (sep='\n', *move_right(copy_test(item.config)))
            if compare(item.parent.config , move_right(copy_test(item.config))) == False:
                if check_if_exists(move_right(copy_test(item.config))) == False:
                    child_node.insert(0, Node(move_right(copy_test(item.config))))
                    final_node.append(child_node[0])
                    item.add_child(child_node[0])
                    if compare(move_right(copy_test(item.config)), goal_config) == True:
                        return child_node[0]
       
        if move_down(copy_test(item.config)) != None:
            #print (sep='\n', *move_down(copy_test(item.config)))
            if compare(item.parent.config , move_down(copy_test(item.config))) == False:
                if check_if_exists(move_down(copy_test(item.config))) == False:
                    child_node.insert(0, Node(move_down(copy_test(item.config))))
                    final_node.append(child_node[0])
                    item.add_child(child_node[0])
                    if compare(move_down(copy_test(item.config)), goal_config) == True:
                        return child_node[0]

        child_node.pop()
  

# Saving the parent-child details        
class Node(object):
    def __init__(self, config):
        self.config= config
        self.children = []
        self.parent = None

    def add_child(self, obj):
        self.children.append(obj)
        obj.parent = self
        
# Checking if the current configuration exists        
def check_if_exists(con):
    for elements in child_node:
        if compare(elements.config, con) == True:
            return True
    return False

# main function call
goal_node= endgame(start_config)

# Printing the solution sequence
temp_goal = goal_node
print("The Solution sequence would be-", file=open("nodePath_5.txt", "a"))
while compare(temp_goal.config, start_config) == False:
    print(temp_goal.config,  file=open("nodePath_5.txt", "a"))
    temp_goal = temp_goal.parent
print(start_config,  file=open("nodePath_5.txt", "a"))


print("Total number of nodes possible: ", len(final_node),  file=open("nodePath_5.txt", "a"))
end_time = datetime.now()
time_taken = end_time - start_time
print("Time taken: ", time_taken,  file=open("nodePath_5.txt", "a")) # Printing the time taken