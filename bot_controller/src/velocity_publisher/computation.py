# -*- coding: utf-8 -*-
"""
Created on Thu Apr 22 21:20:14 2021

@author: sharm
"""

from matplotlib import pyplot as plt
import pygame
import time
from pygame_colliders import ConcaveCollider, ConvexCollider
from abc import ABCMeta, abstractmethod
from heapq import heappush, heappop, heapify
import sys, random, math, pygame
from pygame.locals import *
from math import pi,sqrt,cos,sin,atan2
#from RRT_includes import *
import numpy 

'''
rad= int(input("Radius:"))
clear = int(input("Clearance:"))
initial_coordinate = input("Enter initial x and y coordinates:")
initial_coordinate = initial_coordinate.split()
for i in range(len(initial_coordinate)):
    initial_coordinate[i] = int(initial_coordinate[i])
final_coordinate = input("Enter final x and y coordinates:")
final_coordinate = final_coordinate.split()
for i in range(len(final_coordinate)):
    final_coordinate[i] = int(final_coordinate[i])
initial_orientation = int(input("Initial Orientation:"))
step = int(input("Step size:"))'''

rad = 1
clear = 20.6
step = 10
initial_orientation = 0
initial_coordinate = [700, 700] 
final_coordinate = [900, 900]
time_run =1
RPM_1 = 10
RPM_2 = 25



if initial_coordinate[0] == final_coordinate[0]+rad/2 + clear and initial_coordinate[1] == final_coordinate[1]+rad/2 + clear:
  print('Can not start at goal')
  quit()

pygame.init()

white = (255,255,255)
black = (0,0,0)


red = (255,0,0)
green = (0,255,0)
blue = (0,0,255)
fpsClock = pygame.time.Clock()
gameDisplay = pygame.display.set_mode((1001,1001))
gameDisplay.fill(black)

pygame.draw.circle(gameDisplay, green, (200,200), 100)
pygame.draw.circle(gameDisplay, green, (800,200), 100)

sus1 = pygame.draw.rect(gameDisplay, green, (425,25,150,150))
sus2 = pygame.draw.rect(gameDisplay, green, (425,375,150,250))
sus3 = pygame.draw.rect(gameDisplay, green, (200,725,200,150))


cyan = 0,255,255
# collider_a_points = [(48,191),(36,175),(159,90),(171,105)]
# collider_c_points = [(200,70),(200,20),(230,20),(230,30),(210,30),(210,60),(230,60),(230,70)]

# collider_a = ConcaveCollider(collider_a_points)
# collider_c = ConcaveCollider(collider_c_points)


# def collides_pol(a,b):
    
#     collider_b_points = [(a + rad/2 + clear, b + rad/2 + clear)]
#     collider_b = ConvexCollider(collider_b_points)

#     if collider_a.collide(collider_b) or collider_c.collide(collider_b):
#         print("Collision detected")
#         return True
#     return False

moves_list_ros = []

def collides_circle(a,b):
    x2, y2 = 200, 200
    distance = math.hypot(a - x2, b - y2)+rad/2 + clear
    if distance <= 0.5:
        print(distance)
        return True
    return False


def collides_ell(a,b):
    x2, y2 = 800, 200
    distance = math.hypot(a - x2, b - y2)+rad/2 + clear
    if distance <= 0.5:
        return True
    return False


pygame.display.flip()


def reset():
    global count
    screen.fill(black)
    init_obstacles(GAME_LEVEL)
    count = 0

if sus1.collidepoint(initial_coordinate[0],initial_coordinate[1]) or sus2.collidepoint(initial_coordinate[0],initial_coordinate[1]) or sus2.collidepoint(initial_coordinate[0],initial_coordinate[1]) or collides_circle(initial_coordinate[0],initial_coordinate[1]) or collides_ell(initial_coordinate[0],initial_coordinate[1]):
    print("Start is in obstacle")
    quit()
if sus1.collidepoint(initial_coordinate[0],initial_coordinate[1]) or sus2.collidepoint(initial_coordinate[0],initial_coordinate[1]) or sus2.collidepoint(initial_coordinate[0],initial_coordinate[1]) or collides_circle(final_coordinate[0],final_coordinate[1]) or collides_ell(final_coordinate[0],final_coordinate[1]):
    print("Goal is in obstacle")
    quit()


def Round2Point5(num):
    return (round(num * 2) / 2)

def RoundTo15(x, base=15):
    return base * round(x / base)
 

def move_new (cc, angle, rpm, rad=1.0, clear=3.5, step_size=1.0):
    global time_run   
                                  
    start_x = Round2Point5(cc[0])
    start_y = Round2Point5(cc[1])
    orientation = RoundTo15(angle)
    
    r = rad/2
    RPM_L = rpm[0]
    RPM_R = rpm[1]
    
    t=0
    dt = 0.1
    move_x = 0
    move_y = 0
    dtheta = 0
    
    start_rad_orientation = math.pi * orientation / 180
    while t < time_run:
        t = t + dt
        start_x = start_x + move_x
        start_y = start_y + move_y
        
        dx = r * (RPM_L + RPM_R) * math.cos(start_rad_orientation) * dt
        dy = r * (RPM_L + RPM_R) * math.sin(start_rad_orientation) * dt
        dtheta = ( r / clear) * (RPM_R - RPM_L) * dt
        
        move_x = move_x + dx
        move_y = move_y + dy
        start_rad_orientation = start_rad_orientation + (0.5 * dtheta)
        
        new_x = start_x + move_x
        new_y = start_y + move_y
        new_final_orientation = 180 * start_rad_orientation / math.pi
        
    degrees_rotated = abs(orientation - new_final_orientation)
    degrees_rotated = abs(degrees_rotated % 360)
    cost = 10 + (10 * degrees_rotated / 360)
    #new_final_orientation = round(new_final_orientation,2)
    new_angle = RoundTo15(new_final_orientation)
    new_data = ((Round2Point5(new_x), Round2Point5(new_y)), new_angle, cost)
    curr_data= ((cc[0], cc[1]), orientation, 1000000)
                              
    if 0.00 <= new_data[0][0] <= 1000.00 and 0.00 <= new_data[0][1] <= 1000.00 and sus1.collidepoint(Round2Point5(new_data[0][0]),Round2Point5(new_data[0][1])) == False and sus2.collidepoint(Round2Point5(new_data[0][0]),Round2Point5(new_data[0][1])) == False and sus3.collidepoint(Round2Point5(new_data[0][0]),Round2Point5(new_data[0][1])) == False and collides_circle(Round2Point5(new_data[0][0]),Round2Point5(new_data[0][1])) == False and collides_ell(Round2Point5(new_data[0][0]),Round2Point5(new_data[0][1])) == False:
        return new_data, True
    else:
        return curr_data, False
    

def dist(p1,p2):
    return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))
# global step 
# step = 30


class Alg:
    __metaclass__ = ABCMeta
    __slots__ = ()

    class NodeFind(object):
        def __init__(self, coordinate):
            self.config = coordinate
            self.children = []
            self.dad = None

        def add_child(self, obj):
            self.children.append(obj)
            obj.dad = self
            
    class SearchNode:
        __slots__ = ('cnode', 'angle','g', 'f',
                     'closed', 'open', 'parent','rpm')
        def __init__(self, cnode, angle, g=1000000, f=1000000,rpm = RPM_1):  #Set to actual hypotenuse value for g 
            self.cnode = cnode
            self.angle = angle
            self.g = g
            self.f = f
            self.rpm = rpm
            self.closed = False
            self.open = True
            self.parent = None

        def __lt__(self, a):
            return self.f < a.f




    def path_plan(self, open_list, goal_node, reverseFound=False):
        empty_list = []
        vector_list = []
        rpm_list = []
        rpm_list_empty = []
        rpm_list_fin = []
        current_node = goal_node

        while current_node.cnode != initial_coordinate:
            child= current_node.cnode 

            child_rpm = current_node.rpm

            vec = current_node.cnode+[current_node.angle]

            rpm = current_node.rpm

            vector_list.append(vec)

            rpm_list.append(rpm)

            empty_list.append(current_node.cnode)
            rpm_list_empty.append(current_node.rpm)
            current_node = current_node.parent

            if len(empty_list) > 0:
                X1 = current_node.cnode[0]
                Y1= current_node.cnode[1]
                U1 = child[0]
                V1 = child[1]
              
            q1 = plt.quiver(X1, Y1, U1, V1,units='xy' ,scale=1) 
            rpm_list_fin.extend([current_node.rpm,rpm])
            moves_list_ros.extend([current_node.cnode,child])

            pygame.draw.line(gameDisplay,white,current_node.cnode,child)
            pygame.display.update()
            fpsClock.tick(60)

        X0 =empty_list[-2][0]
        Y0= empty_list[-2][1]
        U0 = empty_list[-1][0]
        V0 = empty_list[-1][1]
        
        fig, ax = plt.subplots()
        q0 = plt.quiver(X0, Y0, U0, V0,units='xy' ,scale=1,color= 'r',headwidth = 1,headlength=0)

        plt.grid()
        ax.set_aspect('equal')
        plt.xlim(0, 1000)
        plt.ylim(0, 1000)

        plt.title('Plot the vector in matplotlib',fontsize=10)
        plt.savefig('how_to_plot_a_vector_in_matplotlib_fig3.png', bbox_inches='tight')

        plt.show()
        plt.close()
        rpm_list_fin.reverse()
        rpm_list_fin1 = rpm_list_fin[1:-1]
        
        fig, ax = plt.subplots()
        return rpm_list_fin

    def end_game2 (self, start_coordinate, orien, goal,radius,clearance,step_size):
        
        def round_of_rating(cc):
            #round(number * 2) / 2
            cc[0] = round(cc[0] * 2) / 2 
            cc[1] = round(cc[1] * 2) / 2
            cc[2] = round(cc[2] *2)/2
            return cc
            
        visited = numpy.zeros((2001,2001,16))

        def isGoal(loc):
            if dist(loc,final_coordinate) <= 1.5 + +radius/2 + clearance:
                print("\nReached the Goal!")
                return loc
                

            else:
                return None
        if isGoal(start_coordinate):
            return [start_coordinate]
            

        startNode = Alg.SearchNode(start_coordinate, orien, g=.0, f=dist(start_coordinate,final_coordinate),rpm = [0,0])
        test_list = []
        open_list = []
        heappush(open_list,startNode)
        moves_list = []
        
        actions = [[0, RPM_1],
           [RPM_1, 0],
           [RPM_1, RPM_1],
           [0, RPM_2],
           [RPM_2, 0],
           [RPM_2, RPM_2],
           [RPM_1, RPM_2],
           [RPM_2, RPM_1]]

        while len(open_list) > 0: 
            currentCoord = heappop(open_list)
            if isGoal(currentCoord.cnode):
                blah2 = self.path_plan(open_list, currentCoord, reverseFound=False)
                print(blah2)
                # pygame.draw.line(gameDisplay,white,final_arr.parent,final_arr.cnode)
                # pygame.display.update()
                # fpsClock.tick(10000)
                return True

            currentCoord.open = True
            currentCoord.closed = True
            
            temp_array = [currentCoord.cnode[0],currentCoord.cnode[1],currentCoord.angle]
            temp_array_2 = round_of_rating(temp_array)
            visited[int(temp_array_2[0]*2),int(temp_array_2[1]*2),int(temp_array_2[2]/30)] = 1
            
            for rpm in actions:
                if move_new(currentCoord.cnode,currentCoord.angle,rpm, radius,clearance,step_size) != None:
                    movefive = move_new(currentCoord.cnode, currentCoord.angle, rpm ,radius,clearance,step_size)
                 

                #print('3', movefive[0][1]/30)
                #print('exe', movefive[0][1])
                # print('tet', movefive[0][0][0]*2)
                print('blah',movefive)
                if visited[int(movefive[0][0][0]*2),int(movefive[0][0][1]*2),int(movefive[0][1]/30)] == 0:
                    moves_list.append(move_new(currentCoord.cnode,currentCoord.angle,rpm,radius,clearance,step_size))
                
                    
                for item in moves_list:
                    temp_coord = [item[0][0][0],item[0][0][1]]
                    stuff = Alg.SearchNode(temp_coord,item[0][1],g=100000,f=dist(currentCoord.cnode,final_coordinate),rpm = rpm) #Might need to change to g = 10000

                    temp_list = stuff.cnode
                    temp_list = temp_list + [stuff.angle] + [rpm[0]] + [rpm[1]]
                    print(rpm[0],rpm[1])
    
                    if visited[int(Round2Point5(temp_list[0]*2)),int(Round2Point5(temp_list[1]*2)),int(RoundTo15(temp_list[2]/30))] == 1:
                        continue 
                    if stuff.closed:
                        continue
                    test_cost = currentCoord.g + dist(currentCoord.cnode, temp_coord)
                    if test_cost >= stuff.g:
                        continue 
    
                    stuff.parent = currentCoord  
                    
                   
                    stuff.g = test_cost
                    stuff.f = test_cost + dist(stuff.cnode, final_coordinate)
                    if stuff.open:
                        stuff.open = False
    
                        heappush(open_list,stuff)

                        visited[int(Round2Point5(item[0][0][0]*2)),int(Round2Point5(item[0][0][1]*2)),int(RoundTo15(item[0][1]/30))] = 1
                    else: 
                        open_list.remove(stuff)
                        heappush(open_list,stuff)
    
                    pygame.draw.line(gameDisplay,cyan,stuff.parent.cnode,stuff.cnode)
    
    
                    pygame.display.update()
                    fpsClock.tick(60)

        return None 


if __name__ == '__main__':
                
    blah = Alg()
    blah.end_game2(initial_coordinate, 30, final_coordinate,rad,clear,step)
    