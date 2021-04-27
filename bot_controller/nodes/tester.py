from matplotlib import pyplot as plt
import pygame
import time
from pygame_colliders import ConcaveCollider, ConvexCollider
from abc import ABCMeta, abstractmethod
from heapq import heappush, heappop, heapify
import sys, random, math, pygame
from pygame.locals import *
from math import sqrt,cos,sin,atan2

from math import pi
from math import sqrt,cos,sin,atan2
import numpy 


# rad= int(input("Radius:"))
# clear = int(input("Clearance:"))
# initial_coordinate = input("Enter initial x and y coordinates:")
# initial_coordinate = initial_coordinate.split()
# for i in range(len(initial_coordinate)):
#     initial_coordinate[i] = int(initial_coordinate[i])
# final_coordinate = input("Enter final x and y coordinates:")
# final_coordinate = final_coordinate.split()
# for i in range(len(final_coordinate)):
#     final_coordinate[i] = int(final_coordinate[i])
# initial_orientation = int(input("Initial Orientation:"))
# step = int(input("Step size:"))

rad = 4
clear = 2
step = 20
initial_orientation = 10
initial_coordinate = [9, 9] 
final_coordinate = [70, 70]
moves_list_ros = []

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
gameDisplay = pygame.display.set_mode((400,300))
gameDisplay.fill(black)
pygame.draw.polygon(gameDisplay, green, ((48,191),(36,175),(159,90),(171,105)))
pygame.draw.circle(gameDisplay, green, (90,299-70), 35)
pygame.draw.ellipse(gameDisplay, green, [186, 300-175, 120, 60])
pygame.draw.polygon(gameDisplay, green, ((200,70),(200,20),(230,20),(230,30),(210,30),(210,60),(230,60),(230,70)))

cyan = 0,255,255
collider_a_points = [(48,191),(36,175),(159,90),(171,105)]
collider_c_points = [(200,70),(200,20),(230,20),(230,30),(210,30),(210,60),(230,60),(230,70)]

collider_a = ConcaveCollider(collider_a_points)
collider_c = ConcaveCollider(collider_c_points)


def collides_pol(a,b):
    
    collider_b_points = [(a+rad/2 + clear,b+rad/2 + clear)]
    collider_b = ConvexCollider(collider_b_points)

    if collider_a.collide(collider_b) or collider_c.collide(collider_b):
        print("Collision detected")
        return True
    return False


def collides_circle(a,b):
    x2, y2 = 90, 299-70
    distance = math.hypot(a - x2, b - y2)+rad/2 + clear
    if distance <= 35:
        return True
    return False


def collides_ell(a,b):
    ze, ye = 60, 30 
    scale_y = ze/ye
    cntrx, cntry = 246,299-145
    dx = a - cntrx
    dy = (b - cntry)*scale_y
    distance = dx*dx + dy*dy
    if distance+rad/2 + clear <= ze*ze:
        return True
    return False


pygame.display.flip()


def reset():
    global count
    screen.fill(black)
    init_obstacles(GAME_LEVEL)
    count = 0

if collides_pol(initial_coordinate[0],initial_coordinate[1]) or collides_circle(initial_coordinate[0],initial_coordinate[1]) or collides_ell(initial_coordinate[0],initial_coordinate[1]):
    print("Start is in obstacle")
    quit()
if collides_pol(final_coordinate[0],final_coordinate[1]) or collides_circle(final_coordinate[0],final_coordinate[1]) or collides_ell(final_coordinate[0],final_coordinate[1]):
    print("Goal is in obstacle")
    quit()


def move_max(cc, orientation, rad=1.0, clear=3.5, step_size=1.0):
    start_x = cc[0]
    start_y = cc[1]

    move_x = 0
    move_y = 0
    
    start_rad_orientation = (orientation*pi)/180
    start_rad_orientation_step = 30
    rad_orientation = start_rad_orientation + (start_rad_orientation_step*2*pi)/180

    dx = math.cos(rad_orientation)*step_size 
    dy = math.sin(rad_orientation)*step_size 
    
    # move_x = move_x + dx
    # move_y = move_y + dy
    
    new_x = start_x + dx
    new_y = start_y + dy

    new_angle_deg = rad_orientation*180/pi
    if new_angle_deg > 360:
        new_angle_deg = abs(new_angle_deg - 360)
    new_node = (new_x, new_y, new_angle_deg)
    curr_node = (cc[0],cc[1], orientation)

    if 0.00 <= new_node[0] <= 400.00 and 0.00 <= new_node[1] <= 300.00 and collides_pol(new_node[0],new_node[1]) == False and collides_circle(new_node[0],new_node[1]) == False and collides_ell(new_node[0],new_node[1]) == False:
        return new_node
    else:
        return curr_node
def move_30(cc, orientation, rad=1.0, clear=3.5, step_size=1.0):
    start_x = cc[0]
    start_y = cc[1]

    move_x = 0
    move_y = 0

    start_rad_orientation = (orientation*pi)/180
    start_rad_orientation_step = 30
    rad_orientation = start_rad_orientation + (start_rad_orientation_step*pi)/180

    dx = math.cos(rad_orientation)*step_size
    dy = math.sin(rad_orientation)*step_size
    
    # move_x = move_x + dx
    # move_y = move_y + dy
    
    new_x = start_x + dx
    new_y = start_y + dy

    new_angle_deg = rad_orientation*180/pi
    if new_angle_deg > 360:
        new_angle_deg = abs(new_angle_deg - 360)
    new_node = (new_x, new_y, new_angle_deg)
    curr_node = (cc[0],cc[1], orientation)

    if 0.00 <= new_node[0] <= 400.00 and 0.00 <= new_node[1] <= 300.00 and collides_pol(new_node[0],new_node[1]) == False and collides_circle(new_node[0],new_node[1]) == False and collides_ell(new_node[0],new_node[1]) == False:
        return new_node
    else:
        return curr_node
def move_neg30(cc, orientation, rad=1.0, clear=3.5, step_size=1.0):
    start_x = cc[0]
    start_y = cc[1]

    move_x = 0
    move_y = 0

    start_rad_orientation = (orientation*pi)/180
    start_rad_orientation_step = 30
    rad_orientation = start_rad_orientation - (start_rad_orientation_step*pi)/180

    dx = math.cos(rad_orientation)*step_size
    dy = math.sin(rad_orientation)*step_size
    
    new_x = start_x + dx
    new_y = start_y + dy

    new_angle_deg = rad_orientation*180/pi
    if new_angle_deg > 360:
        new_angle_deg = abs(new_angle_deg - 360)
    new_node = (new_x, new_y, new_angle_deg)
    curr_node = (cc[0],cc[1], orientation)

    if 0.00 <= new_node[0] <= 400.00 and 0.00 <= new_node[1] <= 300.00 and collides_pol(new_node[0],new_node[1]) == False and collides_circle(new_node[0],new_node[1]) == False and collides_ell(new_node[0],new_node[1]) == False:
        return new_node
    else:
        return curr_node
def move_0(cc, orientation, rad=1.0, clear=3.5, step_size=1.0):
    start_x = cc[0]
    start_y = cc[1]

    move_x = 0
    move_y = 0

    rad_orientation = (orientation*pi)/180
    start_rad_orientation_step = 30

    dx = math.cos(rad_orientation)*step_size
    dy = math.sin(rad_orientation)*step_size

    new_x = start_x + dx
    new_y = start_y + dy

    new_angle_deg = rad_orientation*180/pi
    if new_angle_deg > 360:
        new_angle_deg = abs(new_angle_deg - 360)
    new_node = (new_x, new_y, new_angle_deg)
    curr_node = (cc[0],cc[1], orientation)

    if 0.00 <= new_node[0] <= 400.00 and 0.00 <= new_node[1] <= 300.00 and collides_pol(new_node[0],new_node[1]) == False and collides_circle(new_node[0],new_node[1]) == False and collides_ell(new_node[0],new_node[1]) == False:
        return new_node
    else:
        return curr_node      
def move_min(cc, orientation, rad=1.0, clear=3.5, step_size=1.0):
    start_x = cc[0]
    start_y = cc[1]

    move_x = 0
    move_y = 0

    start_rad_orientation = (orientation*pi)/180
    start_rad_orientation_step = 30
    rad_orientation = start_rad_orientation - (start_rad_orientation_step*2*pi)/180
    
    dx = math.cos(rad_orientation)*step_size
    dy = math.sin(rad_orientation)*step_size
    
    new_x = start_x + dx
    new_y = start_y + dy

    new_angle_deg = rad_orientation*180/pi
    if new_angle_deg > 360:
        new_angle_deg = abs(new_angle_deg - 360)
    new_node = (new_x, new_y, new_angle_deg)
    curr_node = (cc[0],cc[1], orientation)

    if 0.00 <= new_node[0] <= 400.00 and 0.00 <= new_node[1] <= 300.00 and collides_pol(new_node[0],new_node[1]) == False and collides_circle(new_node[0],new_node[1]) == False and collides_ell(new_node[0],new_node[1]) == False:
        return new_node
    else:
        return curr_node     


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
        __slots__ = ('cnode', 'angle', 'g', 'f',
                     'closed', 'parent', 'open')
        def __init__(self, cnode, angle, g=1000000, f=1000000):  #Set to actual hypotenuse value for g 
            self.cnode = cnode
            self.angle = angle
            self.g = g
            self.f = f
            self.closed = False
            self.open = True
            self.parent = None
        def __lt__(self, a):
            return self.f < a.f


    class SearchNodeDict(dict):
        def __missing__(self, cc):
            n = Alg.SearchNode(cc)
            self.__setitem__(cc, n)
            return n


    def path_plan(self, open_list, goal_node, reverseFound=False):
        empty_list = []
        vector_list = []
            
        current_node = goal_node

        while current_node.cnode != initial_coordinate:
            child= current_node.cnode 

            vec = current_node.cnode+[current_node.angle]
            vector_list.append(vec)

            empty_list.append(current_node.cnode)
            current_node = current_node.parent

            if len(empty_list) > 0:
                X1 = current_node.cnode[0]
                Y1= current_node.cnode[1]
                U1 = child[0]
                V1 = child[1]
                #Divyum, you might need to swap these around to make the arrow point in the right direction. Can't get matplot lib to behave

            q1 = plt.quiver(X1, Y1, U1, V1,units='xy' ,scale=1) 
            print(current_node.cnode,child)
            moves_list_ros.append([current_node.cnode,child])
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
        plt.xlim(0, 400)
        plt.ylim(0, 300)

        plt.title('Plot the vector in matplotlib',fontsize=10)
        plt.savefig('how_to_plot_a_vector_in_matplotlib_fig3.png', bbox_inches='tight')

        plt.show()
        plt.close()
        
        fig, ax = plt.subplots()

    def end_game2 (self,start_coordinate, orien, goal,radius,clearance,step_size):
        
        def round_of_rating(cc):
            #round(number * 2) / 2
            cc[0] = round(cc[0] * 2) / 2 
            cc[1] = round(cc[1] * 2) / 2
            cc[2] = round(cc[2] *2)/2
            return cc
            
        visited = numpy.zeros((601,801,13))

        def isGoal(loc):
            if dist(loc,final_coordinate) <= 1.5 + +radius/2 + clearance:
                print("\nReached the Goal!")
                return loc
                

            else:
                return None
        if isGoal(start_coordinate):
            return [start_coordinate]
            
        initSearch = Alg.SearchNodeDict() #Start "list"
        
        startNode = Alg.SearchNode(start_coordinate,orien,g=.0,f=dist(start_coordinate,final_coordinate))
        test_list = []
        open_list = []
        heappush(open_list,startNode)
        
        while len(open_list) > 0: 
            currentCoord = heappop(open_list)
            moves_list = []
            if isGoal(currentCoord.cnode):
                blah2 = self.path_plan(open_list, currentCoord, reverseFound=False)
                
                # pygame.draw.line(gameDisplay,white,final_arr.parent,final_arr.cnode)
                # pygame.display.update()
                # fpsClock.tick(10000)
                return True

            currentCoord.open = True
            currentCoord.closed = True
            
            temp_array = [currentCoord.cnode[0],currentCoord.cnode[1],currentCoord.angle]
            temp_array_2 = round_of_rating(temp_array)
            visited[int(temp_array_2[0]*2),int(temp_array_2[1]*2),int(temp_array_2[2]/30)] = 1

            if move_max(currentCoord.cnode,currentCoord.angle,radius,clearance,step_size) != None:
                moveone = round_of_rating(list(move_max(currentCoord.cnode,currentCoord.angle,radius,clearance,step_size)))
            if move_30(currentCoord.cnode,currentCoord.angle,radius,clearance,step_size) != None:
                movetwo = round_of_rating(list(move_30(currentCoord.cnode,currentCoord.angle,radius,clearance,step_size)))
            if move_0(currentCoord.cnode,currentCoord.angle,radius,clearance,step_size) != None:
                movethree = round_of_rating(list(move_0(currentCoord.cnode,currentCoord.angle,radius,clearance,step_size)))
            if move_neg30(currentCoord.cnode,currentCoord.angle,radius,clearance,step_size)!= None:
                movefour = round_of_rating(list(move_neg30(currentCoord.cnode,currentCoord.angle,radius,clearance,step_size)))
            if move_min(currentCoord.cnode,currentCoord.angle,radius,clearance,step_size) != None:
                movefive = round_of_rating(list(move_min(currentCoord.cnode,currentCoord.angle,radius,clearance,step_size)))
            
            if visited[int(moveone[0]*2),int(moveone[1]*2),int(moveone[2]/30)] == 0: 
                moves_list.append(move_max(currentCoord.cnode,currentCoord.angle,radius,clearance,step_size))
            if visited[int(movetwo[0]*2),int(movetwo[1]*2),int(movetwo[2]/30)] == 0:
                moves_list.append(move_30(currentCoord.cnode,currentCoord.angle,radius,clearance,step_size))
            if visited[int(movethree[0]*2),int(movethree[1]*2),int(movethree[2]/30)] == 0:
                moves_list.append(move_0(currentCoord.cnode,currentCoord.angle,radius,clearance,step_size))
            if visited[int(movefour[0]*2),int(movefour[1]*2),int(movefour[2]/30)] == 0:
                moves_list.append(move_neg30(currentCoord.cnode,currentCoord.angle,radius,clearance,step_size))
            if visited[int(movefive[0]*2),int(movefive[1]*2),int(movefive[2]/30)] == 0:
                moves_list.append(move_min(currentCoord.cnode,currentCoord.angle,radius,clearance,step_size))

            for item in moves_list:
  
                junk_coord = [item[0],item[1]]
                stuff = Alg.SearchNode(junk_coord,item[2],g=100000,f=dist(currentCoord.cnode,final_coordinate)) #Might need to change to g = 10000

                junk_list = stuff.cnode
                junk_list = junk_list + [stuff.angle]

                if visited[int(round_of_rating(list(junk_list))[0]*2),int(round_of_rating(list(junk_list))[1]*2),int(round_of_rating(list(junk_list))[2]/30)] == 1:
                    continue 
                if stuff.closed:
                    continue
                test_cost = currentCoord.g + dist(currentCoord.cnode, junk_coord)
                if test_cost >= stuff.g:
                    continue 

                stuff.parent = currentCoord  
                
               
                stuff.g = test_cost
                stuff.f = test_cost + dist(stuff.cnode, final_coordinate)
                if stuff.open:
                    stuff.open = False

                    heappush(open_list,stuff)
                    visited[int(round_of_rating(list(item))[0]*2),int(round_of_rating(list(item))[1]*2),int(round_of_rating(list(item))[2]/30)] = 1
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
