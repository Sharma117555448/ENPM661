------ Project 3 : Phase 2 ----------
ENPM661				
Charu Sharma			John-Edward Draganov
117555448			113764875
charu107@umd.edu		jdragano@umd.edu
-------------------------------------
python: 3.0

Only 1 file has been uploaded for the test cases as the initial and goal coordinates are to be provided as 
inputs. Along with this we have provided with the video of the output. 
Given below is the basic logic of the program:
The requirement was to move a robot with raduis 10 from initial coordinate to end goal coordinate avoiding 
the obstacle with a clearance of 5, and had to use A* Algorithm to acheive the optimum path.

Libraries adopted: pygame, numpy, math, matplotlib, time, pygame_colliders, pygame.locals, RRT_includes and 
heapq. 
We used pygame to declared the workspace and the obstacles. The movement of the robot was also observed
the secondary modules of pygame.
math library was used to calculate mathematical functions in the program such wqrt, sin, cos, athan2.
DATETIME library to find the time taken to execute the program.

We mapped the workspace using the feature provided through the pygame module. We were having issues with the 
other method so we incorporated this to get the layout. Coordinates with determined with trigonometric functions.
Given the map as the color black (0,0,0), and the obstacle as green(0,255,0). While getting the solution, we drew 
the pathway with blue (0,0,255). Finally the pathway can be seen with white (255, 255, 255) line.

Secondly, compared if the robot was not in the obstacle space or had enough clearance. Also, checked if the 
start or the goal coordinates were under the obstacle space, and terminated the code immediately either case. 

Next are the functions which deals with the actual movement of the robot. These functions are called 
from the main function. Edge conditons and the clearance conditions were also incorporated so as to get 
efficient results. 

A* Algorithm was incorporated to get optimum path. These movements lead to new sequences of operations 
which are saved into the nodes with the parent-child correlations. If the obstacle came up, it was avoided by 
the pygame colliders module and the cycle is repeated until we get the goal region, which when achieved is 
the solution sequence we need. And last we plot the vectors to show the path and orientation.

Finally we can get the total nodes and the time taken to solve the program.