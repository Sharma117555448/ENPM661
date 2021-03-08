------ Project 2 ----------
ENPM661
Charu Sharma
117555448
charu107@umd.edu
---------------------------

2 different files have been made with 2 differend Start and End positions. When choosen to work with the pair 
of initial coordinate as (120, 320) and final coordinate as (200, 350) the code was executed in 3 minutes. 
Similaryly, for initial coordinate (100, 100) and final coordinate (150, 150) the code was executed in 6 minutes.

This program is written in Python 3.0 and worked over on Spyder. Given below is the basic logic of the program 
I have worked with:
The requirement was to move the point sized robot from initial coordinate to end goal coordinate, and I had 
taken up BFS search to acheive that.

Libraries adopted in this case were- cv2, numpy and DATETIME. cv2 and numpy libraries were useful to interact 
with the openCV where we had declared the workspace, or map. DATETIME library to find the time taken to 
execute the program.

I mapped the workspace using the drawing feature provided through cv2 library. I was having issues with the 
other method so I incorporated this to get the layout. Coordinates with determined with trigonometric functions.
Given the map as the color black (0), and the obstacle as white (255). While getting the solution, the movements
were determined with gray (150) and optimum path represented with gray (50).

Second, comapred if the start or the goal coordinate was under the obstacle space, and terminated the code immediately.

Next are the functions which deals with the actual movement of the blank tiles. These functions are called 
from the main function. Edge conditons were also incorporated so as to get efficient results.

Last, these movements lead to new sequences of operations which are saved into the nodes with the parent-child
correlations. If the obstacle cameup, it wasavoided and the cycle is repeated until we get the
goal coordinate,which when achieved is the solution sequence we need. 

Finally we can get the total nodes and the time taken to solve the program.