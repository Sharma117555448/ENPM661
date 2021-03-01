-----ReadME File-----

I have attached 5 different files to be executed for the different test cases. So just running them should work.
Given below is the basic logic of the program i have worked with. 
This program is written in Python 3.0 and worked over on Spyder.


The code written deals with moving the blank space between the different spaces and get to the 
required solution of the sequence for the puzzel- 1 2 3 45 6 7 89 10 11 12 13 14 15 0.

Did not need to work with any library explicitely, but adopted the DATETIME library to find the time taken to 
execute the program.

Program includes declaring the initial and the goal configurations of the puzzel and performing the functions
until the goal configuration is reached.

Second step is to find the location of the blank tile, so we know where the configurations can vary.

Next are the functions which deals with the actual movement of the blank tiles. These functions are called 
from the main function.

Last, these movements lead to new sequences of operations which are saved into the nodes with the parent-child
correlations. If the existing configuration comes up, it is avoided and the cycle is repeated until we get the
goal configurations, which when achieved is the solution sequence we need. 

Finally we can get the total nodes and the time taken to solve the program