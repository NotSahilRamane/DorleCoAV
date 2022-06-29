Importing the required libraries
We need numpy and pandas

Importing csv files
We import the csv file downloaded from mymaps.google.com as it is without making any changes. We manually make the connectivity matrix and edge matrix If the total number of nodes is 'n' then - Connectivity matrix : dimension = n x n - Element (i, j) that is in row 'i' and column 'j' is 1 if there is a direct road from i to j and 0 otherwise Note - There can be a road from node i to j but there may not be a road from j to i
edge_matrix : dimension = n x n - The nature of the matrix is same as the connectivity matrix. Only that in place of 1 at (i, j) we have the 'weight' = road length between 'i' and 'j' nodes.
Connectivity matrix and edge matrix are created manually in excel. Distances for the edge matrix are currently taken from mymaps.google.com by drawing lines between nodes on our map.

Cleaning the imported Data
From the imported csv file for node locations (latitude and longitude), we clear the letters and symbols (brackets, words etc.) and take only those elements which refer to the points on the map.
Change '10' on the second line in the below code block to 'n', where 'n' is the number of nodes. Note that the connectivity matrix and edge matrix that we are importing should also be n x n. Confirm dimensions before importing.
Finally create a cleaned position matrix which contains only the latitude and longitude of the nodes. It should be a 'numpy.array' type variable.

Global Co-ordinates to Euclidean Co-ordinates
Function that returns the euclidean distance between 2 points which are in the global co-ordinate system (latitude and longitude) Input - Two 2x1 arrays of the form [latitude, longitude] Output - Euclidean distance between them

Heuristic
Input - 2 Nodes Output - Euclidean distance between the Nodes Extracct the latitude and longitude of the input Nodes using Node.position attribute of Nodes from the 'position' matrix. Feed this to the global_to_euclidean_dist() function

Node Class - 
Stores the position, parent f cost, g cost and h cost of each node. Position of a node is essentially the node index from 0 to 9, just like the indexing of an array. So for example, Node 6 on the google map will have position = 5 in the Node class. Similarly the parent node of a given node is also the position (index) of the parent node. So if the parent of Node 6 on the google map is Node 3 on the map, then in the Node class, the position of Node 6 will be 5 and parent will be 2. 
Child nodes of each node are explored while implementing the algorithm. 
g cost of child = g cost of parent + edge length (distance along road) connecting the two nodes.
h cost of child = heuristic (distance) between the child and the destination node. 
f cost = g cost + h cost 

Notes - 
1 : Connectivity matrix and Edge matrix are currently made manually. For connectivity matrix one has to manually check on google maps which nodes are joint to a given node. For the edge matrix, one has to manually draw a line on the google maps connecting two nodes along the road and store the distance shown on that line. 
2 : Output is of the form of a list. The global path is essentially the sequence of nodes in the output list. 
3 : Example - The shortest path from 1 to 10 is [1 7 8 10] So from node 1 we have to go to node 7 then to 8 and then to 10. The local planning of the path between two consecutive nodes is yet to be done. 

