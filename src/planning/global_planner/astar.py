# importing the required libraries
import numpy as np
import pandas as pd

# importing the csv file 
nodes_edges = pd.read_csv('testmap- Nodes and Edges.csv')

# extracting the longitudes and the latitudes in a matrix
nodes = nodes_edges.iloc[:10, 0]
# num_nodes = nodes_edges["WKT"].size
sep = nodes.str.split(expand = True)
sep[1] = sep[1].str.replace(r'\(','', regex=True).astype(float)   # longitude
sep[2] = sep[2].str.replace(r'\)','', regex=True).astype(float)   # latitude
position = np.zeros((len(sep[0]), 2))
position[:, 0], position[:, 1] = sep[1], sep[2]

print(position)

connectivity_matrix = pd.read_csv('connectivity_matrix.csv')
edge_matrix = pd.read_csv('edge_matrix.csv')
# print(connectivity_matrix)
# print(edge_matrix)

def global_to_euclidean_dist(a,b):  # inputs must be position matrix (np.array) 2x1
  radius_of_earth = 6371000
  dtr = np.pi/180
  x1 = radius_of_earth*np.cos(a[1]*dtr)*np.cos(a[0]*dtr)
  y1 = radius_of_earth*np.cos(a[1]*dtr)*np.sin(a[0]*dtr)
  z1 = radius_of_earth*np.sin(a[1]*dtr)
  x2 = radius_of_earth*np.cos(b[1]*dtr)*np.cos(b[0]*dtr)
  y2 = radius_of_earth*np.cos(b[1]*dtr)*np.sin(b[0]*dtr)
  z2 = radius_of_earth*np.sin(b[1]*dtr)
  a_pos = np.array([x1, y1, z1])
  b_pos = np.array([x2, y2, z2])
  dist = np.sqrt(np.sum(np.square(a_pos-b_pos)))
  return dist

def heuristic_distance(x, y):            # inputs must be Node type 
  x_pos = position[x.position, :].reshape(2,-1)
  y_pos = position[y.position, :].reshape(2,-1)
  distance = global_to_euclidean_dist(x_pos, y_pos)
  return distance



class Node():
  # A class to store the position, parent and f, g, h costs of each node
  
  def __init__(self, parent = None, position = None):
    self.parent = parent
    self.position = position 

    self.g = 0
    self.h = 0
    self.f = 0

# start_pos and end_pos are from 0 to num_nodes-1. 
def astar(connectivity_matrix, edge_matrix, start_pos, end_pos):
  start_node = Node(None, start_pos)
  end_node = Node(None, end_pos)
  start_node.g = start_node.h = start_node.f = 0
  end_node.g = end_node.h = end_node.f = 0

  # initialize the open and closed lists
  open_list = []                  
  closed_list = []                
  
  # add the start node to the open list 
  open_list.append(start_node)   


  # loop until the open list is not empty. It becomes empty when we reach goal
  while len(open_list) > 0: 

    # get the current node index - Node with min f cost in open list
    current_node = open_list[0]
    current_index = 0

    # loop through the open list to find node with min f cost
    for index, item in enumerate(open_list):
      if item.f < current_node.f :
        current_node = item
        curren_index = index 
    
    # remove the current node from open and put it in closed list 
    open_list.pop(current_index)
    closed_list.append(current_node)

    # check if the current node is the goal. If yes, generate the path
    if current_node.position == end_node.position : 
      path = []
      current = current_node
      while current is not None : 
        path.append(current.position)
        current = current.parent
      # whilel loop ends once we reach origin. Return the reversed path now 
      return path[::-1]  

    # if current node isn't the end node loop continues 

    # Generate the children of the current node 
    # children is also a list with nodes  
    children = []
    pos = current_node.position
    connections = connectivity_matrix.loc[pos].tolist()
    for i in range(len(connections)):
      if (connections[i] == 1):
        new_child_node = Node(current_node, i)
        children.append(new_child_node)
      
    # children list with all child nodes is ready. Check if a child is in closed
    # loop through children node to find f, g, h for each child

    for child in children:

      # if child is in closed, skip it and go to the next child 
      flag1 = 0
      for closed_node in closed_list:
        if child.position == closed_node.position:
          flag1 = 1
      if flag1 == 1 :
        continue        
      
      # calculate f, g, h costs for the child
      child.g = current_node.g + edge_matrix.loc[pos][child.position]
      child.h = heuristic_distance(child, end_node)
      child.f = child.g + child.h

      # check if child is in open list and update g cost and parent 
      flag2 = 0
      for open_node in open_list:
        if child.position == open_node.position:
          flag2 = 1
          if child.g < open_node.g :
            open_node.g = child.g 
            open_node.parent = child.parent
          
        
      if flag2 == 0:
        open_list.append(child)
        
        
# main function to take inputs and implement astar 
start_pos = int(input("Enter a starting position (between 1 and 10) "))
end_pos = int(input("Enter your end position (between 1 and 10) "))
global_path = astar(connectivity_matrix, edge_matrix, start_pos-1, end_pos-1)
path_print = np.array(global_path) + 1
print("Your route is {}".format(path_print))

