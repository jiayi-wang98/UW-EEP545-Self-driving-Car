import math
import numpy
from matplotlib import pyplot as plt
import cv2
import Utils
import time
import random

class HaltonPlanner(object):
  
  # planningEnv: Should be a HaltonEnvironment
  def __init__(self, planningEnv):
    self.planningEnv = planningEnv

  # Generate a plan
  # Assumes that the source and target were inserted just prior to calling this
  # Returns the generated plan
  def plan(self):
    t = time.time()
    self.sid = self.planningEnv.graph.number_of_nodes() - 2 # Get source id
    self.tid = self.planningEnv.graph.number_of_nodes() - 1 # Get target id

    self.closed = {} # The closed list
    self.parent = {self.sid:None} # A dictionary mapping children to their parents
    self.open = {self.sid: 0 + self.planningEnv.get_heuristic(self.sid, self.tid)} # The open list
    self.gValues = {self.sid:0} # A mapping from node to shortest found path length to that node 
    self.evaluated={}
    self.planIndices = []
    self.edges={}
    self.cost = 0
    # ------------------------------------------------------------
    # YOUR CODE HERE
    # 
    # Implement A*
    # Functions that you will probably use
    # - self.get_solution()
    # - self.planningEnv.get_successors()
    # - self.planningEnv.get_distance()
    # - self.planningEnv.get_heuristic()
    # Note that each node in the graph has both an associated id and configuration
    # You should be searching over ids, not configurations. get_successors() will return
    #   the ids of nodes that can be reached. Once you have a path plan
    #   of node ids, get_solution() will compute the actual path in SE(2) based off of
    #   the node ids that you have found.
    #-------------------------------------------------------------
    '''
    while 1:
      #find the point with minimal f_cost
      current_id=min(self.open, key=self.open.get)
      current_config=self.planningEnv.get_config(current_id)    

      #add the node into closed
      self.closed[current_id]=self.open[current_id]
      #delete the node in open
      del self.open[current_id]
      
      if current_id==self.tid:
      	break

      #search neighbours
      successors=self.planningEnv.get_successors(current_id)
      for i in range(len(successors)):
      #for successor_id in successors:
        successor_id=successors[i]
	successor_config=self.planningEnv.get_config(successor_id)

	if successor_id not in self.closed:
          if self.planningEnv.manager.get_edge_validity(successor_config, current_config):
        
            #new_dist=g+dist(thisnode,parentnode)+h(thisnode)
            new_g=self.gValues[current_id]+self.planningEnv.get_distance(current_id,successor_id)
            new_dist=new_g+self.planningEnv.get_heuristic(successor_id, self.tid)
            if successor_id in self.open:
              if self.open[successor_id]>new_dist:   
                self.gValues[successor_id]=new_g
                self.open[successor_id]=new_dist
                self.parent[successor_id]=current_id
            else:
              self.gValues[successor_id]=new_g
              self.open[successor_id]=new_dist
              self.parent[successor_id]=current_id
    '''
    # ------------------------------------------------------------
    # Implement lazy A*
    #-------------------------------------------------------------
    
    all_evaluated_flag=0
    count=1

    #keep finding path until all edges in the path is evaluated
    while all_evaluated_flag==0:
      self.edges={}
      self.closed = {} # The closed list
      self.parent = {self.sid:None} # A dictionary mapping children to their parents
      self.open = {self.sid: 0 + self.planningEnv.get_heuristic(self.sid, self.tid)} # The open list
      self.gValues = {self.sid:0} # A mapping from node to shortest found path length to that node 
      while 1:
        #find the point with minimal f_cost
        current_id=min(self.open, key=self.open.get)
        current_config=self.planningEnv.get_config(current_id)    

        #add the node into closed
        self.closed[current_id]=self.open[current_id]
        #delete the node in open
        del self.open[current_id]
        
        if current_id==self.tid:
           
          #get edges
          self.edges=self.get_edge(current_id)
          #print('find path',count)
          count+=1
          break

        #search neighbours
        for successor_id in self.planningEnv.get_successors(current_id):
        
	  successor_config=self.planningEnv.get_config(successor_id)
        
          #do not check collision here
    	  #if self.planningEnv.manager.get_edge_validity(successor_config, current_config) and (successor_id not in self.closed):
          if successor_id not in self.closed: 

            #new_dist=g+dist(thisnode,parentnode)+h(thisnode)
            if (current_id,successor_id) in self.evaluated:
              #true value
              new_g=self.gValues[current_id]+self.evaluated[(current_id,successor_id)]
              new_dist=new_g+self.planningEnv.get_heuristic(successor_id, self.tid)

            else:
              #estimater
              new_g=self.gValues[current_id]+self.planningEnv.get_distance(current_id,successor_id)
              new_dist=new_g+self.planningEnv.get_heuristic(successor_id, self.tid)

            if (successor_id in self.open and self.open[successor_id]>new_dist) or successor_id not in self.open:
              self.gValues[successor_id]=new_g
              self.open[successor_id]=new_dist
              self.parent[successor_id]=current_id
      
      #check if all edges are evaluated and evaluate all unevaluated edges
      all_evaluated_flag=1
      for edge in self.edges:
        if edge not in self.evaluated:
           all_evaluated_flag=0  
           edge0_config=self.planningEnv.get_config(edge[0])
           edge1_config=self.planningEnv.get_config(edge[1])

           #evaluate the first unevaluated edge 
           self.evaluated[edge]=self.planningEnv.get_distance(edge[0],edge[1]) if self.planningEnv.manager.get_edge_validity(edge0_config, edge1_config) else 10000
           break

    
    plan=self.get_solution(current_id)
    plan=self.post_process(plan, 3)
    #print(plan)
    print('Total cost=',self.cost)
    print('Total Time=',time.time()-t)
    return plan


# Backtrack across parents in order to get path edges
  def get_edge(self,vid):
    # Get all the node ids
    planID = []
    edge=[]
    while vid is not None:
      planID.append(vid)
      vid = self.parent[vid]
    
    planID.reverse()
    for i in range(len(planID)-1):
      edge.append((planID[i],planID[i+1]))
    return edge
    
    

# Try to improve the current plan by repeatedly checking if there is a shorter path between random pairs of points in the path
  def post_process(self, plan, timeout):

    t1 = time.time()
    elapsed = 0
    count=0
    while elapsed < timeout: # Keep going until out of time
      # ---------------------------------------------------------
      # YOUR CODE HERE
      
      # Pseudocode
      
      # Pick random id i
      # Pick random id j
      # Redraw if i == j
      # Switch i and j if i > j
     
      # if we can find path between i and j (Hint: look inside ObstacleManager.py for a suitable function)
        # Get the path
        # Reformat the plan such that the new path is inserted and the old section of the path is removed between i and j
        # Be sure to CAREFULLY inspect the data formats of both the original plan and the plan returned
        # to ensure that you edit the path correctly
      i=numpy.random.randint(0,len(plan))
      j=numpy.random.randint(0,len(plan))
      while j==i:
        j=numpy.random.randint(0,len(plan))
      if i>j:
        temp=j
        j=i
        i=temp
      
      iconfig = plan[i]
      jconfig = plan[j]

      if self.planningEnv.manager.get_edge_validity(iconfig, jconfig):
        px, py, clen = self.planningEnv.manager.discretize_edge(iconfig, jconfig)
        original_cost=0
        for k in range(i,j):
          original_cost+=numpy.linalg.norm(numpy.array(plan[k+1]) - numpy.array(plan[k]))
        if original_cost>clen:
          count+=1
          #print(count,"times optimize")
          del plan[i:j]
          plan=plan[:i]+[list(a) for a in zip(px, py)]+plan[i:]
          self.cost=self.cost-original_cost+clen

      elapsed = time.time() - t1
    return plan

  # Backtrack across parents in order to recover path
  # vid: The id of the last node in the graph
  def get_solution(self, vid):

    # Get all the node ids
    planID = []
    while vid is not None:
      planID.append(vid)
      vid = self.parent[vid]

    plan = []
    planID.reverse()
    for i in range(len(planID) - 1):
      startConfig = self.planningEnv.get_config(planID[i])
      goalConfig = self.planningEnv.get_config(planID[i + 1])
      px, py, clen = self.planningEnv.manager.discretize_edge(startConfig, goalConfig)
      plan.append([list(a) for a in zip(px, py)])
      self.planIndices.append(len(plan))
      self.cost += clen

    flatPlan = [item for sublist in plan for item in sublist]
    return flatPlan

  # Visualize the plan
  def simulate(self, plan):
    # Get the map
    envMap = 255*(self.planningEnv.manager.mapImageBW+1) # Hacky way to get correct coloring
    envMap = cv2.cvtColor(envMap, cv2.COLOR_GRAY2RGB)
    
    for i in range(numpy.shape(plan)[0]-1): # Draw lines between each configuration in the plan
      startPixel = Utils.world_to_map(plan[i], self.planningEnv.manager.map_info)
      goalPixel = Utils.world_to_map(plan[i+1], self.planningEnv.manager.map_info)
      cv2.line(envMap,(startPixel[0],startPixel[1]),(goalPixel[0],goalPixel[1]),(255,0,0),5)

    # Generate window
    cv2.namedWindow('Simulation', cv2.WINDOW_NORMAL)
    cv2.imshow('Simulation', envMap)

    # Terminate and exit elegantly
    cv2.waitKey(20000)
    cv2.destroyAllWindows()
