import cv2
import math
import numpy
import Utils
import matplotlib.pyplot as plt

B_POINT = [[1910, 340, 0.0],[1500, 210, 0.0],[1520, 435, 0.0], [1130, 400, 0.0],[670, 840, 0.0]]

class ObstacleManager(object):

	def __init__(self, mapMsg, car_width, car_length, collision_delta):
		self.map_info = mapMsg.info
		self.mapImageGS = numpy.array(mapMsg.data, dtype=numpy.uint8).reshape(
			(mapMsg.info.height, mapMsg.info.width, 1))

		# Retrieve the map dimensions
		height, width, channels = self.mapImageGS.shape
		self.mapHeight = height
		self.mapWidth = width
		self.mapChannels = channels

		# Binarize the Image
		self.mapImageBW = 255 * numpy.ones_like(self.mapImageGS, dtype=numpy.uint8)
		self.mapImageBW[self.mapImageGS == 0] = 0
                print(self.mapImageBW.shape)
                for i in xrange(5):
                   print(self.mapImageBW[self.mapHeight-B_POINT[i][1]][B_POINT[i][0]])
                   self.mapImageBW[self.mapHeight-B_POINT[i][1]-10:self.mapHeight-B_POINT[i][1]+10,B_POINT[i][0]-10:B_POINT[i][0]+10]=255
                   print(self.mapImageBW[self.mapHeight-B_POINT[i][1]][B_POINT[i][0]])

               # plt.imshow(self.mapImageBW.reshape((mapMsg.info.height, mapMsg.info.width)))
               # plt.show()
		# Obtain the car length and width in pixels
		self.robotWidth = int(car_width / self.map_info.resolution + 0.5)
		self.robotLength = int(car_length / self.map_info.resolution + 0.5)
		self.collision_delta = collision_delta

	# Check if the passed config is in collision
	# config: The configuration to check (in meters and radians)
	# Returns False if in collision, True if not in collision
	def get_state_validity(self, config):

		# Convert the configuration to map-coordinates -> mapConfig is in pixel-space
		mapConfig = Utils.world_to_map(config, self.map_info)

		# ---------------------------------------------------------
		# YOUR CODE HERE
		#
		# Return true or false based on whether the robot's configuration is in collision
		# Use a square to represent the robot, return true only when all points within
		# the square are collision free
		#
		# Also return false if the robot is out of bounds of the map
		#
		# Although our configuration includes rotation, assume that the
		# square representing the robot is always aligned with the coordinate axes of the
		# map for simplicity
		# ----------------------------------------------------------
                for i in xrange(mapConfig[1]-int(0.5*(self.robotLength)),mapConfig[1]+int(0.5*self.robotLength)+1):
                       for j in xrange(mapConfig[0]-int(0.5*self.robotWidth),mapConfig[0]+int(0.5*self.robotWidth)+1):
                              if i<0 or i>=self.mapImageBW.shape[0] or j<0 or j>=self.mapImageBW.shape[1]:
                                     #print(i,j,'out of boundary',self.mapHeight,self.mapWidth)
                                     return False
                              if self.mapImageBW[i][j]!=0:
                                     #print(i,j,'collision occurs',)
                                     return False               

		return True

	# Discretize the path into N configurations, where N = path_length / self.collision_delta
	#
	# input: an edge represented by the start and end configurations
	#
	# return three variables:
	# list_x - a list of x values of all intermediate points in the path
	# list_y - a list of y values of all intermediate points in the path
	# edgeLength - The euclidean distance between config1 and config2
	def discretize_edge(self, config1, config2):
		list_x, list_y = [], []
		edgeLength = 0
		# -----------------------------------------------------------
		# YOUR CODE HERE
		# -----------------------------------------------------------'
                edgeLength = numpy.linalg.norm(numpy.array(config1) - numpy.array(config2))

                N = int(edgeLength/self.collision_delta)
                
                list_x=numpy.linspace(config1[0], config2[0], num=N+1, endpoint=False)
                list_y=numpy.linspace(config1[1], config2[1], num=N+1, endpoint=False)

		return list_x, list_y, edgeLength


	# Check if there is an unobstructed edge between the passed configs
	# config1, config2: The configurations to check (in meters and radians)
	# Returns false if obstructed edge, True otherwise
	def get_edge_validity(self, config1, config2):
		# -----------------------------------------------------------
		# YOUR CODE HERE
		#
		# Check if endpoints are obstructed, if either is, return false
		# Find path between two configs by connecting them with a straight line
		# Discretize the path with the discretized_edge function above
		# Check if all configurations along path are obstructed
		# -----------------------------------------------------------
                if (not self.get_state_validity(config1)) or (not self.get_state_validity(config2)):
                        return False
                
                list_x,list_y,edgeLength=self.discretize_edge(config1, config2)
                
                for i in range(len(list_x)):
                	if not self.get_state_validity([list_x[i],list_y[i]]):
                                 return False
                        
		return True


# Write Your Test Code For Debugging
#if __name__ == '__main__':
#	return
	# Write test code here!
