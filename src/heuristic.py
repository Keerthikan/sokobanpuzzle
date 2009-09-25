import sys
import math
import copy
from sokoban import *
from sokoban_main import *

BIG = 1e308;

def manhattan_distance(c1,c2):
	return abs(c1[0]-c2[0])+abs(c1[1]-c2[1])

def euclidean_distance(c1,c2):
	return math.sqrt(double((c1[0]-c2[0])*(c1[0]-c2[0])+(c1[1]-c2[1])*(c1[1]-c2[1])));
STORE_VISITED_STATES = 1
INT_MAX = 99999
LEFT, RIGHT, UP, DOWN = range(4)

# Performs a Dijkstra's algorithm to find the shortest paths from a position to all
# coordinates on the map.  It computes a list that stores
# the preceding shortest-path action leading from the start to point (x,y).
# That is, a path from the start to any grid point (x,y) can be extracted by
# following the action at (x,y) back to its parent (x',y'), and then
# recursing on (x',y') until the start is reached.  The array is indexed
# at parent[y*width+x] for a point (x,y).  If the action is None, then it is
# either the start node, or unreachable from the start.		
def navigation_search(start, nmap):
	numSteps = [INT_MAX] * (nmap.w*nmap.h)
	parent = [-1] * (nmap.w*nmap.h)
	q = []	
	#print 'map obstacles',nmap.obstacles.keys()
	
	index = nmap.coord_to_index(start)
	numSteps[index]=0
	q.append(start)
	while q:
		s = q.pop()
		index = nmap.coord_to_index(s)
		s_numSteps = numSteps[index]
		
		temp = (s[0]+1,s[1])
		index = nmap.coord_to_index(temp)
		if (not (nmap.is_obstacle(temp))) and (numSteps[index] > s_numSteps+1):
			parent[index] = RIGHT
			numSteps[index] = s_numSteps+1
			q.append(temp)

		temp = (s[0]-1,s[1])
		index = nmap.coord_to_index(temp)
		if (not (nmap.is_obstacle(temp))) and (numSteps[index] > s_numSteps+1):
			parent[index] = LEFT
			numSteps[index] = s_numSteps+1
			q.append(temp)

		temp = (s[0],s[1]+1)
		index = nmap.coord_to_index(temp)
		if (not (nmap.is_obstacle(temp))) and (numSteps[index] > s_numSteps+1):
			parent[index] = UP
			numSteps[index] = s_numSteps+1
			q.append(temp)

		temp = (s[0],s[1]-1)
		index = nmap.coord_to_index(temp)
		if (not (nmap.is_obstacle(temp))) and (numSteps[index] > s_numSteps+1):
			parent[index] = DOWN
			numSteps[index] = s_numSteps+1
			q.append(temp)
	
	for i in range(len(numSteps)):
		if numSteps[i] == INT_MAX:
			numSteps[i] = -1
	
	#print 'navigation_search',start,numSteps #for debugging
	return (numSteps, parent) 					


# Same to navigation_distance except that this is for block (boxes).
		
# In order for a block to move, it has free space on both side, i.e. left-right, up-down.
# One free space is the target location, the other free space is for player.		
def block_navigation_search(start, nmap):
	numSteps = [INT_MAX] * (nmap.w*nmap.h)
	parent = [-1] * (nmap.w*nmap.h)
	q = []		
		
	index = nmap.coord_to_index(start)
	numSteps[index]=0
	q.append(start)
	while q:
		s = q.pop()
		index = nmap.coord_to_index(s)
		s_numSteps = numSteps[index]
		
		a = (s[0]-1,s[1])
		b = (s[0]+1,s[1])
		if (not nmap.is_obstacle(a)) and (not nmap.is_obstacle(b)):
			index = nmap.coord_to_index(a)
			if (numSteps[index] > s_numSteps+1):
				parent[index] = LEFT
				numSteps[index] = s_numSteps+1
				q.append(a)
			index = nmap.coord_to_index(b)
			if (numSteps[index] > s_numSteps+1):
				parent[index] = RIGHT
				numSteps[index] = s_numSteps+1
				q.append(b)

		a = (s[0],s[1]-1)
		b = (s[0],s[1]+1)
		if (not nmap.is_obstacle(a)) and (not nmap.is_obstacle(b)):
			index = nmap.coord_to_index(a)
			if (numSteps[index] > s_numSteps+1):
				parent[index] = UP
				numSteps[index] = s_numSteps+1
				q.append(a)
			index = nmap.coord_to_index(b)
			if (numSteps[index] > s_numSteps+1):
				parent[index] = DOWN
				numSteps[index] = s_numSteps+1
				q.append(b)
	
	for i in range(len(numSteps)):
		if numSteps[i] == INT_MAX:
			numSteps[i] = -1

	#print 'navigation_search',start,numSteps #for debugging
	return (numSteps, parent)
	
		

# 
# IMPLEMENT ME!
# 
# Each of the following ___Heuristic() methods (with the exception of
# NullHeuristic() should be implemented.
# 
class SokobanHeuristic:
	def __init__(self, smap):
		self.smap = smap
		self.navMap = NavigationMap()
		self.navMap.w = smap.w
		self.navMap.h = smap.h
		self.navMap.obstacles = smap.obstacles
		self.cached_shortest_paths = [0] * (smap.w*smap.h)
		self.cached_shortest_paths_block = [0] * (smap.w*smap.h)
		self.shortest_distance = [0] * (smap.w*smap.h)
		self.shortest_distance_block = [0] * (smap.w*smap.h)

	def null_heuristic(self,state): 
		return 0
		
	def manhattan_heuristic(self,state):	
		goalCoords = list( self.smap.goals.keys() )
		# delete the @ object 
		for object in state.objects:
			for index in range(len(goalCoords)):
				if object == goalCoords[index]:
					del goalCoords[index]
					break
		
		#
		sum = 0
		playerCoord = state.playerCoord		
		#print "playerCoord: ", playerCoord 
		visitedGoalIndex = {}
		for object in state.objects:
			# if it is at the goal position, just continue
			if self.smap.is_goal( object ):
				continue
			# find the nearest goal
			minDist = 1000000;
			minIndex = 0;
			for index in range(len(goalCoords)):
				if index in visitedGoalIndex:
					continue
				dist = manhattan_distance( object, goalCoords[index] )
				if dist < minDist:
					minDist = dist
					minIndex = index
				#visitedGoalIndex[minIndex] = True;
			sum += minDist
			#visitedGoalIndex[minIndex] = True;
			#sum += manhattan_distance( object, playerCoord )
			#playerCoord = goalCoords[minIndex]
			#del goalCoords[minIndex] 		
		return sum

	def navigation_heuristic(self,state):
		return 0

	def cached_navigation_heuristic(self,state):
		return 0

	def other_heuristic(state):
		return 0

