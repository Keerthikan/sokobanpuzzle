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
		
		self.count = 0

	def null_heuristic(self,state): 
		return 0
		
	def manhattan_heuristic(self,state):	
		goalCoords = copy.deepcopy( self.smap.goals.keys() )
		boxes = copy.deepcopy( state.objects )
		
		sum = 0;
		for box in boxes:
			dist, index = self.getMDistAndIndexFromNearestObj( box, goalCoords )
			sum += dist
			
		return sum

	def navigation_heuristic(self,state):	
		goalCoords = copy.deepcopy( self.smap.goals.keys() )
		boxes = copy.deepcopy( state.objects )
		sum = 0
		for box in boxes:
			steps_block, parents_block = block_navigation_search( box, self.navMap )
			min_steps_block, goal_index = self.getStepsAndIndexToNearestGoal( goalCoords, steps_block )
			sum += min_steps_block
		return sum
		
		
#		goalCoords = copy.deepcopy( self.smap.goals.keys() )
#		boxes = copy.deepcopy( state.objects )
#		sum = 0
#		for box in boxes:
#			steps_nav, parents_block = navigation_search( box, self.navMap )
#			min_steps_nav, goal_index = self.getStepsAndIndexToNearestGoal( goalCoords, steps_nav )
#			sum += min_steps_nav
#		return sum
		
#		steps_nav, parents_nav = navigation_search( state.playerCoord, self.navMap )
#		steps_block, parents_block = block_navigation_search( [ 1, 3 ], self.navMap )
#		target = [ 2, 1 ]
#		index = self.navMap.coord_to_index( target )
#		print "parents_nav[index] ", parents_nav[index], "	steps_nav[index] ", steps_nav[index]
#		print "parents_block[index] ", parents_block[index], "	steps_block[index] ", steps_block[index]
#		return 0

	def cached_navigation_heuristic(self,state):
		return 0

	def other_heuristic(self, state):
		return 0

	def getMDistAndIndexFromNearestObj(self, object, goals):
		minDist = 1000000;
		minIndex = 0;
		for index in range(len(goals)):
			dist = manhattan_distance( object, goals[index] )
			if dist < minDist:
				minDist = dist
				minIndex = index
		return (minDist, minIndex)
	
	def getStepsAndIndexToNearestGoal(self, goals, steps):
		min_steps = 1000000
		min_index = 0
		for index in range(len(goals)):
			step_index = self.navMap.coord_to_index( goals[index] )
			if steps[ step_index ] < 0:
				continue
			if steps[ step_index ] < min_steps:
				min_steps = steps[ step_index ]
				min_index = index
		return min_steps, min_index
		