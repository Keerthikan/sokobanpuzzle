from heapq import *
from heuristic import *
import sys

# can be set to 1 -- may return a suboptimal solution (but faster)
TEST_GOAL_ON_GENERATION = 0

class AStar:
	F, H, G, NID, STATE, PARENT, CHILDREN = range(7)
  
	def __init__(self, state):
		self.set_start(state)
		
		
  # Resets the search from the given start state
	def set_start(self,start):
		self.clear_visited()

	  	# The A* search fringe.
	  	# A priority list of nodes
	  	# A Node is again a list consisting of [f h state parent_node children_nodes]
	  	# when heappop, node with smallest f comes first, 
	  	# 	if two nodes have same f value, the node with smallest h comes first and so on
		self.fringe = []
		self.goal = []
		self.nid = 0

		if TEST_GOAL_ON_GENERATION:
			if self.is_goal(start):
				self.path.append(start)
				return
			
		# initialize with the root node
		self.add_successor([],start,0)
   
	# Performs search until a goal is reached
	def search(self):
		while( self.fringe ):
			res = self.search_step()
			if res:
				return True
		return False
  
  # Performs a single iteration of search
	def search_step(self):
		if( self.fringe == [] ):
			return False;

		n = heappop(self.fringe);

		if not TEST_GOAL_ON_GENERATION:
			if self.is_goal(n[AStar.STATE]):
				self.goal = n
				self.path = []
				while n:
					self.path.append(n[AStar.STATE])
					n = n[AStar.PARENT]
				self.path.reverse()
				return True
		
		successors, costs = self.successors(n[AStar.STATE])

		for i, succ in enumerate(successors):
			#print 'successor',succ.playerCoord,succ.objects	# for debugging
			# succ is a Sokoban state, not a node yet
			if TEST_GOAL_ON_GENERATION:
				if self.is_goal(succ):
					self.goal = []
					self.path = []
					self.path.append(succ)
					while n:
						self.path.append(n[AStar.STATE])
						n = n[AStar.PARENT]
					self.path.reverse()
					return True
			visited = self.visited_state_node(succ)
			#print 'visited',visited #for debugging
			if(visited):
				#print 'revisit happend'	#for debugging
				if( n[AStar.G] + costs[i] >= visited[AStar.G] ):	# cost is higer than previous one, then ignore this new state
					continue
				else:	#cost is lower than previous, keep new state, delete the previous one from fringe
					#print 'old cost',visited[AStar.G],'new cost',n[AStar.G]+costs[i]
					try:
						self.fringe.remove(visited)
						heapify(self.fringe)
					except:
						print 'Strange! self.fringe.remove(visited) fails'
					self.add_successor(n,succ,costs[i])
			else:	# succ's state has never been visited
				self.add_successor(n,succ,costs[i])
		return False
  

  # Returns true if search failed
	def search_failed(self):
		return fringe == []
  	
  # Returns the number of nodes in the tree
	def num_nodes(self):
		return 1 + self.num_descendents(self.root);
  	
  # Returns the number of descendents of n
	def num_descendents(self,node):
		count = len(node[AStar.CHILDREN])
		for child in node[AStar.CHILDREN]:
			count += self.num_descendents(child)
		return count
  		
  # Adds a state as a successor of n, adds to fringe, and visits it
	def add_successor(self,node,state,cost):
		if node == []:
			g = 0
			h = self.heuristic(state)
			f = g + h
			parent = []
			children = []
			self.root = [f, h, g, self.nid, state, parent, children]
			self.nid += 1
			heappush(self.fringe,self.root)
			self.visit(state,self.root)
			return self.root
		else:
			g = node[AStar.G] + cost
			h = self.heuristic(state)
			f = g + h
			parent = node
			children = []
			child = [f, h, g, self.nid, state, parent, children]
			self.nid += 1
			# add the new node to the fringe and mark its state as visited
			heappush(self.fringe,child)
			node[AStar.CHILDREN].append(child)
			#print 'child',child #for debugging
			#print 'fringe',self.fringe	#for debugging
			self.visit(state,child)

			return child

	#
	# The followings must be overloaded by the subclass
	#
	def is_goal(self,state):
		return
	
	def successors(self,state):
		return

	# visited test
	def clear_visited(self):
		return
		
	def visit(state,node):
		return
		
	def visited_state_node(state): 
		return []

  # Optionally, overload these functions.  If not overloaded, does no
	def heuristic(self,state):
		return 0


