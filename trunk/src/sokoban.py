##################################
# sokoban.py
#
# Classes acting as representations of a Sokoban map
# and a navigation map (for navigation heuristic).
#
# I/O routines for Sokoban maps at end of file.
#
# Written: Mark Wilson (mw54)
# Adapted from C++ by Kris Hauser (hauserk)
# Last updated: 9/11/09 (mw54)
##################################


import sys

# Uses 2-tuples (x,y) to represent coordinates throughout.
# Tuples are immutable (so can safely be used as dict keys)
# and also comparable with lexicographical ordering (as the
# original NavigationState object in C++)
#
# As in original code, x increases left to right and
# y increases top to bottom.




# Quick sub for an enum and some utility methods
class NavigationDirection:
	LEFT = 0
	RIGHT = 1
	UP = 2
	DOWN = 3
	
	# Verifies whether a value is one of the enumerated directions
	def is_direction(cls, direction):
		return direction == NavigationDirection.LEFT \
			or direction == NavigationDirection.RIGHT \
			or direction == NavigationDirection.UP \
			or direction == NavigationDirection.DOWN
	is_direction = classmethod(is_direction)
	
	# Given a 2-coordinate and a direction, returns the coordinate
	# representing a move one space in the indicated direction
	def move_in_direction(cls, coord, direction):
		# Adjust x direction if moving left/right
		# Adjust y direction if moving up/down
		dx = 0
		dy = 0
		if direction == NavigationDirection.LEFT:
			dx = -1
		elif direction == NavigationDirection.RIGHT:
			dx = 1
		elif direction == NavigationDirection.UP:
			dy = -1
		elif direction == NavigationDirection.DOWN:
			dy = 1
		else:
			print >> sys.stderr, \
				"NavigationDirection::move_in_direction(): Invalid action"
		ncoord = (coord[0]+dx, coord[1]+dy)
		return ncoord
	move_in_direction = classmethod(move_in_direction)
	
	# Given a 2-coordinate and a direction, returns the coordinate
	# representing a move one space opposite to the given direction
	def move_opposite_direction(cls, coord, direction):
		dx = 0
		dy = 0
		if direction == NavigationDirection.LEFT:
			dx = 1
		elif direction == NavigationDirection.RIGHT:
			dx = -1
		elif direction == NavigationDirection.UP:
			dy = 1
		elif direction == NavigationDirection.DOWN:
			dy = -1
		else:
			print >> sys.stderr, \
				"NavigationDirection::move_opposite_direction(): Invalid action"
		ncoord = (coord[0]+dx, coord[1]+dy)
		return ncoord
	move_opposite_direction = classmethod(move_opposite_direction)






############################
#
# Navigation-heuristic classes
#
###########################


class NavigationMap:
	zeroCoord = (0,0)
	def __init__(self, w=0, h=0):
		# I'm giving a try to storing the obstacles as keys in a dictionary.
		# Hopefully hashing will give us quick lookup without requiring O(n)
		# space (though worst-case lookup is amortized O(n), so who knows)
		self.obstacles = {}
		# Potential TODO: negative width, height
		self.w = w
		self.h = h
		
	def resize(self, w, h):
		self.w = w
		self.h = h
	
	def is_in_bounds(self, coord):
		return coord[0] < self.w and coord[1] < self.h \
			and coord[0] > 0 and coord[1] > 0
	
	def set_obstacle(self, coord, val=True):
		self.obstacles[coord] = val
	
	def is_obstacle(self, coord):
		return coord in self.obstacles and self.obstacles[coord] == True
	
	def coord_to_index(self, coord):
		return coord[1]*self.w + coord[0]
	
	def clear(self):
		self.obstacles = {}
		


class NavigationRules:
	# navMap should be a NavigationMap, goalCoord should be a coordinate tuple
	def __init__(self, navMap, goalCoord):
		self.navMap = navMap
		self.goalCoord = goalCoord
		# Potential TODO: Goal coords not valid
	
	# Original code seems to leave bounds checking out?
	def is_valid(self, coord):
		return self.navMap.is_in_bounds(coord) \
			and not self.navMap.is_obstacle(coord)
	
	def is_goal(self, coord):
		return self.goalCoord == coord
	
	# Returns the tuple (-1,-1) if the action is invalid
	def perform_action(self, coord, direction):
		if not NavigationDirection.is_direction(direction):
			print >> sys.stderr, \
				"NavigationRules::perform_action(): Invalid action"
			sys.exit(1)
		
		coord = NavigationDirection.move_in_direction(coord, direction)
		
		# Check to make sure it's on the board and not in an obstacle
		if not self.is_valid(coord):
			return (-1,-1)
		return coord
		
	def successors(self, coord):
		# A hack using the internals of the NavigationDirection
		successors = []
		for i in range(4):
			temp = self.perform_action(coord, i)
			if temp != (-1,-1):
				successors.append(temp)
		return successors
		
		
		
		
		


##############################
#
# Sokoban classes
#
##############################		


# Sokoban has objects on the map as well as a player, so it needs
# more than just a 2-coordinate to represent state
class SokobanState:
	# playerCoord is a 2-coordinate tuple
	# objects is a list of 2-coordinate tuples
	def __init__(self, playerCoord=(), objects=[]):
		self.playerCoord = playerCoord
		self.objects = objects
	
	def tup(self):
		return (self.playerCoord, tuple(self.objects))
		
	def __repr__(self):
		return str( self.tup() )
	
	def __hash__(self):
		return hash( self.tup() )
	
	def __lt__(self, other):
		return self.tup() < other.tup()
	
	def __le__(self, other):
		return self.tup() <= other.tup()
	
	def __eq__(self, other):
		return self.tup() == other.tup()
	
	def __ne__(self, other):
		return self.tup() != other.tup()
	
	def __gt__(self, other):
		return self.tup() > other.tup()
	
	def __ge__(self, other):
		return self.tup() >= other.tup()
		


# SokobanMap reimplements a lot of NavigationMap, so it just
# inherits
class SokobanMap(NavigationMap):
	def __init__(self, w=0, h=0):
		# Again, dictionary for storing the goals, hoping for
		# good lookup times
		NavigationMap.__init__(self,w,h)
		self.goals = {}
		
	def set_goal(self, coord):
		self.goals[coord] = True
		
	def is_goal(self, coord):
		return coord in self.goals and self.goals[coord] == True


# The navMap should now be a SokobanMap.
#
# Originally inherited from NavigationRules, but the difference between
# the 2-coordinate state and SokobanState messes with the method overrides.
# Could be polymorphic again if overridden methods using SokobanStates
# explicitly handled tuples and called the super functions (ugly) or if 
# NavigationRules was rewritten to use a SokobanState with no objects.
class SokobanRules:
	def __init__(self, navMap):
		self.navMap = navMap
	
	# Checks to see if a 2-coordinate matches up with any objects
	# in the state's object list.  If not, returns (-1, ()).
	#
	# If yes, returns (i, (x,y)), where i is the object's index in the
	# SokobanState and (x,y) are the object's new coordinates when moved in the
	# indicated direction.
	#
	# Utility method for perform_action()
	def check_moved_object(self, state, coord, direction):
		newObject = ()
		movedObject = -1
		for i in range(len(state.objects)):
			if coord == state.objects[i]:
				if movedObject >= 0:
					print >> sys.stderr, \
						"SokobanRules::check_moved_object(): More than one obstacle moved?"
					sys.exit(1)
				movedObject = i
				
				if not NavigationDirection.is_direction(direction):
					print >> sys.stderr, \
						"SokobanRules::check_moved_object(): Invalid action"
					sys.exit(1)
				
				# safe because tuples are immutable
				newObject = NavigationDirection.move_in_direction( \
					state.objects[i], direction)
		return (movedObject, newObject)
		
	
	# Deep-copies an object list from a state, subbing in a new object
	# if it was moved
	#
	# Set movedObject to -1 to just copy the whole list
	#
	# Utility method for perform_action()
	def make_new_object_list(self, state, movedObject, newObject):
		objects = []
		for i in range(len(state.objects)):
			if i != movedObject:
				# Tuples are immutable, so just copy
				objects.append(state.objects[i])
			else:
				objects.append(newObject)
		return objects
	
	# Lets us know if a moved object is legal
	# (i.e., not on another object or an obstacle)
	#
	# Utility method for perform_action()
	def is_legal_object_move(self, state, movedObject, coord):
		# Check if the new object is on an obstacle
		if self.navMap.is_obstacle(coord):
			return False
		# Check if the new object is on another object
		for i in range(len(state.objects)):
			if i != movedObject and coord == state.objects[i]:
				return False
		return True
	
	# With Sokoban, we check states for validity, not 2-coords
	def is_valid(self, state):
		# player has to be in free space
		if self.navMap.is_obstacle(state.playerCoord):
			return False
		# objects have to be in free space,
		# may not overlap player,
		# and may not overlap each other
		for i in range(len(state.objects)):
			if self.navMap.is_obstacle(state.objects[i]):
				return False
			if state.objects[i] == state.playerCoord:
				return False
			for j in range(len(state.objects))[i+1:]:
				if state.objects[i] == state.objects[j]:
					return False
		# number of objects has to equal number of goals
		if len(state.objects) != len(self.navMap.goals):
			return False
		return True
	
	# With Sokoban, we have to check a SokobanState for goal-ness
	# rather than a 2-coord
	def is_goal(self, state):
		for object in state.objects:
			if not self.navMap.is_goal(object):
				return False
		return self.is_valid(state)
	
	# With Sokoban, we have to check to see if objects are being
	# moved and whether it's legal
	#
	# This time we take a SokobanState instead of a coordinate tuple.
	#
	# Likewise on error we return a SokobanState with player coordinate (-1,-1)
	#
	# In Sokoban, we may or may not be pulling blocks - set "pull"
	# True or False as appropriate
	def perform_action(self, state, direction, pull):
		if not NavigationDirection.is_direction(direction):
			print >> sys.stderr, \
				"SokobanRules::perform_action(): Invalid action"
			sys.exit(1)
		
		newCoord = NavigationDirection.move_in_direction(state.playerCoord, \
			direction)
		
		# Check to make sure it's on the board and not in an obstacle
		if not self.navMap.is_in_bounds(newCoord) \
				or self.navMap.is_obstacle(newCoord):
			return SokobanState((-1,-1), [])
		
		# have to check for a moved object
		movedObject, newObject = self.check_moved_object(state, newCoord, \
			direction)
		
		# if moved object, check to make sure it's legal
		if movedObject >= 0 and not self.is_legal_object_move(state, \
				movedObject, newObject):
			return SokobanState((-1,-1), [])
		
		# HANDLE PULLING
		if movedObject < 0 and pull:
			# We've checked direction validity twice already
			pullSpot = NavigationDirection.move_opposite_direction( \
				state.playerCoord, direction)
			movedObject, newObject = self.check_moved_object(state, pullSpot, \
				direction)
		
		# deep-copy the object list and sub in the moved object if necessary
		newStateObjects = self.make_new_object_list(state, movedObject, \
			newObject)
		
		# Make a new SokobanState from revised player, object coordinates
		return SokobanState(newCoord, newStateObjects)
	

	# Again, we now take a SokobanState and not a coordinate tuple
	# Likewise, return list of SokobanStates
	#
	# In Sokoban, we may or may not be pulling blocks - set "pull"
	# True or False as appropriate
	def successors(self, state, pull):
		# A hack using the internals of the NavigationDirection
		successors = []
		for i in range(4):
			temp = self.perform_action(state, i, pull)
			if temp.playerCoord != (-1,-1):
				successors.append(temp)
		return successors
		



###############################
#
# I/O Routines
#
###############################


# Loads a Sokoban map from an open file handle
# Returns a tuple containing (SokobanState, SokobanMap) on success
# Returns False on failure
#
# NB: Opening and closing the file handle are both the caller's responsibility!
# The seek position of the file handle after the function executes will be
# arbitrary.
def load_sokoban(fin):
	fin.seek(0)
	
	# get width and height
	w = None
	h = None
	try:
		w,h = fin.readline().strip().split()
	except IOError:
		print >> stderr, "load_sokoban(): width, height not present."
		return False
		
	w = int(w)
	h = int(h)
	if w < 0 or h < 0:
		print >> stderr, "load_sokoban(): width, height negative."
		return False
	
	
	# Create the map object and prep for the state object
	smap = SokobanMap(w,h)
	objects = []
	player = ()
	
	# Count lines
	lCount = 0
	for line in fin:
		if lCount >= h:
			break
		line = line.strip('\n')
		# Count characters
		cCount = 0
		for c in line:
			coord = (cCount, lCount)
			if c == '#':
				smap.set_obstacle(coord)
			elif c == 'o':
				objects.append(coord)
			elif c == 'p':
				if player != ():
					print >> stderr, \
						"load_sokoban(): more than one player on board."
					return False
				player = coord
			elif c == '@':
				objects.append(coord)
				smap.set_goal(coord)
			elif c == '.':
				smap.set_goal(coord)
			elif c == '8':
				# Original code treats this player as an object?
				if player != ():
					print >> stderr, \
						"load_sokoban(): more than one player on board."
					return False
				player = coord
				smap.set_goal(coord)
			elif c != ' ':
				print >> stderr, \
					"load_sokoban(): unrecognized character."
				return False
			cCount += 1
			#end loop over characters in line
			
		if cCount != w:
			print >> stderr, "load_sokoban(): incorrect width"
			return False
		lCount += 1
		#end loop over lines in file
		
	if lCount != h:
		print >> stderr, "load_sokoban(): incorrect height"
		return False
		
	# Error checking
	# I'm pretty sure this is not complete, just covers the most obvious cases
	if player == ():
		print >> stderr, "load_sokoban(): player not found"
		return False
	if len(objects) == 0:
		print >> stderr, "load_sokoban(): no objects found"
		return False
	if len(objects) != len(smap.goals):
		print >> stderr, \
			"load_sokoban(): number of goals does not match number of objects"
		
	state = SokobanState(player, objects)
	return (state, smap)


# Prints a Sokoban map (represented in entirety by state and map objects)
# to a file handle opened for writing.
#
# To print to console, pass in sys.stdout for fout.
#
# NB: Opening and closing the file handle are both the caller's responsibility!
# The seek position of the file handle after the function executes will be
# arbitrary.
def print_sokoban(fout, state, smap):
	for i in range(smap.h):
		for j in range(smap.w):
			coord = (j,i)
			if smap.is_obstacle(coord):
				fout.write('#')
			elif smap.is_goal(coord):
				if state.playerCoord == coord:
					fout.write('8')
				else:
					isObject = False
					for object in state.objects:
						if object == coord:
							fout.write('@')
							isObject = True
							break
					if not isObject:
						fout.write('.')
			elif state.playerCoord == coord:
				fout.write('p')
			else:
				isObject = False
				for object in state.objects:
					if object == coord:
						fout.write('o')
						isObject = True
						break
				if not isObject:
					fout.write(' ')
		fout.write('\n')
	fout.write('\n')
	