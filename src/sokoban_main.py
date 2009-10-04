############################################
# sokoban_main.py
#
# Written: Ikhyun Park (ikpark)
# Adapted from C++ by Kris Hauser (hauserk)
# Last updated: 9/12/09 (ikpark)
############################################

import sys
import string
import os
from sokoban import *
from heuristic import *
from astar import *

heuristic_type = ['Null', 'ManhattanDistance', 'NavigationDistance', 'CachedNavigationDistance', 'Other']
PUSH, PULL = range(2)
NULL,MANHATTAN,NAVIGATION,CACHENAVIGATION,OTHER = range(5)

class SokobanAStar(AStar):
	def __init__(self, smap):
		self.smap = smap
		self.rules = SokobanRules(smap)
		self.hfunc = SokobanHeuristic(smap)
		self.s = PUSH
		self.h = NULL
		self.visited = {}
		
	def is_goal(self, state):
		#print 'SokobanAStar:is_goal:',state.playerCoord,state.objects	#for debugging
		return self.rules.is_goal(state)
		
	def successors(self, state):
		suc = self.rules.successors(state,self.s)
		c = [1] * len(suc)	# unit cost for every move
		return (suc, c)
		
	def heuristic(self, state):
		if self.h == NULL:
			return self.hfunc.null_heuristic(state)
		elif self.h == MANHATTAN:
			return self.hfunc.manhattan_heuristic(state)
		elif self.h == NAVIGATION:
			return self.hfunc.navigation_heuristic(state)
		elif self.h == CACHENAVIGATION:
			return self.hfunc.cached_navigation_heuristic(state)
		else:
			return self.hfunc.other_heuristic(state);

	def clear_visited(self):
		self.visited.clear()
		
	def visit(self, state, node):
		key = state.tup()
		self.visited[key] = node
		
	def visited_state_node(self, state):
		key = state.tup()
		if key in self.visited:
			return self.visited[key]
		else:
			return []

# This is for play mode
# 
class SokobanPlay:
	# Refered to sokoban.py. should be modified when sokoban.py's constant values change
	directions = {'l':0, 'r':1, 'u':2, 'd':3}
	
	def __init__(self, smap, start):
		self.rules = SokobanRules(smap)
		self.states = [SokobanState(start.playerCoord,start.objects)]
		self.cur_state = 0
		self.allow_pull = False

	def is_valid_command(self,c):
		valid_commands = "lrudbf1q"
		return c in valid_commands
	
	def perform_command(self,c):
		if c in self.directions:
			temp = self.rules.perform_action(self.states[self.cur_state],SokobanPlay.directions[c],self.allow_pull)
			if temp.playerCoord == (-1,-1):	# invalid move
				print 'Invalid Move', c
			else:
				self.states[self.cur_state+1:] = []
				self.states.append(temp)
				self.cur_state += 1
		elif c == 'b':
			if self.cur_state > 0:
				self.cur_state -= 1
			else:
				print "Can't go backward any more"
		elif c == 'f':
			if self.cur_state < len(self.states):
				self.cur_state += 1
			else:
				print "Can't go forward any more"
		elif c == '1':
			self.cur_state = 0



USAGE_STRING = "USAGE: python sokoban [options] file.map";

OPTIONS_STRING = "OPTIONS:\n\
-h HEURISTICS: use the heuristics indexed HEURISTICS, where\n\
\t0 is a null heuristic,\n\
\t1 uses ManhattanHeuristic,\n\
\t2 uses NavigationHeuristic,\n\
\t3 uses CachedNavigationHeuristic,\n\
\t4 uses OtherHeuristic.\n\
Multiple heuristics can be specified, e.g. 023.\n\
By default, HEURISTICS is 0.\n\
-max max_iters: set the maximum number of iterations (default 1,000,000).\n\
-debug print_iters: print search stats every print_iters iterations.\n\
-search: enable search mode (enabled by default)\n\
-play: enable play mode\n\
-path path_file: playback the file path_file in play mode\n\
-pull: allow pulling boxes\
";

if os.name == 'nt':
	CLEAR_SCREEN = 'cls'
else:
	CLEAR_SCREEN = 'clear'

if __name__ == "__main__":
	SEARCH, PLAY = range(2)
	NULL = 0;
	if len(sys.argv) <= 1:
		print USAGE_STRING
		print OPTIONS_STRING
	else:
		heuristics = "0"
		print_iter_count = 0
		max_iters = 100000
		mode=SEARCH
		pathfile = NULL
		allow_pulls=False

		# parse command-line
		i = 1
		while i < len(sys.argv):
			if sys.argv[i][0] != '-':
				break
			else:
				if sys.argv[i] == "-h":
					heuristics = sys.argv[i+1]
					i += 1
				elif sys.argv[i] == "-debug":
					print_iter_count = int(sys.argv[i+1])
					i += 1
				elif sys.argv[i] == "-max":
					max_iters = int(sys.argv[i+1])
					i += 1
				elif sys.argv[i] == "-search":
					mode = SEARCH
				elif sys.argv[i] == "-play":
					mode = PLAY
				elif sys.argv[i] == "-path":
					mode = PLAY;
					pathfile=sys.argv[i+1];
					i += 1;
				elif sys.argv[i] == "-pull":
					allow_pulls=True;
				else:
					print 'Invalid option', sys.argv[i]
					print OPTIONS_STRING
					sys.exit(0)
			i += 1
		if i == len(sys.argv):
			print 'No map file specified\n'
			print USAGE_STRING
			sys.exit(0)
  
		#load the map
		mapfile = sys.argv[i];
		fin = open(mapfile)
		res = load_sokoban(fin)
		fin.close()
		if res == False:
			sys.exit(0)
		state, smap = res
		#print 'start state',state.playerCoord, state.objects	# for debugging
		#print 'map goals',smap.goals	# for debugging
		
		if mode == PLAY:
			os.system(CLEAR_SCREEN)
			play = SokobanPlay(smap,state)
			play.allow_pull=allow_pulls
		
			# need to check if it works
			if pathfile:
				#print 'reading pathfile',pathfile
				fin = open(pathfile)
				while fin:
					c = fin.read(1)
					if c == "":
						break;
					if not (c in string.whitespace):
						if play.is_valid_command(c):
							play.perform_command(c)
						else:
							print 'Invalid command', c

			# game loop

			done=False
			while(not done):
				print 'Step',play.cur_state
				if play.rules.is_goal(play.states[play.cur_state]):
					print 'FINISHED!'
				print_sokoban(sys.stdout, play.states[play.cur_state], smap)
				print "l=left,r=right,u=up,d=down,b=back,f=forward,1=reset,q=quit"
				
				valid_string = False
				while(not valid_string):
					cmd = sys.stdin.readline().strip()
					if len(cmd) > 0 :
						valid_string = True
						for c in cmd:
							if not play.is_valid_command(c):
								print 'Invalid command',c
								valid_string = False
							if c == 'q':
								print 'Quiting'
								sys.exit(0)
				os.system(CLEAR_SCREEN)
				for c in cmd:
					play.perform_command(c)
		else: # SEARCH mode
			astar = SokobanAStar(smap)
			n=len(heuristics)
			for i in range(n):
				if('0'<=heuristics[i] and heuristics[i]<='4'):
					h = int(heuristics[i]);
					astar.h = h;
					if allow_pulls:
						astar.s = PULL
					else:
						astar.s = PUSH
					astar.set_start(state)
					print "Beginning planning with heuristic: ", heuristic_type[h]
					
					res=False;
					for iters in range(1,max_iters):
						if(astar.search_step()):
							res = True
							break
	
						if(print_iter_count>0 and iters%print_iter_count==0):
							os.system(CLEAR_SCREEN);
							print "Iteration ",iters,":"
							print "Tree has size ",astar.num_nodes()
							print "Fringe has size ",len(astar.fringe)
							
					if(res):
					  print "Astar completed with ",astar.num_nodes()," nodes, depth ",len(astar.path)
					  #print "Astar completed with ",astar.num_nodes(),astar.nid," nodes, depth ",len(astar.path)
					else:
					  print "Astar failed after ",max_iters," iterations were reached."
					  print "Tree has ",astar.num_nodes()," nodes"
				else:
					print "Invalid heuristic specification ",heuristics[i],", must be between 0 and 4"
					sys.exit(-1)
	
			if(astar.path):
				pathfile = mapfile.replace('map','path')
				print "Saving result to ",pathfile
				fout = open(pathfile,'w')
				for i in range(1,len(astar.path)):
					s = astar.path[i].playerCoord
					p = astar.path[i-1].playerCoord;
					if(s[0] > p[0]):
					  fout.write('r')
					elif(s[0] < p[0]):
					  fout.write('l')
					elif(s[1] < p[1]):
					  fout.write('u')
					elif(s[1] > p[1]):
					  fout.write('d')
					else:
					  print "Uhhh... invalid path being saved???"
					  sys.exit(-1)
	      
				fout.write('\n')
				fout.close()
			
