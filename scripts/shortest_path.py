#!/usr/bin/env python

import math
import numpy
from select_data import DataSelect


class Dijkstra:
	def __init__(self, graph):
		self.graph = graph
		self.nodes = numpy.unique(self.graph[:, :2])
		
	def get_path(self, start, end):
		distances = {}
		prev = {}
		visited = []
		cur = start
		for i in self.nodes:
			if i == start:
				distances[i] = 0
			else:
				distances[i] = float('inf')
			prev[i] = -1
		
		while end not in visited:
			for path in self.graph:
				d = math.sqrt((path[5] - path[4]) ** 2 + (path[3] - path[2]) ** 2)
				if path[0] == cur and distances[cur] + d < distances[path[1]]:
					distances[path[1]] = distances[cur] + d
					prev[path[1]] = cur
				elif path[1] == cur and distances[cur] + d < distances[path[0]]:
					distances[path[0]] = distances[cur] + d
					prev[path[0]] = cur
			visited.append(cur)
			
			not_visited = {k : v for k, v in distances.iteritems() if k not in visited}
			if len(not_visited) > 0:
				cur = min(not_visited, key=not_visited.get)
			else:
				break
		
		path = [end]
		cur = end
		while prev[cur] != -1:
			cur = prev[cur]
			path.insert(0, cur)
		return path
