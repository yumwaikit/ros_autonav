#!/usr/bin/env python

import numpy
import pymssql

class DataSelect:
	def __init__(self):
		self.server = 'localhost'
		self.user = 'SA'
		self.password = 'Mediaasia123'
		self.db = 'TestDB'

	def select_db(self, sql):
		try:
			conn = pymssql.connect(self.server, self.user, self.password, self.db)
			cursor = conn.cursor()
			cursor.execute(sql)
			rows = cursor.fetchall()
			conn.close()
			return numpy.array(rows)
		except pymssql.ProgrammingError:
			return None
		
	def select_graph(self):
		return self.select_db('''select p.a, p.b, n1.x as xa, n1.y as ya, n2.x as xb, n2.y as yb from Paths as p 
							left outer join Nodes as n1 on p.a = n1.id 
							left outer join Nodes as n2 on p.b = n2.id''')
	
	def select_nodes(self):
		nodes = self.select_db('select id, x, y from Nodes')
		return {row[0] : (row[1], row[2]) for row in nodes}
	
if __name__=="__main__":
	data = DataSelect()
	print(data.select_nodes())