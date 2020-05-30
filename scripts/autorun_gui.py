#!/usr/bin/env python

import math
import tkinter
from PIL import Image, ImageTk

class Controller:
	def __init__(self, image_file, graph):
		self.offset_x = 900
		self.offset_y = 1473
		self.scale = 125.2
		
		self.window = tkinter.Tk()
		self.window.geometry('800x800')
		self.window.title("Navigation Status")

		self.canvas = tkinter.Canvas(self.window, width = 800, height = 800)
		self.canvas.pack(fill = tkinter.BOTH, expand = 1)

		self.bg_image = ImageTk.PhotoImage(Image.open(image_file).resize((800, 800)))
		self.canvas.create_image(0, 0, anchor = tkinter.NW, image = self.bg_image)
		self.display_graph(graph)
		
		x, y = self.transform(0, 0)
		self.cursor_size = 10.0
		self.cursor = self.canvas.create_polygon([0, 0, 0, 0, 0, 0], fill = 'blue')
		self.dest_size = 15.0
		self.dest = -1
		
	def update(self, x, y, w):
		new_x, new_y = self.transform(x, y)
		x1 = new_x - math.sin(w) * self.cursor_size
		y1 = new_y - math.cos(w) * self.cursor_size
		x2 = new_x - math.cos(w) * self.cursor_size / 2
		y2 = new_y + math.sin(w) * self.cursor_size / 2
		x3 = new_x + math.cos(w) * self.cursor_size / 2
		y3 = new_y - math.sin(w) * self.cursor_size / 2
		self.canvas.coords(self.cursor, x1, y1, x2, y2, x3, y3)
		self.window.update_idletasks()
		self.window.update()
		
	def display_graph(self, graph):
		texts = {}
		for row in graph:
			x1, y1 = self.transform(row[2], row[3])
			x2, y2 = self.transform(row[4], row[5])
			self.canvas.create_line(x1, y1, x2, y2, fill = 'red')
			if not int(row[0]) in texts:
				texts[int(row[0])] = self.canvas.create_text(x1, y1, text = str(int(row[0])))
			if not int(row[1]) in texts:
				texts[int(row[1])] = self.canvas.create_text(x2, y2, text = str(int(row[1])))
			
	def display_destination(self, x, y):
		x1 = x - self.dest_size / 4
		x2 = x + self.dest_size / 4
		y1 = y - self.dest_size
		if self.dest == -1:
			self.dest = self.canvas.create_polygon([x, y, x1, y1, x2, y1], fill = 'green')
		else:
			self.canvas.coords(self.dest, x, y, x1, y1, x2, y1)
			
	def transform(self, x, y):
		new_x = 3000 - (y * self.scale + self.offset_x)
		new_y = 3000 - (x * self.scale + self.offset_y)
		new_x = new_x * 800 / 3000
		new_y = new_y * 800 / 3000
		return int(new_x), int(new_y)
	
	def reverse_transform(self, x, y):
		new_x = float(x) * 3000 / 800
		new_y = float(y) * 3000 / 800
		new_x = (3000 - new_x - self.offset_x) / self.scale
		new_y = (3000 - new_y - self.offset_y) / self.scale
		return new_y, new_x