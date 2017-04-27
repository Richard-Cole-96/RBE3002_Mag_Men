#!/usr/bin/env python
import math

class Node:

	def __init__(self, x, y, wallChance, resolution):
		self.x = x
		self.y = y 
		self.resolution = resolution
		self.gCost = 0
		self.hCost = 0
		self.fCost = 0
		self.parent = None
		self.visited = False
		self.inNotVisited = False
		self.padding = False
		self.frontier = False
		if (wallChance == -1 or (wallChance >= 40 and wallChance < 60)):
			self.frontier = True
			self.obstacle = False
		elif (wallChance < 40):
			self.obstacle = False
			self.frontier = False
		elif(wallChance >= 60 ):
			self.obstacle = True
			self.frontier = False 
			
	def calculateGCosts(self,start):
		#self.gCost = abs((start.x - self.x) * self.resolution) + abs((start.y - self.y) * self.resolution)
		if(self.parent == None):
			self.gCost = 0
		else:
			self.gCost = self.parent.gCost + self.resolution

	def calculateHCosts(self,goal):
		self.hCost = math.sqrt((((goal.x - self.x) * self.resolution) ** 2) + (((goal.y - self.y) * self.resolution) ** 2))

	def calculateFCosts(self):
		self.fCost = self.gCost+self.hCost

	def calcCosts(self,start, goal):
		self.calculateGCosts(start)
		self.calculateHCosts(goal)
		self.calculateFCosts()
