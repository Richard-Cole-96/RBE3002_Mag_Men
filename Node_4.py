#!/usr/bin/env python
import math

class Node:

	def __init__(self, x, y, wallChance):
		self.x = x
		self.y = y 
		self.gCost = 0
		self.hCost = 0
		self.fCost = 0
		self.parent = None
		self.visited = False
		self.inNotVisited = False
		self.padding = False
		self.costMap = 0
		if (wallChance < 50):
			self.obstacle = False
		elif(wallChance > 50):
			self.obstacle = True
		elif(wallChance == 50):
			self.obstacle = None
			
	def calculateGCosts(self,start):
		#self.gCost = abs((start.x - self.x) * .05) + abs((start.y - self.y) * .05)
		if(self.parent == None):
			self.gCost = 0
		else:
			self.gCost = self.parent.gCost + .05

	def calculateHCosts(self,goal):
		self.hCost = math.sqrt((((goal.x - self.x) * .05) ** 2) + (((goal.y - self.y) * .05) ** 2))

	def calculateFCosts(self):
		self.fCost = self.gCost+self.hCost

	def calcCosts(self,start, goal):
		self.calculateGCosts(start)
		self.calculateHCosts(goal)
		self.calculateFCosts()
