#!/usr/bin/env python

from Node import Node
from random import randint
import rospy, tf, numpy, math


if __name__ == '__main__':

	rospy.init_node('Mag_Men_Test')

	Nodes = []

	x=10

	while(x >= 0):
		Nodes.append(Node(randint(0,10), randint(0,10), 30))
		x -= 1

	x = 10
	while(x >= 0):
		Nodes[x].calcCosts(Nodes[0], Nodes[9])
		x -= 1

	x = len(Nodes) - 1
	while(x >= 0):
		print "{}".format(Nodes[x].fCost)
		x -= 1

	print ""

	Nodes = sorted(Nodes, key=lambda node: node.fCost, reverse = True)

	x = len(Nodes) - 1
	while(x >= 0):
		print "{}".format(Nodes[x].fCost)
		x -= 1
	print ""