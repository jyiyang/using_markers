#!/usr/bin/env python
#Filename: Cubes
#Author: Jack Yang
#This script generates a simulated environment of blocks and paths for further
# development of quadcopter movement in a 3D space. Currently the list of nodes and
# blocks, and pathes are not able to change their current position.

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
import rospy
import math

topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray)

rospy.init_node('register')

markerArray = MarkerArray()

#Set stepSize for nodes
stepSize = 1.5

#Initialize an empty list to store all intersection nodes
nodeList = []

#Create the cube markers and put them into the publishing marker array
for i in range(-4, 4):
   for j in range (-4, 4):
      for k in range(0, 4):
         markerC = Marker()
         markerC.header.frame_id = "world"
         markerC.type = markerC.CUBE
         markerC.action = markerC.ADD
         markerC.scale.x = 0.6
         markerC.scale.y = 0.6
         markerC.scale.z = 0.6
         markerC.color.a = 1.0
         markerC.color.r = 0.82
         markerC.color.g = 0.41
         markerC.color.b = 0.12
         markerC.pose.orientation.w = 1.0
         markerC.pose.position.x = stepSize * i
         markerC.pose.position.y = stepSize * j
         markerC.pose.position.z = 2 * k
         markerArray.markers.append(markerC)

#Create the line marker and put them into marker array
for i in range(-3,4):
   for j in range(0, 7):
      point1 = Point()
      point1.x = stepSize * i - 0.75
      point1.y = -6
      point1.z = j
      point2 = Point()
      point2.x = stepSize * i - 0.75
      point2.y = 6
      point2.z = j
      line = Marker()
      line.header.frame_id = "world"
      line.type = line.LINE_STRIP
      line.action = line.ADD
      line.scale.x = 0.1
      line.scale.y = 0.1
      line.scale.z = 0.1
      line.color.a = 1.0
      line.color.r = 0.2
      line.color.g = 0.5
      line.color.b = 0.64
      line.points = (point1, point2)

      markerArray.markers.append(line)

#Create the line markers and put them into the publishing marker array
for i in range(-3,4):
   for j in range(0, 7):
      point1 = Point()
      point1.x = -6
      point1.y = stepSize * i - 0.75
      point1.z = j
      point2 = Point()
      point2.x = 6
      point2.y = stepSize * i - 0.75
      point2.z = j
      line = Marker()
      line.header.frame_id = "world"
      line.type = line.LINE_STRIP
      line.action = line.ADD
      line.scale.x = 0.1
      line.scale.y = 0.1
      line.scale.z = 0.1
      line.color.a = 1.0
      line.color.r = 0.6
      line.color.g = 0.5
      line.color.b = 0.63
      line.points = (point1, point2)

      markerArray.markers.append(line)

#Create the line markers and put them into the publishing marker array
for i in range(-3, 4):
   for j in range(-3, 4):
      point1 = Point()
      point1.x = stepSize * j - 0.75
      point1.y = stepSize * i - 0.75
      point1.z = 0
      point2 = Point()
      point2.x = stepSize * j - 0.75
      point2.y = stepSize * i - 0.75
      point2.z = 6
      line = Marker()
      line.header.frame_id = "world"
      line.type = line.LINE_STRIP
      line.action = line.ADD
      line.scale.x = 0.1
      line.scale.y = 0.1
      line.scale.z = 0.1
      line.color.a = 1.0
      line.color.r = 0.3
      line.color.g = 0.72
      line.color.b = 0.63
      line.points = (point1, point2)

      markerArray.markers.append(line)

#Put all nodes into the list
for i in range(-3, 4):
   for j in range(-3, 4):
      for k in range(0, 6):
         node = [stepSize * i - 0.75, stepSize * j - 0.75, k]
         #print node
         nodeList.append(node)



# Renumber the marker IDs
id = 0
for m in markerArray.markers:
   m.id = id
   id += 1


# Publish the MarkerArray
while not rospy.is_shutdown():
   publisher.publish(markerArray)

   rospy.sleep(0.01)


