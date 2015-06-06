#!/usr/bin/env python
#Author: Jack Yang
#A random obstacle generator that uses the 

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import rospy
import math
import random

topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray)

markerArray = MarkerArray()

#Use point (0,0,0) as a starting point to create markers in 3D space

#Set number of boxes and the area that intends to generate the obstacles
step = 4
numOfBoxes = 4

#Define a range for obstacle's width, length, and height
widthRange = 2
lengthRange = 2
heightRange = 3

def main():
    rospy.init_node('register')
    boxList = randGenerator()
    markerBox = visualizeBox(boxList)
    while not rospy.is_shutdown():
        publisher.publish(markerBox)
        rospy.sleep(0.01)

class box:
    def __init__(self, x, y, z, length, width, height):
        self.length = length
        self.width = width
        self.height = height
        self.x_cor = x
        self.y_cor = y
        self.z_cor = z

    def __repr__(self):
        return "(" + str(self.x_cor) + "," + str(self.y_cor) + ","\
               + str(self.z_cor) + "," + str(self.length) + "," +\
               str(self.width) + "," + str(self.height) + ")"

    def volume(self):
        return self.x_cor*self.y_cor*self.z_cor

    def checkCollide(self, box2):
        """Check if box1 collides with box2. If they do not collide, return true.
        If they collide, return false."""
        xPos = box2.x_cor
        yPos = box2.y_cor
        zPos = box2.z_cor
        secLen = box2.length
        secWidth = box2.width
        secHeight = box2.height

        if xPos < self.x_cor - secLen or xPos > self.x_cor + self.length:
            return True
        elif yPos < self.y_cor - secWidth or yPos > self.y_cor + self.width:
            return True
        elif zPos < self.z_cor - secHeight or zPos > self.z_cor + self.height:
            return True
        else:
            return False

    #point is represented by a list (x, y, z)
    def checkPointCollide(self, point):
        """check if a point collide with the box. If they do not collide, return
            true. If they collide, return false."""
        
        



def randGenerator( center = [0,0,0] ):   #center should be a list of [x, y, z]
    """randGenerator takes in a center point and return the corresponding number of markers
    of random markers within the range that is defined globally."""
    #Initialize a list of points
    boxList = []
    cent_x = center[0]
    cent_y = center[1]
    cent_z = center[2]
    i = 0
    while i <= numOfBoxes:
        x = random.uniform(-step, step)
        y = random.uniform(-step, step)
        z = random.uniform(0, step)
        width = random.uniform(0, widthRange)
        length = random.uniform(0, lengthRange)
        height = random.uniform(0, heightRange)
        #generate the box object
        Box = box(x, y, z, width, length, height)
        if len(boxList) == 0:
            boxList.append(Box)
            i += 1
        else:
        
            collideList = map(Box.checkCollide, boxList)
            notCollide = reduce(lambda x, y: x and y, collideList)
            if notCollide == True:
                #append the box object into the box list
                boxList.append(Box)
                i += 1
    return boxList




def visualizeBox(boxList):
    """visualizeBox put the box object generated from randGenerator to ros
    marker and put them into the marker array"""
    for i in range(len(boxList)):
        Box = boxList[i]
        marker = Marker()
        marker.header.frame_id = "world"
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.scale.x = Box.length
        marker.scale.y = Box.width
        marker.scale.z = Box.height
        marker.color.a = 1
        marker.color.r = 0.82
        marker.color.g = 0.41
        marker.color.b = 0.12
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = Box.x_cor + Box.length/2
        marker.pose.position.y = Box.y_cor + Box.width/2
        marker.pose.position.z = Box.z_cor + Box.height/2
        marker.id = i
        markerArray.markers.append(marker)
    return markerArray


if __name__ == "__main__":
    main()
    
