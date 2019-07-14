#!/usr/bin/env python
import numpy as np
import rospy
import math
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")

    def __init__(self):
        # Initialize your publishers and
        # subscribers
        self.data = None    
        self.angle = 0
        self.cmd = AckermannDriveStamped()
        self.laser_sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.scan, queue_size=1)
        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)
    
    def scan(self, data):
    #stores the lidar data so you can work with it
        self.data = data

    #calls function that controls driving
        self.drive()
    
    def drive(self):
    #controls driving
    #gets the angle required
    #    self.angle = self.find_wall()
    #sets speed and driving angle

        physvects = self.convertToForce(self.data.ranges)

        yval = (self.convertPoints(physvects))[1]
        xval = (self.convertPoints(physvects))[0]

        print("y = " + str(yval))
        print("x = " + str(xval))
        print("mag = " + str(math.sqrt(yval**2 + xval**2)))
        print("deg = " + str(math.tan(yval/xval)))

        #self.cmd.drive.speed = math.sqrt(yval**2 + xval**2) * 0.01
        self.cmd.drive.speed = (yval * 0.01)
        #- is right and + is left for some reason in this
        #self.cmd.drive.steering_angle = (-1 * math.atan(yval/xval))
        self.cmd.drive.steering_angle = (-1 * xval)

        #publishes the command
        self.drive_pub.publish(self.cmd)

    def convertToForce(self, points):
        physvects = []
        #convert lidar data to repulsion force

        #use physics force equation
        for point in points:
            physvects.append(1/(point))
        
        #should be returning a list
        return physvects

    def convertPoints(self, points):
        #find net force
        #imagine xy plane, racecar is facing upwards, 0 degrees is east

        #x coordinates of vectors
        xvals = []
        #y coordinates of vectors
        yvals = []

        #converting to float so it doesn't concatenate
        pointsLength = float(len(points))

        #calculating x and y coordinates of lidar data via magnitude and angle
        for i in range(len(points)):

            iFloat = float(i)
            print(i)
            print("i radians: " + str(math.radians(iFloat/pointsLength * 270 - 45)))
            xvals.append(points[i] * (math.cos(math.radians(iFloat/pointsLength * 270 - 45))))
            yvals.append(points[i] * (math.sin(math.radians(iFloat/pointsLength * 270 - 45))))
        
        #initializing xtot and ytot
        xtot = 0
        ytot = 0

        #xval controls steering
        for xval in xvals:
            xtot = xtot + xval

        #yval controls speed
        for yval in yvals:
            ytot = ytot + yval

        #right now this is simulating as if the wall is 
        #attracting the car so we need to make it negative
        yval = yval * -1
        xval = xval * -1

        #gives the car a strong forwards velocity
        yval = yval + 10

        #returning both net yforce and xforce
        coord = (xval, yval)
        return coord

    #disclaimer: the variables and comments are backwards
    #def find_wall(self):
    # if lidar data has not been received, do nothing

    #    turnAngle = 0
    #under ideal conditions, turnAngle is 0
    
    # follow on left side
    #    if self.SIDE == -1:
    #        dataside = len(self.data.ranges)*5/6-1
    #        datadiagonal = len(self.data.ranges)*4/6-1
    #    # follow on right side
    #    elif self.SIDE == 1:
    #        dataside = len(self.data.ranges)*1/6-1
    #        datadiagonal = len(self.data.ranges)*2/6-1
            
    #    left = self.data.ranges[dataside]
    #    e1 = left - self.DESIRED_DISTANCE
    #    forwardleft = self.data.ranges[datadiagonal]
    #    e2 = forwardleft - (self.DESIRED_DISTANCE ** (1/2))
    #e2 goes against DESIRED_DISTANCE * sqrt(2) because of 45 45 90 triangle formula
    #in ideal conditions e1 and e2 should be zero (perfect distance, heading straight)
    
    #positive turnAngle should be towards the wall, negative should be away
    
        #car is farther than desired distance, need to turn towards wall
    #    if e1 > 0 and e2 > 0:
    #        turnAngle = 1
        #car is pointed towards wall, need to turn away from wall
    #    elif e1 > 0 and e2 < 0:
    #        turnAngle = -0.5
        #car is pointed away from wall, needs to turn towards wall
    #    elif e1 < 0 and e2 > 0:
    #        turnAngle = 0.5
        #car is too close to wall, need to turn away from wall
    #    elif e1 < 0 and e2 < 0:
    #        turnAngle = -1

        #if none of the if or elif statements are run, turnAngle stays 0, ideal situation
    #    return turnAngle    
        #turnAngle is the angle which the car will turn by


"""Lidar data is now stored in self.data, which can be accessed
using self.data.ranges (in simulation, returns an array).
Lidar data at an index is the distance to the nearest detected object
self.data.ranges[0] gives the leftmost lidar point
self.data.ranges[100] gives the rightmost lidar point
self.data.ranges[50] gives the forward lidar point
"""

#returns the output of your alg, the new angle to drive in

if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
rospy.spin()
