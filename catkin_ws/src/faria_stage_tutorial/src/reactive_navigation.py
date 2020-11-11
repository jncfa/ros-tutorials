#!/usr/bin/python

from random import randint
import rospy
import math
import time 
import sys
import random

# adds locks for handling multithread data access
import threading

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ReactiveController(object):
    """
    Reactive controller for stageros simulation
    """
    
    NODE_NAME = "reactive_controller"
    def __init__(self):

        # grab params if they exist
        self.linearVel = rospy.get_param('~linear_velocity', 1)
        self.angularVel = rospy.get_param('~angular_vel', 3)

        # set variables for random direction after collision
        self._newCollision = True
        self._newCollisionTimer = rospy.get_time()

        # Create variables and respective locks
        self.obstacleDistanceLock = threading.Lock()
        self.obstacleDistance = 0
        self.robotStoppedLock = threading.Lock()
        self.robotStopped = False

        # setup subscribers and publishers
        self.velocityPub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.laserSub = rospy.Subscriber('/base_scan', LaserScan, self.laserCallback, queue_size=10)

        # setup main thread execution
        self.runTimer = rospy.Timer(period=rospy.Duration(0.01), callback=self.runCallback)
    
    def laserCallback(self, msg):
        """
        Callback for laser
        """
        
        with self.obstacleDistanceLock:
            #print (msg)
            self.obstacleDistance = min([x for x in msg.ranges if (x >= msg.range_min and x <= msg.range_max)])
            #rospy.loginfo("Min distance to obstacle {}".format(self.obstacleDistance))  

    def calculateVelocity(self):
        vel = Twist()
        
        # read variable and use it for execution
        with self.obstacleDistanceLock:
            obstacleDistance = self.obstacleDistance

        if (obstacleDistance > 0.5):
            vel.linear.x = self.linearVel
            self._newCollision = True
        else: 
            if self._newCollision:

                # flip sign based on random factor (two very close collision prob ~ 0, if two very far colisions -> prob = 1/2)
                self.angularVel = self.angularVel * (-1 if random.random() < 0.5*(1 - math.exp(0.5*(self._newCollisionTimer - rospy.get_time()))) else 1)

                # flag new collision and update timestamp
                self._newCollision = False
            
            self._newCollisionTimer = rospy.get_time()
            vel.angular.z = self.angularVel
            
        #print(vel)

        return vel

    def runCallback(self, _):
        # shallow copy and use it for execution
        with self.robotStoppedLock:
            robotStopped = self.robotStopped

        if not robotStopped:
            self.velocityPub.publish(self.calculateVelocity())

    def start(self):
        if not self.runTimer.is_alive():
            self.runTimer.start()
        # else: implement error handling

    def stop(self):
        if self.runTimer.is_alive():
            self.runTimer.shutdown()
        # else: implement error handling


def main():
    # ros init
    rospy.init_node(ReactiveController.NODE_NAME)
   
    # check stageros is up and sleep for a bit to make sure it's fully operational
    rospy.wait_for_service('/stage_ros/get_loggers')

    # create turtle and run loop
    controller = ReactiveController()
    controller.start()

    # all async methods are setup, now we can spin
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("error")
        pass
    print("Exiting process...")
