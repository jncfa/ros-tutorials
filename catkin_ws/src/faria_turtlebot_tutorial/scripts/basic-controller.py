#!/usr/bin/python

import rospy
import math

# adds locks for handling multithread data access
import threading

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from faria_turtlebot_tutorial.srv import GoalObjective, GoalObjectiveRequest, GoalObjectiveResponse


class TurtleController(object):
    """
    Basic turtle controller about ROS tutorials
    """
    
    # global class consts
    NODE_NAME='turtlebot_tut'
    DEF_TURTLE_NAME='/turtle1'
    DISTANCE_MAX_DIFF=0.05
    # initialize variables and setup pub/subs
    def __init__(self):
        
        print('spawning!')
        # grab params if they exist
        self.currentPose = rospy.get_param('~turtle_pose', Pose())
        self.turtleName = rospy.get_param('~turtle_name', TurtleController.DEF_TURTLE_NAME)
        self.linearVel = rospy.get_param('~linear_velocity', 1)
        self.angularVel = rospy.get_param('~angular_vel', 3)
        
        # setup subscribers and publishers
        self.velocityPublisher = rospy.Publisher(self.turtleName+'/cmd_vel', Twist, queue_size=10)
        self.poseSubscriber = rospy.Subscriber(self.turtleName+'/pose', Pose, self.poseCallback, queue_size=10)
        
        # setup service handlers
        self.goalSrvHandler = rospy.Service('headToGoal', GoalObjective, self.headToGoal)

        # spawn turtle on turtlesim
        #self.spawnTurtle()
        # set default goal
        self.running = True
        self.goal = GoalObjectiveRequest(x=3, y=3)

        # add lock for multi-thread access to pose
        self.poseLock = threading.Lock()
        self.goalLock = threading.Lock()

        # create timer for main thread execution
        self.timer = rospy.Timer(period=rospy.Duration(secs=0.01), callback=self.runCallback)
        

    # spawn turtle with given information (self.currentPose, self.turtleName)
    def spawnTurtle(self):
        spawnSrv = rospy.ServiceProxy('/spawn', Spawn)
        spawnInfo = Spawn(x=self.currentPose.x, y=self.currentPose.y, theta=self.currentPose.theta, name=self.turtleName)

        try:
            spawnSrv.call(spawnInfo)
        except:
            rospy.logerr("Error spawning turtle! Quitting...")
            rospy.signal_shutdown("Exception thrown on spawning turtle")
    rospy.is_shutdown
    
    # update pose based on turtlesim data
    def poseCallback(self, newPose):
        # lock while assigning new Pose 
        with self.poseLock:
            self.currentPose = newPose
            #print(self.currentPose)

    def headToGoal(self, goalMessage):

        # lock while setting new goal
        with self.goalLock:
            self.goal = goalMessage   

            # if not running, set to run
            self.running = True
            print("new goal ({},{})".format(self.goal.x, self.goal.y))

        return GoalObjectiveResponse(success = True)

    def calculateVelocity(self):

        # check if heading for goal
        vel = Twist()

        with self.poseLock, self.goalLock:
            currentPose = self.currentPose
            goal = self.goal

        # TODO: account for small perturbances (path taken is oscilating instead of just being a linear path)
        currentDistance = math.sqrt((goal.x - currentPose.x)**2 + (goal.y - currentPose.y)**2)
        currentTheta = math.atan2(goal.y - currentPose.y, goal.x - currentPose.x)    
        
        print(currentDistance, currentTheta, currentPose.theta, currentPose.x, currentPose.y)
        
        # set velocity depending on optimal trajectory (will reverse if angle > 90deg)
        vel.linear.x = self.linearVel * TurtleController.clamp1_min(currentDistance, 10*TurtleController.DISTANCE_MAX_DIFF) * TurtleController.clamp1(((math.pi/2) - abs(currentTheta - currentPose.theta))/(math.pi/2))     
        vel.angular.z = self.angularVel * TurtleController.clamp((currentTheta - currentPose.theta) / (math.pi), 0.01)

        if (currentDistance < TurtleController.DISTANCE_MAX_DIFF):
            with self.goalLock:
                self.running = False
            print('goal reached!')

        return vel

    def runCallback(self, _):
        if (self.running):
            self.velocityPublisher.publish(self.calculateVelocity())

    def start(self):
        if not self.timer.is_alive():
            self.timer.start()
        # else: implement error handling

    def stop(self):
        if self.timer.is_alive():
            self.timer.shutdown()
        # else: implement error handling

    @staticmethod
    def sigmoid(x):
        return 1 / (1 + math.exp(-x))

    @staticmethod
    def clamp1(x):
        return x if abs(x) < 1 else x / abs(x) 

    @staticmethod
    def clamp1_min(x, xmin):
        return (x / abs(xmin)) if abs(x) < abs(xmin) else x / abs(x)

    @staticmethod
    def clamp(x, xmin):
        return 0 if abs(x) < abs(xmin) else x / abs(x)

def main():
    # ros init
    rospy.init_node(TurtleController.NODE_NAME)
   
    # check turtlesim is up and sleep for a bit to make sure it's fully operational
    rospy.wait_for_service('/spawn')

    # create turtle and run loop
    turtle = TurtleController()
    turtle.start()

    # all async methods are setup, now we can spin
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("error")
        pass
    print("Exiting process...")
