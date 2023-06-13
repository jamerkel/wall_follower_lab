#!/usr/bin/env python2

import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class SafetyController:
    #set class parameters
    DRIVE_TOPIC = rospy.get_param("safety_controller/drive_topic")
    SCAN_TOPIC = rospy.get_param("safety_controller/scan_topic")
    SAFETY_TOPIC = rospy.get_param("safety_controller/safety_topic")
    SAFE_DISTANCE = rospy.get_param("safety_controller/safe_distance")
    VELOCITY = rospy.get_param("safety_controller/velocity")

    def __init__(self):
        #Initialize publisher and subscriber data

        self.will_crash = False
        
        self.scan_sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.check_safety)
        self.safety_pub = rospy.Publisher(self.SAFETY_TOPIC, AckermannDriveStamped, queue_size=10)

    def check_safety(self, msg):
        #check if robot will crash
            #TODO
        dist = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, num=len(dist))
        d_theta = (angles[-1]-angles[0])/len(angles)
        weight = np.reciprocal(dist)
        front = np.split(dist, 5)[2]
        

        if min(dist) < self.SAFE_DISTANCE:
            self.will_crash = True

        if self.will_crash == True:

            #publish override
            override = AckermannDriveStamped()
            override.header.stamp = rospy.Time.now()
            override.drive.speed = 0#self.VELOCITY
            override.drive.steering_angle = 0 #TODO calculate avoidance angle
            self.safety_pub.publish(override)

if __name__ == "__main__":
    rospy.init_node('safety_controller')
    safety_controller = SafetyController()

    rospy.spin()


    
