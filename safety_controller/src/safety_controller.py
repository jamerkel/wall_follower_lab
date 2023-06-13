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
    VELOCITY = 1#rospy.get_param("wall_follower/velocity")

    def __init__(self):
        #Initialize publisher and subscriber data

        self.current_command = None
        self.current_scan = None
        self.will_crash = False

        self.safety_pub = rospy.Publisher(self.SAFETY_TOPIC, AckermannDriveStamped, queue_size=10)

    def command_callback(self, msg):
        #store most recent drive command published
        self.current_command = msg
        self.check_safety()

    def scan_callback(self, msg):
        #store most recent scan published
        self.current_scan = msg
        self.check_safety()

    def check_safety(self):
        #check if robot will crash
            #TODO
        dist = np.array(self.current_scan.ranges)
        angles = np.linspace(self.current_scan.angle_min, self.current_scan.angle_max, num=len(self.current_scan.ranges))
        d_theta = (angles[-1]-angles[0])/len(angles)
        weight = np.reciprocal(dist)
        

        if min(dist) < self.SAFE_DISTANCE:
            self.will_crash = True

        if self.will_crash == True:

            #publish override
            override = AckermannDriveStamped()
            override.header.stamp = rospy.Time.now()
            override.drive.speed = self.VELOCITY
            override.drive.steering_angle = 0 #TODO calculate avoidance angle
            self.safety_pub.publish(override)

if __name__ == "__main__":
    rospy.init_node('safety_controller')
    safety_controller = SafetyController()

    rospy.Subscriber(safety_controller.DRIVE_TOPIC, AckermannDriveStamped, safety_controller.command_callback)
    rospy.Subscriber(safety_controller.SCAN_TOPIC, LaserScan, safety_controller.scan_callback)

    rospy.spin()


    
