#!/usr/bin/env python

import rospy
# import scipy.signal as signal

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, PoseArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix

from vd_msgs.msg import AckermannDriveStamped
from vd_msgs.msg import AckermannDrive
from vd_msgs.msg import GpsInfo

import atexit
import os
import sys

import math
import numpy
from numpy import genfromtxt
from tf.transformations import euler_from_quaternion, quaternion_from_euler, random_vector
import csv

SEARCH_INTERVAL = 1000

class RtkPlayer(object):
    """
    rtk player class
    """
    def __init__(self, record_file, replan = 'F', completepath = 'F'):
        """Init player."""
        self.firstvalid = False
        print("Load record file from: %s" % record_file)

        try:
            self.file_handler = open(record_file, 'r')
        except:
            print("Cannot find file: " + record_file)
            self.file_handler.close()
            sys.exit()

        # read
        self.data = genfromtxt(self.file_handler, delimiter=',', names=True)
        self.file_handler.close()
        self.localization = NavSatFix()
        self.chassis = gps_status()
        self.odometry = Odometry()
        self.goal = 1
        self.goal_received = False
        self.odometry_received = False
        self.chassis_received = False
        self.localization_received = False

        self.terminating = False    
        self.sequence_num = 0

        self.speedmultiplier = 1

        self.replan = (replan == 't')
        self.completepath = (completepath == 't')

        self.cartime = rospy.get_time()
        self.carx = 0 
        self.cary = 0
        self.carz = 0
        self.carspeed = 0
        self.carspeedVx = 0
        self.carspeedVy = 0
        self.carspeedVz = 0
        self.carthetax = 0
        self.carthetay = 0
        self.carthetaz = 0
        self.flatx = 0
        self.flaty = 0
        self.flatz = 0

        # filters 
        # b, a = signal.butter(6, 0.05, 'low')
        # self.data['speed'] = signal.filtfilt(b, a, self.data['speed'])

        self.start = 0
        self.end = 0
        self.closestpoint = 0
        self.goal = rospy.Subscriber('/gps/data', NavSatFix, self.goal_callback, queue_size=1)
        self.planning_pub = rospy.Publisher('/move_base/VEDDHAPlannerROS/global_plan', Path, queue_size=1)
        self.odom_pub = rospy.Publisher('/odom/by_rtk', Odometry, queue_size=1)
        print("Planning Ready")

    def goal_callback(self, data):
        #self.chassis.CopyFrom(data)
        self.goal = data
        self.goal_received = True
        
    def chassis_callback(self, data):
        #self.chassis.CopyFrom(data)
        self.chassis = data
        self.chassis_received = True
        
    def odometry_callback(self, data):
        # self.chassis.CopyFrom(data)
        # print('odem:', data.pose.pose.position)
        self.odometry = data
        self.odometry_received = True

    def localization_callback(self, data):
        #self.localization.CopyFrom(data)
        self.localization = data
        self.cartime = self.localization.header.stamp
        self.carx = self.localization.latitude 
        self.cary = self.localization.longitude
        self.carz = self.localization.altitude
        self.carspeed = self.chassis.Vv
        self.carspeedVx = self.chassis.Vx
        self.carspeedVy = self.chassis.Vy
        self.carspeedVz = self.chassis.Vz
        self.carthetax = self.chassis.Heading
        self.carthetay = self.chassis.Pitch
        self.carthetaz = self.chassis.Roll
        self.flatx = self.odometry.pose.pose.position.x
        self.flaty = self.odometry.pose.pose.position.y
        self.flatz = self.odometry.pose.pose.position.z
        self.localization_received = True

    def restart(self):
        print("before replan self.start=%s, self.closestpoint=%s" % (self.start, self.closestpoint))
        self.closestpoint = self.closest_dist()
        self.start = max(self.closestpoint - 100, 0)
        self.starttime = rospy.get_time()
        self.end = min(self.start + 1000, len(self.data) - 1)
        print("finish replan at time %s, self.closestpoint=%s" % (self.starttime, self.closestpoint))

    def closest_dist(self):
        shortest_dist_sqr = float('inf')
        print("before closest self.start=%s" % (self.start))
        search_start = max(self.start - SEARCH_INTERVAL / 2, 0)
        search_end = min(self.start + SEARCH_INTERVAL / 2, len(self.data))
        start = self.start

        for i in range(search_start, search_end):
            dist_sqr = (self.flatx - self.data['x'][i]) ** 2 + \
                   (self.flaty - self.data['y'][i]) ** 2
            if dist_sqr <= shortest_dist_sqr:
                start = i
                shortest_dist_sqr = dist_sqr

        print("after closest self.start=%d" % (start))
        return start

    def closest_time(self):
        time_elapsed = rospy.get_time() - self.starttime
        closest_time = self.start
        time_diff = self.data['time'][closest_time] - \
           self.data['time'][self.closestpoint]

        while time_diff < time_elapsed and closest_time < (len(self.data) - 1):
            closest_time = closest_time + 1
            time_diff = self.data['time'][closest_time] - \
                self.data['time'][self.closestpoint]

        return closest_time

    def publish_planningmsg(self):
        """
        Generate New Path
        """
        if not self.localization_received:
            print("locaization not received yet when publish_planningmsg")
            return

        if not self.odometry_received:
            print("odemetry not received yet when publish_planningmsg")
            return

        planningdata = Path()
        odomdata = Odometry()
        now = rospy.get_rostime()
        planningdata.header.stamp = now
        # planningdata.header.frame_id = "map"
        planningdata.header.frame_id = "odom"
        planningdata.header.seq = self.sequence_num
        self.sequence_num = self.sequence_num + 1  
        
        print(
            "publish_planningmsg: before adjust start: self.start = %s, self.end=%s"
            % (self.start, self.end))

        if self.replan or self.sequence_num <= 1:
            print(
                "trigger replan: self.replan=%s, self.sequence_num=%s"
                % (self.replan, self.sequence_num))
            self.restart()
        else:
            timepoint = self.closest_time()
            distpoint = self.closest_dist()
            self.start = max(min(timepoint, distpoint) - 100, 0)
            self.end = min(max(timepoint, distpoint) + 900, len(self.data) - 1)

            xdiff_sqr = (self.data['x'][timepoint] - self.flatx)**2
            ydiff_sqr = (self.data['y'][timepoint] - self.flaty)**2
            if xdiff_sqr + ydiff_sqr > 4.0:
                self.restart()

        if self.completepath:
            self.start = 0
            self.end = len(self.data) - 1

        print(
            "publish_planningmsg: after adjust start: self.start = %s, self.end=%s"
            % (self.start, self.end))

        for i in range(self.start, self.end):
            # geometry_msgs/Odometry
            adc_point = PoseStamped()
            adc_point.header.stamp = now
            adc_point.header.frame_id = "map"
            # adc_point.header.frame_id = "odom"
            adc_point.pose.position.x = self.data['x'][i]
            adc_point.pose.position.y = self.data['y'][i]
            adc_point.pose.position.z = self.data['z'][i]
            path_yaw = self.data['theta'][i]
            path_pitch = self.data['pitch'][i]
            path_roll = self.data['roll'][i]
            # trans quaternion_from_euler
            # q = transformations.quaternion_from_euler(0.0, 0.0, th/180.0*pi)
            path_quaternion = quaternion_from_euler(path_roll, path_pitch, path_yaw)
            adc_point.pose.orientation.x = path_quaternion[0]
            adc_point.pose.orientation.y = path_quaternion[1]
            adc_point.pose.orientation.z = path_quaternion[2]
            adc_point.pose.orientation.w = path_quaternion[3]
            
            # adc_point.twist.twist.linear.x = self.data['speed'][i]
            # adc_point.twist.twist.angular.z = self.data['theta'][i]
            # time_diff = self.data['time'][i] - \
            #     self.data['time'][self.closestpoint]
            # adc_point.relative_time = time_diff / self.speedmultiplier - (
            #     now - self.starttime)
            
            planningdata.poses.extend([adc_point])
        # planningdata.total_path_length = self.data['s'][self.end] - \
        #     self.data['s'][self.start]
        # planningdata.total_path_time = self.data['time'][self.end] - \
        #     self.data['time'][self.start]
        # planningdata.engage_advice.advice = \
        #     drive_state_pb2.EngageAdvice.READY_TO_ENGAGE

        self.planning_pub.publish(planningdata)
        # print("Generated Planning Sequence: " + str(self.sequence_num - 1))

    def shutdown(self):
        self.terminating = True
        print("Shutting Down...")
        self.file_handler.close()
        rospy.sleep(0.1)

    def quit(self, signum, frame):
        """
        shutdown the keypress thread
        """
        sys.exit()

def main():
    """
    Main rosnode
    """
    print('rtk_player start...')
    rospy.init_node('rtk_player')

    log_dir = os.path.dirname(os.path.abspath(__file__)) + "/../../../data"
    record_file = log_dir + "/garage.csv"

    player = RtkPlayer(record_file)
    atexit.register(player.shutdown)

    rospy.Subscriber('/gps/status', 
                     gps_status, 
                     player.chassis_callback, queue_size=1)
    rospy.Subscriber('/gps/data', 
                     NavSatFix, 
                     player.localization_callback, queue_size=1)
    rospy.Subscriber('/odom/by_gps', 
                     Odometry, 
                     player.odometry_callback, queue_size=1)

    rate = rospy.Rate(100)
    rate.sleep()
    while not rospy.is_shutdown():
        player.publish_planningmsg()
        rate.sleep()
        
    rospy.spin()

if __name__ == '__main__':
    main()
