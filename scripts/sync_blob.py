#!/usr/bin/env python3

import rospy

import numpy as np

import struct
import ctypes

import message_filters

from geometry_msgs.msg import PoseStamped

import matplotlib.pyplot as plt

import tf2_ros
from std_msgs.msg import Bool

import csv

class RsListener(object):
  def __init__(self):

    rospy.init_node('camera_calib_node')

    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)
    self.base_frame = "panda_link0"
    self.eef_frame = "panda_hand"
    self.eef_frame = "panda_link8"
    self.eef_frame = "panda_link9"

    self.pose = None
    self.tf = None
    self.flag_record = False

    pose_sub = rospy.Subscriber('mood/tracked_pose', PoseStamped, self.callback)

    rospy.sleep(2.0)

    # subscriber to topic announcing new pose for AT recording
    s1 = rospy.Subscriber('/calib/record_pose', Bool, self.record_new_pose)
    # save all saved AT positions and hand poses for camera/hand calibration
    s2 = rospy.Subscriber('/calib/optimize', Bool, self.poses_done)
    

    # self.pose_pub = rospy.Publisher('/pose', Float64MultiArray, queue_size=1)


    self.points = None
    self.tfs = None



  def record_new_pose(self, msg):
      try:
          self.tf = self.tfBuffer.lookup_transform(self.base_frame, self.eef_frame, rospy.Time())
          self.flag_record = True
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
          rospy.logwarn("Error looking up transform from base frame: ", self.base_frame, \
          " to eef frame: ", self.eef_frame, ". Check TF publisher.")
          self.flag_record = False

      print("did we get tf?")
      print(self.flag_record)
      # print(self.tf)

      # self.tfs = [self.tf]

  def poses_done(self, msg):
      print(self.points)
      print(self.tfs)

      with open("poses.txt", "w+") as f:
          for points in self.points:
              writer = csv.writer(f)
              writer.writerows([points])

      with open("tfs.txt", "w+") as f:
          for tf in self.tfs:
              f.write("translation: \n")
              t = tf.transform.translation
              f.write("x: "+ str(t.x) + "\n")
              f.write("y: "+ str(t.y)+ "\n")
              f.write("z: "+ str(t.z)+ "\n")
              f.write("rotation: \n")
              t = tf.transform.rotation
              f.write("x: "+ str(t.x)+ "\n")
              f.write("y: "+ str(t.y)+ "\n")
              f.write("z: "+ str(t.z)+ "\n")
              f.write("w: "+ str(t.w)+ "\n")
      self.points = None
      self.tfs = None



  def callback(self, msg):
      if self.flag_record:
          self.pose = msg
          


def main():

    listener = RsListener()
    rrate = rospy.Rate(100)

    try:
        while not rospy.is_shutdown():

            if listener.pose is not None:

                tmp_pose = listener.pose.pose.position
                points = [tmp_pose.x, tmp_pose.y, tmp_pose.z]

                print(points)

                if not listener.points:
                    listener.points = [points]
                    listener.tfs = [listener.tf]
                else:
                    listener.points.append(points)
                    listener.tfs.append(listener.tf)

                listener.tf = None
                listener.flag_record = False

                listener.pose = None


            else:

                rrate.sleep()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
  main()
