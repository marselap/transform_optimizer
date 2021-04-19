#!/usr/bin/env python3

import rospy

import numpy as np
import cv2

from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2

from pupil_apriltags import Detector
import struct
import ctypes

import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

import matplotlib.pyplot as plt

from ros_numpy import point_cloud2 as pc2_np

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

    self.pcmsg = None
    self.imgmsg = None
    self.tf = None

    self.bridge = CvBridge()

    self.detector = Detector()

    # image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
    image_sub = message_filters.Subscriber('/camera/color/image_rect_color', Image)
    pc_sub = message_filters.Subscriber('/camera/depth_registered/points', PointCloud2)

    rospy.sleep(2.0)

    # subscriber to topic announcing new pose for AT recording
    s1 = rospy.Subscriber('/record_apriltag', Bool, self.record_new_pose)
    # save all saved AT positions and hand poses for camera/hand calibration
    s2 = rospy.Subscriber('/apriltags_done', Bool, self.poses_done)
    self.flag_record = False

    # self.pose_pub = rospy.Publisher('/pose', Float64MultiArray, queue_size=1)

    ts = message_filters.TimeSynchronizer([image_sub, pc_sub], 10)
    ts.registerCallback(self.callback)

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
              writer.writerows(points)

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



  def callback(self, image, pc):
      if self.flag_record:
          self.pcmsg = pc
          self.imgmsg = image
          # self.flag_record = False


def main():

    listener = RsListener()
    rrate = rospy.Rate(100)

    try:
        while not rospy.is_shutdown():

            if listener.pcmsg is not None:

                cv_image = listener.bridge.imgmsg_to_cv2(listener.imgmsg, desired_encoding='passthrough')
                gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

                tags = listener.detector.detect(gray)
                if len(tags) > 0:
                # try:
                    corners = np.transpose(tags[0].corners)

                    # print("found apriltags:")
                    # print("no: ", len(tags))
                    # print(tags)

                    # print(corners)

                    x = corners[0,:]
                    y = corners[1,:]

                    x = [int(temp) for temp in x]
                    y = [int(temp) for temp in y]
                    cx = int(tags[0].center[0])
                    cy = int(tags[0].center[1])
                    x.append(cx)
                    y.append(cy)


                    points = []
                    pcarray = pc2_np.pointcloud2_to_array(listener.pcmsg)

                    points = []
                    for ix,iy in zip(x, y):
                        p = []
                        ptem = pcarray[iy, ix]
                        for idim in range(3):
                            p.append(ptem[idim])
                        # p = pcarray[iy, ix][0:3]
                        points.append(p)

                    print(points)

                    if not listener.points:
                        listener.points = [points]
                        listener.tfs = [listener.tf]
                    else:
                        listener.points.append(points)
                        listener.tfs.append(listener.tf)

                    listener.tf = None
                    listener.flag_record = False

                # except IndexError:
                else:
                    listener.flag_record = True
                    pass

                listener.pcmsg = None

                listener.imgmsg = None

            else:

                rrate.sleep()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
  main()
