#!/usr/bin/env python

import rospy

import numpy as np
import cv2

from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2

# from pupil_apriltags import Detector
import struct
import ctypes

import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class RsListener(object):
  def __init__(self):

    rospy.init_node('camera_calib_node')

    # subs2 = rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self.pc_callback)

    # self.pose_pub = rospy.Publisher('/pose', Float64MultiArray, queue_size=1)

    self.corners = None
    self.centres = None
    self.pcmsg = None
    self.bridge = CvBridge()

    # self.detector = Detector()
    # tags = at_detector.detect(img)
    # tags[0].corners[0]
    # tags[0].centres
    self.flag = False


    image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
    pc_sub = message_filters.Subscriber('/camera/depth_registered/points', PointCloud2)

    ts = message_filters.TimeSynchronizer([image_sub, pc_sub], 10)
    ts.registerCallback(self.callback)




  def callback(self, image, pc):
  # Solve all of perception here...
    if not self.flag:
        self.pcmsg = pc
        self.imgmsg = image
        self.flag = True


def main():

    listener = RsListener()

    rrate = rospy.Rate(100)

    try:
        while not rospy.is_shutdown():

            if listener.pcmsg:
                w = listener.pcmsg.width
                h = listener.pcmsg.height

                i_w = 0
                j_h = 0

                pc = np.zeros((h,w,3))
                img = np.zeros((h,w,3))

                for point in pc2.read_points(listener.pcmsg):

                    try:
                        pc[j_h,i_w,0] = point[0]
                        pc[j_h,i_w,1] = point[1]
                        pc[j_h,i_w,2] = point[2]
                    except IndexError:
                        pass

                    # print(point[0])
                    test = point[3]
                    s = struct.pack('>f', test)
                    i = struct.unpack('>l', s)[0]
                    pack = ctypes.c_uint32(i).value
                    r = (pack & 0x00FF0000) >> 16
                    g = (pack & 0x0000FF00) >> 8
                    b = (pack & 0x000000FF)

                    try:
                        img[j_h,i_w,2] = r
                        img[j_h,i_w,1] = g
                        img[j_h,i_w,0] = b
                    except IndexError:
                        # print("//")
                        # print(j_h)
                        # print(h)
                        # print(i_w)
                        # print(w)
                        # print("........")
                        pass

                    # img[j_h,i_w,0] = r
                    # img[j_h,i_w,1] = g
                    # img[j_h,i_w,2] = b
                    if i_w < w - 1:
                        i_w += 1
                    else:
                        # print("/////")
                        # print(j_h)
                        # print(h)
                        # print(i_w)
                        # print(w)
                        i_w = 0
                        j_h += 1


                # print(np.shape(pc))
                cv_image = listener.bridge.imgmsg_to_cv2(listener.imgmsg, desired_encoding='passthrough')

                cv2.imshow("image", cv_image)
                cv2.waitKey()

                listener.pcmsg = None
            else:
                rrate.sleep()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
  main()
