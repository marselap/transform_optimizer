#!/usr/bin/env python2
import numpy as np
import scipy.optimize
import scipy
import scipy.io
from tf.transformations import quaternion_matrix, quaternion_from_matrix, unit_vector
import time
import sys
import random
import csv

import tf2_ros

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from matplotlib import cm

cmaps = OrderedDict()
cmaps['Qualitative'] = ['Pastel1', 'Pastel2', 'Paired', 'Accent',
                        'Dark2', 'Set1', 'Set2', 'Set3',
                        'tab10', 'tab20', 'tab20b', 'tab20c']


class PlotPoints:
    def __init__(self, *args):

        if len(args) > 0:
            self.passed_arg = args[0]
        else:
            self.passed_arg = []
        pass




    def load_set(self, *args):
        if len(args) > 0:
            points = args[0]
            tfs = args[1]

        if isinstance(points, str):
            pts = np.loadtxt(points, delimiter=",")

            tf = []
            vec = []
            with open(tfs, "r+") as f:
                lines = f.readlines()
                for line in lines:
                    if "translation" in line:
                        if len(vec) > 0:
                            tf.append(vec)
                        vec = []
                    elif "rotation" in line:
                        pass
                    else:
                        val = line.split(':')[1]
                        vec.append(float(val))

            tf.append(vec)
            tf = np.asmatrix(tf)



        elif isinstance(points, list):
            print (np.shape(points))

            pts_temp = []
            for set in points:
                for row in set:
                    pts_temp.append(np.asarray(row))

            pts = np.asmatrix(pts_temp)
            tf = []
            tmp = []
            for t_ in tfs:
                trans = t_.transform.translation
                tmp.append(trans.x)
                tmp.append(trans.y)
                tmp.append(trans.z)
                trans = t_.transform.rotation
                tmp.append(trans.x)
                tmp.append(trans.y)
                tmp.append(trans.z)
                tmp.append(trans.w)
                tf.append(tmp)
            tf = np.asmatrix(tf)

        else:
            print("error in loading data")

        self.data = pts
        self.tfs = tf



    def plot_points(self):

        data = np.float32(self.data[4::5,:])


        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')


        for cmap_category, cmap_list in cmaps.items():
            print cmap_category, cmap_list
            # plot_color_gradients(cmap_category, cmap_list)


        # for 

        # ax.scatter(x, y, z, c='r', marker='o')




if __name__ == "__main__":


    pltobj = PlotPoints()
    pltobj.load_set("/home/marsela/libraries/cam_calib/camera_calib/scripts/poses.txt", "/home/marsela/libraries/cam_calib/camera_calib/scripts/tfs.txt")

    pltobj.plot_points()


    pass
