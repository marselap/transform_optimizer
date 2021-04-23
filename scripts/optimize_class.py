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

class Fminsearch:
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


    def do_optimise_T(self, t1, data = None):

        if data is None:
            # data = np.float32(self.data[4::5,:])
            data = self.data
        tf = np.asmatrix(self.tfs)


        data_pos = []
        [l, m] = np.shape(data)
        for row in data:
            # row.append(1.)
            new_row = []
            for i in row:
                new_row.append(i)
            new_row.append(1.)
            data_pos.append(np.asarray(new_row))
        data = np.asmatrix(data_pos)


        # data = data[(0,1,2,4,5),:]
        # tf = tf[(0,1,2,4,5),:]
        print data
        print tf

        objectiveFunLambda = lambda x: self.objectiveFunTransformation(x, data, tf)

        self.txs_optimized = data
        m,n = np.shape(data)

        print("")
        print ("Optimizing transform on " +str(m)+ " data points.")
        print("data:")
        print(data)
        start = time.time()
        topt = scipy.optimize.fmin(func=objectiveFunLambda, x0=t1)
        end = time.time()

        print ("---------------")
        print("Optimization duration: " + str(end - start) + " seconds.")
        print ("Optimized transform: " + str(topt))
        print("")
        return topt, self.objectiveFunTransformation(topt, data, tf)



    def objectiveFunTransformation(self, posquat, data_pos, data_tf):

        Tx = np.eye(4, dtype="float32")


        Tx = np.asmatrix(quaternion_matrix(posquat[3:]))
        Tx[0:3, 3] = np.resize(posquat[0:3], (3,1))

        [l, m] = np.shape(data_tf)
        Teef = np.zeros((l,4,4))
        for i in range(l):
            Teef[i,:,:] = np.asmatrix(quaternion_matrix(np.asarray(data_tf)[i,3:]))
            Teef[i,0:3,3] = np.asarray(data_tf)[i,0:3]

        [l, m] = np.shape(data_pos)
        f = 0
        
        
        for i in range(l):
            Tglob = np.dot(Teef[i,:,:], Tx)
            glob_1 = np.dot(Tglob, np.transpose(data_pos[i,:]))
            for j in range(l):
                Tglob = np.dot(Teef[j,:,:], Tx)
                glob = np.dot(Tglob, np.transpose(data_pos[j,:]))
                p = glob - glob_1
                f = f + np.linalg.norm(p[0:3])

        return f


    def transformInputData(self, posquat, data = None):
        if data is None:
            data = self.data
        data_tf = np.asmatrix(self.tfs)

        data_pos = []
        [l, m] = np.shape(data)
        for row in data:
            new_row = []
            for i in row:
                new_row.append(i)
            new_row.append(1.)
            data_pos.append(np.asarray(new_row))
        data_pos = np.asmatrix(data_pos)

        Tx = np.asmatrix(quaternion_matrix(posquat[3:]))
        Tx[0:3, 3] = np.resize(posquat[0:3], (3,1))

        [l, m] = np.shape(data_tf)
        Teef = np.zeros((l,4,4))
        for i in range(l):
            Teef[i,:,:] = np.asmatrix(quaternion_matrix(np.asarray(data_tf)[i,3:]))
            Teef[i,0:3,3] = np.asarray(data_tf)[i,0:3]

        [l, m] = np.shape(data_pos)

        for i in range(l):
            Tglob = np.dot(Teef[i,:,:], Tx)
            glob_1 = np.dot(Tglob, np.transpose(data_pos[i,:]))
            self.txs_optimized[i,0:3] = np.transpose(glob_1[0:3])


    def dissipation(self, t1, data = None):

        if data is None:
            #if working with sync - AR tags - only optimize on central point. 
            # data = np.float32(self.data[4::5,:])
            data = np.float32(self.data[:,:])
        tf = np.asmatrix(self.tfs)

        data_pos = []
        [l, m] = np.shape(data)
        for row in data:
            # row.append(1.)
            new_row = []
            for i in row:
                new_row.append(i)
            new_row.append(1.)
            data_pos.append(np.asarray(new_row))
        data = np.asmatrix(data_pos)

        return self.objectiveFunTransformation(t1, data, tf)



    def plot_inputs(self, ax1, col = 'k'):

        mtrx = self.txs_optimized.transpose()
        pos_vec = []
        m,n = np.shape(mtrx)
        print (m,n)
        for i in range(n):
            m = mtrx[0:3,i]
            # pos = m[0:3]
            # quat = quaternion_from_matrix(m)
            # pos_quat = []
            # for p in pos:
            #     pos_quat.append(p)
            # for q in quat:
            #     pos_quat.append(q)
            # p_vec = pos_q_2_pos_vec(pos_quat)
            # pos_vec.append(p_vec)
            pos_vec.append(m)

        cm = np.asarray(pos_vec)
        ax1.scatter(cm[:,0], cm[:,1], cm[:,2], color=col)

if __name__ == "__main__":


    optim = Fminsearch()
    optim.load_set("./poses_ex.txt", "./tfs_ex.txt")


    t1 = np.asarray([-0.033,-0.017,0.075,0,0,0.7068252, 0.7073883])
    # t1 = np.asarray([0.03,0,0.08,0,0,0., 1.])
    # t1 = np.asarray([0,0,0,0,0,0,1])
    topt, f = optim.do_optimise_T(t1)

    q = topt[3:]
    q /= np.linalg.norm(q)

    topt[3:] = q

    print(topt)

    print("dissipation with T used:")
    print(optim.dissipation(t1)/30.)
    print("dissipation with T optimized:")
    print(optim.dissipation(topt)/30.)

    t1 = np.asarray([0,0,0,0,0,0,1])
    topt, f = optim.do_optimise_T(t1)
    q = topt[3:]
    q /= np.linalg.norm(q)
    topt[3:] = q
    print(topt)

    print("dissipation with T zero:")
    print(optim.dissipation(t1)/30.)
    print("dissipation with T optimized:")
    print(optim.dissipation(topt)/30.)

    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    optim.transformInputData(topt)
    optim.plot_inputs(ax)
    plt.show()

    pass