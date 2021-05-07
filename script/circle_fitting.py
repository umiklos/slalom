#!/usr/bin/env python

import rospy
import numpy as np
import visualization_msgs.msg as vismsg


def callback_detectedobjects(data):
    polygon_list=[]
    

    for i in range (len(data.markers)):
        polygon_data=np.zeros((len(data.markers[i].points),2))
        for j in range(len(data.markers[i].points)):
            polygon_data[j,0]=data.markers[i].points[j].x
            polygon_data[j,1]=data.markers[i].points[j].y
        polygon_list.append(polygon_data)
    polygon_list=np.array(polygon_list)    
    
    a=np.zeros((len(polygon_list),3))
    
    for i in range(len(a)):
        xc,yc,r=fit_circle_2d(polygon_list[i][:,0],polygon_list[i][:,1])
        a[i,0]=xc
        a[i,1]=yc
        a[i,2]=r

    circles=np.intersect1d(np.where(a[:,2] < 1.5),np.where(a[:,2] > 0.75))
    
    
    
def fit_circle_2d(x, y, w=[]):
    
    A = np.array([x, y, np.ones(len(x))]).T
    b = x**2 + y**2
    
    # Modify A,b for weighted least squares
    if len(w) == len(x):
        W = np.diag(w)
        A = np.dot(W,A)
        b = np.dot(W,b)
    
    # Solve by method of least squares
    c = np.linalg.lstsq(A,b)[0]
    
    # Get circle parameters from solution c
    xc = c[0]/2
    yc = c[1]/2
    r = np.sqrt(c[2] + xc**2 + yc**2)
    return xc, yc, r

def rodrigues_rot(P, n0, n1):
    
    # If P is only 1d array (coords of single point), fix it to be matrix
    if P.ndim == 1:
        P = P[np.newaxis,:]
    
    # Get vector of rotation k and angle theta
    n0 = n0/np.linalg.norm(n0)
    n1 = n1/np.linalg.norm(n1)
    k = np.cross(n0,n1)
    k = k/np.linalg.norm(k)
    theta = np.arccos(np.dot(n0,n1))
    
    # Compute rotated points
    P_rot = np.zeros((len(P),3))
    for i in range(len(P)):
        P_rot[i] = P[i]*np.cos(theta) + np.cross(k,P[i])*np.sin(theta) + k*np.dot(k,P[i])*(1-np.cos(theta))

    return P_rot

def angle_between(u, v, n=None):
    if n is None:
        return np.arctan2(np.linalg.norm(np.cross(u,v)), np.dot(u,v))
    else:
        return np.arctan2(np.dot(n,np.cross(u,v)), np.dot(u,v))



def pub():
    rospy.init_node('circle_fitting')
    rospy.Subscriber("/converted_euclidean_objects", vismsg.MarkerArray, callback_detectedobjects)
    rospy.spin()

if __name__ == '__main__':
    try:
        pub()
    except rospy.ROSInterruptException:
        pass