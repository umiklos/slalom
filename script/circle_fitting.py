#!/usr/bin/env python

import rospy
import numpy as np
import visualization_msgs.msg as vismsg
import ros_numpy
import sensor_msgs.msg as senmsg
from shapely.geometry import Polygon,Point
import autoware_msgs.msg as autoware
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
import geometry_msgs.msg as geomsg

P= None
free_space = None
valid_circles = None
slope = None


def point_cloud_callback(data):
    global free_space, valid_circles
    pc = ros_numpy.numpify(data)
    points=np.zeros((pc.shape[0],3))
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']


    zmask = points[:,2] > -0.4
    points_zmasked = points[zmask] 
    clustering=DBSCAN(eps=0.75,min_samples=10).fit(points_zmasked[:,0:2])

    # plt.scatter(points_zmasked[:,0],points_zmasked[:,1])
    # plt.show()
    
    number_clusters= len(set(clustering.labels_)) - (1 if -1 in clustering.labels_ else 0)
    b=np.zeros((number_clusters,3))
    points_zmasked=np.column_stack((points_zmasked,clustering.labels_))
    for i in range(number_clusters):
        a=np.where(points_zmasked[:,3]==i)

        xc,yc,r=fit_circle_2d(points_zmasked[a][:,0],points_zmasked[a][:,1])
        b[i,0]=xc
        b[i,1]=yc
        b[i,2]=r
    

    circles_index = np.intersect1d(np.where(b[:,2] < 2.0),np.where(b[:,2] > 0.1))
    valid_circles = b[circles_index]

    #free_space=Polygon([[np.min(points[:,0]),np.max(points[:,1])],[np.max(points[:,0]),np.max(points[:,1])],[np.max(points[:,0]),np.max(points[:,1])],[np.min(points[:,0]),np.min(points[:,1])]])

def callback_markerarray(data):
    global P,slope
    polygon_list=[]
    polygons=[]
    markers_list=[]
    c=[]
    
    for i in range(len(data.markers)):
        markers=[]
        #print(i)

        #markers=np.empty([0])
        
        # for j in range(len(data.markers[i].points)):
        #     np.append(markers,[data.markers[i].points[j].x,data.markers[i].points[j].y])

        for j in range(len(data.markers[i].points)):
            markers.append([data.markers[i].points[j].x,data.markers[i].points[j].y]) 
        markers_list.append(markers)
    free_space=Polygon(np.concatenate(markers_list)).minimum_rotated_rectangle
    #free_space=(np.concatenate(markers_list))

    if free_space.exterior.coords.xy[0][0] > free_space.exterior.coords.xy[0][1]: 
        slope=np.arctan2((free_space.exterior.coords.xy[1][0]-free_space.exterior.coords.xy[1][1]),(free_space.exterior.coords.xy[0][0]-free_space.exterior.coords.xy[0][1]))
    elif free_space.exterior.coords.xy[0][0] < free_space.exterior.coords.xy[0][1]: 
        slope=np.arctan2((free_space.exterior.coords.xy[1][1]-free_space.exterior.coords.xy[1][0]),(free_space.exterior.coords.xy[0][1]-free_space.exterior.coords.xy[0][0])) 

    #     polygon_list.append(markers)
    # for k in range(len(polygon_list)):        
    #     polygons.append(Polygon(polygon_list[k]))
    

    
    if valid_circles is not None:
        for i in range(len(valid_circles)):
            c.append((free_space).contains(Point(valid_circles[i,0],valid_circles[i,1]).buffer(0.2)))
                #c.append(i)

        valid_circles_=valid_circles[c]

        if len(valid_circles_)==2:
            P=((valid_circles_[0,0] + valid_circles_[1,0])/2,(valid_circles_[0,1] + valid_circles_[1,1])/2)
        #     pass
    
    
    # plt.plot(free_space.exterior.coords.xy[0],free_space.exterior.coords.xy[1])
    # plt.scatter(valid_circles[:,0],valid_circles[:,1])
    # plt.show()

    # points=data.points 
    
    # a=np.zeros((5,2))

    # polygon_data= np.zeros((len(points),2)) 
    # for i in range (len(points)):
    #     polygon_data[i,0]=points[i].x
    #     polygon_data[i,1]=points[i].y
    
    # free_space=Polygon(polygon_data).minimum_rotated_rectangle

    # print('a')
    # # if valid_circles is not None:
    # #     for i in range(len(valid_circles)):




# def callback_detectedobjects(data):
#     polygon_list=[]
#     polygon=[]
    
    
#     for i in range (len(data.objects)):
#         polygon_data=np.zeros((len(data.objects[i].convex_hull.polygon.points),2))
#         for j in range(len(data.objects[i].convex_hull.polygon.points)):
#             polygon_data[j,0]=data.objects[i].convex_hull.polygon.points[j].x
#             polygon_data[j,1]=data.objects[i].convex_hull.polygon.points[j].y
#         polygon_list.append(polygon_data)
#     polygon_list=np.array(polygon_list)  

#     a=np.zeros((len(polygon_list),3))
    
#     for i in range(len(a)):
#         xc,yc,r=fit_circle_2d(polygon_list[i][:,0],polygon_list[i][:,1])
#         a[i,0]=xc
#         a[i,1]=yc
#         a[i,2]=r


    
#     circles=np.intersect1d(np.where(a[:,2] < 3.0),np.where(a[:,2] > 0.2))  

#     for i in circles:
#         p=Polygon(polygon_list[i])
#         if free_space.intersects(p):
#             polygon.append(i)

#     valid_polygon=a[polygon]

#     print(len(valid_polygon),valid_polygon)
#     # print(valid_polygon)

    
#     # for k in range(len(polygon_list)):
#     #     p=Polygon(polygon_list[k])
#     #     polygon.append(p)

    
    
#     # for i in range (len(polygon)):
#     #     if free_space.contains(polygon[i]) == True:

#     #         xc,yc,r=fit_circle_2d(polygon_list[i][:,0],polygon_list[i][:,1])
#     #         if r >0.75 and r< 1.5:
#     #             circles.append(i)

    

        


    
   
    
    
    
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
    rospy.Subscriber("/cloud_filtered_Box", senmsg.PointCloud2, point_cloud_callback)
    rospy.Subscriber("/visualization_MarkerArray", vismsg.MarkerArray, callback_markerarray)
    pub_goal_midle=rospy.Publisher("/goal_midle",geomsg.PoseStamped,queue_size=1)


    rate=rospy.Rate(20)
    goal=geomsg.PoseStamped()
    goal.header.frame_id="right_os1/os1_sensor"

    while not rospy.is_shutdown():
        if P is not None and slope is not None:
            goal.pose.position.x=P[0]
            goal.pose.position.y=P[1]
            goal.pose.position.z= -1.36
            goal.pose.orientation.z=np.sin(slope/2.0)
            goal.pose.orientation.w=np.cos(slope/2.0)
        
        pub_goal_midle.publish(goal)
    rate.sleep()
    #rospy.spin()

if __name__ == '__main__':
    try:
        pub()
    except rospy.ROSInterruptException:
        pass