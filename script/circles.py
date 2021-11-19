#!/usr/bin/env python

from numpy.core.defchararray import index
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
import math
import tf2_ros 
from sklearn.linear_model import LinearRegression

slope = None
free_space = None
valid_circles = None
slope_left = None
slope_right = None
intercept_left=intercept_right= None
trans_x = 1.938
trans_y = -0.931 
middle_pose = None
pub_valid_circles = None
pub_goal_midle = None
lenght_right = lenght_left = None
local_slope=None





def point_cloud_callback(data):
    global valid_circles, pub_valid_circles
    pc = ros_numpy.numpify(data)
    points=np.zeros((pc.shape[0],3))
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']

    
    
    valid_circles=[]
    if len(points) > 0:
        clustering=DBSCAN(eps=0.6,min_samples=5).fit(points[:,0:2])

        number_clusters= len(set(clustering.labels_)) - (1 if -1 in clustering.labels_ else 0)
        b=np.zeros((number_clusters,3))
        points=np.column_stack((points,clustering.labels_))
        for i in range(number_clusters):
            a=np.where(points[:,3]==i)

            xc,yc,r=fit_circle_2d(points[a][:,0],points[a][:,1])
            b[i,0]=xc
            b[i,1]=yc
            b[i,2]=r
        

        circles_index = np.intersect1d(np.where(b[:,2] < 1.0),np.where(b[:,2] > 0.1))
        valid_circles = b[circles_index]
        area_compare()

    marker_arr=vismsg.MarkerArray() 
    marker_arr.markers=[]


    if pub_valid_circles is not None:
        i=0
        for l in valid_circles:
            i+=1
            mark_f = vismsg.Marker()
            mark_f.header.stamp= rospy.Time.now()
            mark_f.header.frame_id = "left_os1/os1_sensor"
            mark_f.type = mark_f.SPHERE
            mark_f.action = mark_f.ADD
            mark_f.scale.x=mark_f.scale.y=mark_f.scale.z = 0.5

            mark_f.color.r = 0.1
            mark_f.color.g = 0.4
            mark_f.color.b = 0.9
            mark_f.color.a = 0.9 # 90% visibility
            mark_f.pose.orientation.x = mark_f.pose.orientation.y = mark_f.pose.orientation.z = 0.0
            mark_f.pose.orientation.w = 1.0
            mark_f.id=i
            mark_f.pose.position.x = l[0]
            mark_f.pose.position.y = l[1]
            mark_f.pose.position.z = 0.0
            mark_f.lifetime=rospy.Duration(0.1)
            marker_arr.markers.append(mark_f)
        pub_valid_circles.publish(marker_arr)     
                
  
    
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

def left_lane_callback(msg):
    global trans_x,trans_y,slope_left,intercept_left,lenght_left
    x=np.zeros(len(msg.points),)
    y=np.zeros(len(msg.points),)
    for i in range (len(msg.points)):
        x[i]=msg.points[i].x + trans_x
        y[i]=msg.points[i].y + trans_y
    lenght_left = math.sqrt((x[-1] - x[0])**2 + (y[-1] - y[0])**2)
    model = LinearRegression().fit(x.reshape(-1,1),y.reshape(-1,1))
    slope_left = model.coef_
    intercept_left = model.intercept_

   
def right_lane_callback(msg):
    global trans_x,trans_y,slope_right,intercept_right,lenght_right
    x=np.zeros(len(msg.points),)
    y=np.zeros(len(msg.points),)
    for i in range (len(msg.points)):
        x[i]=msg.points[i].x + trans_x
        y[i]=msg.points[i].y + trans_y


    lenght_right = math.sqrt((x[-1] - x[0])**2 + (y[-1] - y[0])**2)
    model = LinearRegression().fit(x.reshape(-1,1),y.reshape(-1,1))
    slope_right = model.coef_
    intercept_right = model.intercept_

def area_compare():
    global valid_circles,intercept_right,intercept_left,slope_left,slope_right,middle_pose,slope, lenght_left,lenght_right,local_slope
    
    slope = (slope_left + slope_right)/2
    
    goal=geomsg.PoseStamped()
    goal.header.frame_id="left_os1/os1_sensor"
    
    if len(valid_circles) > 0:
        y_left = slope_left*valid_circles[:,0]+intercept_left
        y_right = slope_right*valid_circles[:,0]+intercept_right
        
        y_mask = np.logical_and(valid_circles[:,1] > y_right,valid_circles[:,1] < y_left)
        
        valid_circles = valid_circles[y_mask[0]]


        # indexes=[]
        # for i in range(len(valid_circles)-1):
        #     dx=valid_circles[i+1,0]-valid_circles[i,0]
        #     dy=valid_circles[i+1,1]-valid_circles[i,1]    
        #     dist = math.sqrt(dx**2+dy**2)
        #     if dist > 3.2 and dist < 3.6:
        #         indexes.append(i)
        #         if len(indexes) == 1:
        #             local_slope=np.arctan2((valid_circles[indexes[0]+1,1]-valid_circles[indexes[0],1]),(valid_circles[indexes[0]+1,0]- valid_circles[indexes[0],0]))
        #             # print(yaw,slope,abs(slope-yaw))
        #             #if local_slope < slope + math.radians(20) or local_slope - math.radians(20):
        #             middle_pose=((valid_circles[indexes[0],0] + valid_circles[indexes[0]+1,0])/2,(valid_circles[indexes[0],1] + valid_circles[indexes[0]+1,1])/2)
                


        for i in range(len(valid_circles)-1):
            dx=valid_circles[i+1,0]-valid_circles[i,0]
            dy=valid_circles[i+1,1]-valid_circles[i,1]            
            if math.sqrt(dx**2+dy**2) < 3.5  and lenght_right > 0.8 and lenght_left > 0.8 :
                local_slope = np.arctan2((valid_circles[i+1,1]-valid_circles[i,1]),(valid_circles[i+1,0]- valid_circles[i,0]))
                middle_pose=((valid_circles[i,0] + valid_circles[i+1,0])/2,(valid_circles[i,1] + valid_circles[i+1,1])/2)
                

        if middle_pose is not None and local_slope is not None:
            goal.pose.position.x=middle_pose[0]
            goal.pose.position.y=middle_pose[1]
            goal.pose.position.z= -1.36
            if local_slope > 0:
                goal.pose.orientation.z=np.sin((local_slope - math.pi/2)/2.0)
                goal.pose.orientation.w=np.cos((local_slope - math.pi/2)/2.0)
            elif local_slope < 0:
                goal.pose.orientation.z=np.sin((local_slope + math.pi/2)/2.0)
                goal.pose.orientation.w=np.cos((local_slope + math.pi/2)/2.0)

        pub_goal_midle.publish(goal)
                


    

def pub():
    global valid_circles,pub_valid_circles,pub_goal_midle
    rospy.init_node('circle_fitting')
    rospy.Subscriber("/cloud_filtered_Box", senmsg.PointCloud2, point_cloud_callback)
    rospy.Subscriber("/left_lane_", vismsg.Marker, left_lane_callback)
    rospy.Subscriber("/right_lane", vismsg.Marker, right_lane_callback)
    pub_valid_circles= rospy.Publisher("/valid_circles",vismsg.MarkerArray,queue_size=1)
    pub_goal_midle=rospy.Publisher("/goal_midle",geomsg.PoseStamped,queue_size=1)

    rospy.spin()
       
    

if __name__ == '__main__':
    try:
        pub()
    except rospy.ROSInterruptException:
        pass