#!/usr/bin/env python

import rospy
import numpy as np

import visualization_msgs.msg as vismsg
import ros_numpy
import sensor_msgs.msg as senmsg
from shapely.geometry import Polygon,Point
import autoware_msgs.msg as autoware
import obstacle_detector.msg as ob_det
import geometry_msgs.msg as geomsg



circles=None
segments=None
P=None
slope=None


def tysik_callback(data):
    global circles,segments
    circles=data.circles
    segments=data.segments

    # for i in range(len(segments)):
    #     slope,intercept=slope_intercept(segments[i].first_point.x,segments[i].first_point.y,segments[i].last_point.x,segments[i].last_point.y)

def callback_freespace_local(data):
    global slope

    points=data.points 
    
    a=np.zeros((5,2))

    polygon_data= np.zeros((len(points),2)) 
    for i in range (len(points)):
        polygon_data[i,0]=points[i].x
        polygon_data[i,1]=points[i].y
    #free_space=Polygon([[np.min(polygon_data[:,0]),np.max(polygon_data[:,1])],[np.max(polygon_data[:,0]),np.max(polygon_data[:,1])],[np.max(polygon_data[:,0]),np.min(polygon_data[:,1])],[np.min(polygon_data[:,0]),np.min(polygon_data[:,1])]])    

    free_space=Polygon(polygon_data).minimum_rotated_rectangle


    # a[:,0]=free_space.exterior.coords.xy[0]
    # a[:,1]=free_space.exterior.coords.xy[1]
    if free_space.exterior.coords.xy[0][0] < free_space.exterior.coords.xy[0][1]: 
        slope=np.arctan2((free_space.exterior.coords.xy[1][0]-free_space.exterior.coords.xy[1][1]),(free_space.exterior.coords.xy[0][0]-free_space.exterior.coords.xy[0][1]))
    elif free_space.exterior.coords.xy[0][0] > free_space.exterior.coords.xy[0][1]: 
        slope=np.arctan2((free_space.exterior.coords.xy[1][1]-free_space.exterior.coords.xy[1][0]),(free_space.exterior.coords.xy[0][1]-free_space.exterior.coords.xy[0][0]))    
    
def callback_freespace_global(data):
    global P

    points=data.points 
    
    valid_circles=[]
    middle=[]
    side=[]

    polygon_data= np.zeros((len(points),2)) 
    for i in range (len(points)):
        polygon_data[i,0]=points[i].x
        polygon_data[i,1]=points[i].y
    #free_space=Polygon([[np.min(polygon_data[:,0]),np.max(polygon_data[:,1])],[np.max(polygon_data[:,0]),np.max(polygon_data[:,1])],[np.max(polygon_data[:,0]),np.min(polygon_data[:,1])],[np.min(polygon_data[:,0]),np.min(polygon_data[:,1])]])    

    free_space=Polygon(polygon_data).minimum_rotated_rectangle
    #print(free_space.exterior.coords.xy)

    #slope_left,interception=slope_intercept(free_space.exterior.coords.xy[0][0],)

    #slope=np.arctan2((free_space.exterior.coords.xy[1][1]-free_space.exterior.coords.xy[1][0]),(free_space.exterior.coords.xy[0][1]-free_space.exterior.coords.xy[0][0]))
    #slope_right=np.arctan2((free_space.exterior.coords.xy[1][2]-free_space.exterior.coords.xy[1][3]),(free_space.exterior.coords.xy[0][2]-free_space.exterior.coords.xy[0][3]))

    if circles is not None:
        for i in range(len(circles)):
            if free_space.contains(Point(circles[i].center.x,circles[i].center.y))==True:
                valid_circles.append(Point(circles[i].center.x,circles[i].center.y))
        
    #     #print(valid_circles)
    #     for i in valid_circles:
    #         if Point(circles[i].center.x,circles[i].center.y).buffer(circles[i].radius).within(free_space)==True:
    #             middle.append(i)
    #         else:
    #             side.append(i)
                    
    # print(len(middle),len(side))
    #         #circles_=Point(circles[i].center.x,circles[i].center.y).buffer(circles[i].radius)
    #         # print(circles_.within(free_space),i)
    # #         if free_space.contains(circles_)==True:
    # #             valid_circles.append(circles_)
    
    # #print(valid_circles[0].x
    if len(valid_circles)==2:
        P=((valid_circles[0].x + valid_circles[1].x)/2,(valid_circles[0].y + valid_circles[1].y)/2)
        #py=(valid_circles[0].y + valid_circles[1].y)/2
        #print(P)

        
    
        # if pub_goal_midle is not None:
        #     pub_goal_midle.publish(goal)


        #(np.max([valid_circles[0].y,valid_circles[1].y])) - np.abs(np.min([valid_circles[0].y,valid_circles[1].y]))-np.max([valid_circles[0].y,valid_circles[1].y])     
    # else:
    #     print('nincs/tul sok kor',len(valid_circles))


        
    
def slope_intercept(x1,y1,x2,y2):
    slope = (y2 - y1) / (x2 - x1)
    interception = y1 - slope * x1     
    return np.arctan2(slope),interception






def pub():
    rospy.init_node('slalom')
    rospy.Subscriber("/raw_obstacles", ob_det.Obstacles, tysik_callback)
    rospy.Subscriber("/converted_bounding_poly", vismsg.Marker, callback_freespace_global)
    rospy.Subscriber("/free_space_marker", vismsg.Marker, callback_freespace_local)

    pub_goal_midle=rospy.Publisher("/goal_midle",geomsg.PoseStamped,queue_size=1)

    rate=rospy.Rate(20)
    goal=geomsg.PoseStamped()
    goal.header.frame_id="map"

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