#!/usr/bin/env python

import rospy
import numpy as np

import visualization_msgs.msg as vismsg
import ros_numpy
import sensor_msgs.msg as senmsg
from shapely.geometry import Polygon,Point
import autoware_msgs.msg as autoware
import obstacle_detector.msg as ob_det


circles=None
segments=None


def tysik_callback(data):
    global circles,segments
    circles=data.circles
    segments=data.segments

    # for i in range(len(segments)):
    #     slope,intercept=slope_intercept(segments[i].first_point.x,segments[i].first_point.y,segments[i].last_point.x,segments[i].last_point.y)
   
    

    
    
def callback_freespace(data):
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
        P=Point((valid_circles[0].x + valid_circles[1].x)/2,(valid_circles[0].y + valid_circles[1].y)/2)
        #py=(valid_circles[0].y + valid_circles[1].y)/2
        print(P)
        #(np.max([valid_circles[0].y,valid_circles[1].y])) - np.abs(np.min([valid_circles[0].y,valid_circles[1].y]))-np.max([valid_circles[0].y,valid_circles[1].y])     
    else:
        print('nincs/tul sok kor')


        
    
def slope_intercept(x1,y1,x2,y2):
    slope = (y2 - y1) / (x2 - x1)
    interception = y1 - slope * x1     
    return slope,interception

def pub():
    rospy.init_node('slalom')
    rospy.Subscriber("/raw_obstacles", ob_det.Obstacles, tysik_callback)
    rospy.Subscriber("/free_space_marker", vismsg.Marker, callback_freespace)
    #rospy.Publisher()
    rospy.spin()

if __name__ == '__main__':
    try:
        pub()
    except rospy.ROSInterruptException:
        pass