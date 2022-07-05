#!/usr/bin/env python


from numpy.core.fromnumeric import argmin
from scipy.spatial import distance
import rospy
import numpy as np
import visualization_msgs.msg as vismsg
import ros_numpy
import sensor_msgs.msg as senmsg
from sklearn.cluster import DBSCAN
import geometry_msgs.msg as geomsg
import math
import tf2_ros 
from sklearn.linear_model import LinearRegression
from dynamic_reconfigure.server import Server
from slalom.cfg import SlalomConfig
import matplotlib.pyplot as plt

slope = None
free_space = None
valid_circles = None
slope_left = None
slope_right = None
intercept_left=intercept_right= None
middle_pose = None
pub_valid_circles = None
pub_goal_midle = None
pub_slope = None
pub_sloper = None
pub_slopel = None
lenght_right = lenght_left = None
local_slope=None
EPS = None
Min_samples = None
max_radius = None
min_radius = None
max_distance = None
min_distance = None
trans_ouster_laser = None
ouster_frame = None
trans_ouster_ground_link = None


def tf_callback():
    global trans_ouster_laser,trans_ouster_ground_link,ouster_frame
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown():
    
        try:
            trans_ouster_laser = tfBuffer.lookup_transform(ouster_frame, 'laser', rospy.Time(0))
            trans_ouster_ground_link = tfBuffer.lookup_transform(ouster_frame, 'ground_link', rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        break
            

def config_callback(config,level):
    global EPS,Min_samples,max_radius,min_radius,max_distance,min_distance,ouster_frame,min_x,max_x,min_y,max_y,barrier_z,topic_name
    EPS=config['dbscan_eps']
    Min_samples=config['dbscan_min_samples']
    max_radius=config['max_radius']
    min_radius=config['min_radius']
    max_distance = config['max_distance'] 
    min_distance = config['min_distance']
    ouster_frame = config['ouster_frame']
    min_x = config['min_x'] 
    max_x = config['max_x']
    min_y = config['min_y'] 
    max_y = config['max_y']  
    barrier_z = config['barrier_z']
    topic_name = config['topic_name']
    return config


def point_cloud_callback(data):
    global valid_circles, pub_valid_circles,EPS,Min_samples,max_radius,min_radius,trans,ouster_frame,transform_received,trans_ouster_ground_link,min_x,max_x,min_y,max_y,barrier_z,pub_slopel

    pc = ros_numpy.numpify(data)
    points=np.zeros((pc.shape[0],3))
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']

    
    
    xmask= np.logical_and(points[:,0] > min_x,points[:,0] < max_x)
    ymask= np.logical_and(points[:,1] > min_y, points[:,1] < max_y )
    zmask= points[:,2] > trans_ouster_ground_link.transform.translation.z + barrier_z

    mask=np.logical_and(np.logical_and(xmask,ymask),zmask) 
    

    points=points[mask]
    
    
    valid_circles=[]
    if len(points) > 0:
        clustering=DBSCAN(eps=EPS,min_samples=Min_samples).fit(points[:,0:2])

        number_clusters= len(set(clustering.labels_))   # - (1 if -1 in clustering.labels_ else 0)
        #print(set(clustering.labels_),number_clusters)
        b=np.zeros((number_clusters,3))
        points=np.column_stack((points,clustering.labels_))

        for i in range(number_clusters):
            a=np.where(points[:,3]==i)
            

            xc,yc,r=fit_circle_2d(points[a][:,0],points[a][:,1])
            b[i,0]=xc
            b[i,1]=yc
            b[i,2]=r
        
        #print(b)
        #circles_index = np.intersect1d(np.where(b[:,2] < max_radius),np.where(b[:,2] > min_radius))
        #valid_circles = b[circles_index]
        valid_circles = b
        #print(valid_circles)
        #area_compare()

    marker_arr=vismsg.MarkerArray() 
    marker_arr.markers=[]


    if pub_valid_circles is not None:
        i=0
        for l in valid_circles:
            i+=1
            mark_f = vismsg.Marker()
            mark_f.header.stamp= rospy.Time.now()
            mark_f.header.frame_id = ouster_frame
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
    global trans_ouster_laser,slope_left,intercept_left,lenght_left,pub_slopel
    xl=np.zeros(len(msg.points),)
    yl=np.zeros(len(msg.points),)
    for i in range (len(msg.points)):
        xl[i]=msg.points[i].x + trans_ouster_laser.transform.translation.x
        yl[i]=msg.points[i].y + trans_ouster_laser.transform.translation.y
    lenght_left = math.sqrt((xl[-1] - xl[0])**2 + (yl[-1] - yl[0])**2)
    model = LinearRegression().fit(xl.reshape(-1,1),yl.reshape(-1,1))
    slope_left = model.coef_
    intercept_left = model.intercept_
    yls=model.predict(xl.reshape(-1,1))

    #yls=(slope_left*(-25))
    #print(yl[0],yl[-1],yls)
    yl_u=(intercept_left-30)*(slope_left)
    
    mark_sl = vismsg.Marker()
    mark_sl.header.stamp= rospy.Time.now()
    mark_sl.header.frame_id = ouster_frame
    mark_sl.type = mark_sl.LINE_STRIP
    mark_sl.action = mark_sl.ADD
    mark_sl.scale.x=mark_sl.scale.y=mark_sl.scale.z = 0.5
    mark_sl.color.r = 0.3
    mark_sl.color.g = 0.5
    mark_sl.color.b = 0.9
    mark_sl.color.a = 0.9 # 90% visibility
    mark_sl.pose.orientation.x = mark_sl.pose.orientation.y = mark_sl.pose.orientation.z = 0.0
    mark_sl.pose.orientation.w = 1.0
    mark_sl.lifetime=rospy.Duration(0.1)
    mark_sl.points=[]
    if slope_left is not None:
        p1l=geomsg.Point(); p1l.x=xl[-1];p1l.y=yl[-1]; p1l.z=0.0
        p2l=geomsg.Point(); p2l.x=30;p2l.y=yl_u; p2l.z=0.0
        mark_sl.points.append(p1l)
        mark_sl.points.append(p2l)

    pub_slopel.publish(mark_sl)

   
def right_lane_callback(msg):
    global trans_ouster_laser,slope_right,intercept_right,lenght_right,pub_sloper
    xr=np.zeros(len(msg.points),)
    yr=np.zeros(len(msg.points),)
    for i in range (len(msg.points)):
        xr[i]=msg.points[i].x + trans_ouster_laser.transform.translation.x
        yr[i]=msg.points[i].y + trans_ouster_laser.transform.translation.y
    lenght_right = math.sqrt((xr[-1] - xr[0])**2 + (yr[-1] - yr[0])**2)
    model = LinearRegression().fit(xr.reshape(-1,1),yr.reshape(-1,1))
    
    slope_right = model.coef_
    intercept_right = model.intercept_
    yrs=model.predict(xr.reshape(-1,1))

    #yrs=slope_right*25
    
    mark_sr = vismsg.Marker()
    mark_sr.header.stamp= rospy.Time.now()
    mark_sr.header.frame_id = ouster_frame
    mark_sr.type = mark_sr.LINE_STRIP
    mark_sr.action = mark_sr.ADD
    mark_sr.scale.x=mark_sr.scale.y=mark_sr.scale.z = 0.5
    mark_sr.color.r = 0.3
    mark_sr.color.g = 0.5
    mark_sr.color.b = 0.9
    mark_sr.color.a = 0.9 # 90% visibility
    mark_sr.pose.orientation.x = mark_sr.pose.orientation.y = mark_sr.pose.orientation.z = 0.0
    mark_sr.pose.orientation.w = 1.0
    mark_sr.lifetime=rospy.Duration(0.1)
    mark_sr.points=[]
    if slope_right is not None:
        p1r=geomsg.Point(); p1r.x=xr[-1];p1r.y=yrs[-1]; p1r.z=0.0
        p2r=geomsg.Point(); p2r.x=xr[0];p2r.y=yrs[0]; p2r.z=0.0
        mark_sr.points.append(p1r)
        mark_sr.points.append(p2r)
    pub_sloper.publish(mark_sr)
    

# def area_compare():
#     global valid_circles,intercept_right,intercept_left,slope_left,slope_right,middle_pose,slope, lenght_left,lenght_right,local_slope,max_distance,min_distance,ouster_frame
    

#     if slope_left is not None and slope_right is not None:
#         slope = (slope_left + slope_right)/2
        
#         goal=geomsg.PoseStamped()
#         goal.header.frame_id=ouster_frame
        
#         #if len(valid_circles) > 0: #and slope_left is not None and slope_right is not None:
#         y_left = slope_left*valid_circles[:,0]+intercept_left
#         y_right = slope_right*valid_circles[:,0]+intercept_right
        
#         y_mask = np.logical_and(valid_circles[:,1] > y_right,valid_circles[:,1] < y_left)
        
#         valid_circles = valid_circles[y_mask[0]]
#         positive_rotated_slope=slope[0]+(math.pi/2)
#         negative_rotated_slope=slope[0]-(math.pi/2)
        
#         y=slope*(25)
       
       
        
#         mark_s = vismsg.Marker()
#         mark_s.header.stamp= rospy.Time.now()
#         mark_s.header.frame_id = ouster_frame
#         mark_s.type = mark_s.LINE_STRIP
#         mark_s.action = mark_s.ADD
#         mark_s.scale.x=mark_s.scale.y=mark_s.scale.z = 0.5
#         mark_s.color.r = 0.1
#         mark_s.color.g = 0.4
#         mark_s.color.b = 0.9
#         mark_s.color.a = 0.9 # 90% visibility
#         mark_s.pose.orientation.x = mark_s.pose.orientation.y = mark_s.pose.orientation.z = 0.0
#         mark_s.pose.orientation.w = 1.0
#         mark_s.lifetime=rospy.Duration(0.1)
#         mark_s.points=[]
#         if y is not None:
#             p1=geomsg.Point(); p1.x=0.0;p1.y=0.0; p1.z=0.0
#             p2=geomsg.Point(); p2.x=-25;p2.y=y; p2.z=0.0
#             mark_s.points.append(p1)
#             mark_s.points.append(p2)

#         # mark_sr = vismsg.Marker()
#         # mark_sr.header.stamp= rospy.Time.now()
#         # mark_sr.header.frame_id = ouster_frame
#         # mark_sr.type = mark_sr.LINE_STRIP
#         # mark_sr.action = mark_sr.ADD
#         # mark_sr.scale.x=mark_sr.scale.y=mark_s.scale.z = 0.5
#         # mark_sr.color.r = 0.3
#         # mark_sr.color.g = 0.5
#         # mark_sr.color.b = 0.9
#         # mark_sr.color.a = 0.9 # 90% visibility
#         # mark_sr.pose.orientation.x = mark_sr.pose.orientation.y = mark_sr.pose.orientation.z = 0.0
#         # mark_sr.pose.orientation.w = 1.0
#         # mark_sr.lifetime=rospy.Duration(0.1)
#         # mark_sr.points=[]
#         # if yr is not None:
#         #     p1=geomsg.Point(); p1.x=0.0;p1.y=-1.65; p1.z=0.0
#         #     p2=geomsg.Point(); p2.x=-25;p2.y=yr; p2.z=0.0
#         #     mark_sr.points.append(p1)
#         #     mark_sr.points.append(p2)

#         pub_slope.publish(mark_s)
#         # pub_sloper.publish(mark_sr)
#         a=[]


#         for i in range(len(valid_circles)-1):
#             dx=valid_circles[i+1,0]-valid_circles[i,0]
#             dy=valid_circles[i+1,1]-valid_circles[i,1]            
#             if math.sqrt(dx**2+dy**2) < max_distance and math.sqrt(dx**2+dy**2) > min_distance  and lenght_right > 0.8 and lenght_left > 0.8 :
#                 local_slope = np.arctan2((valid_circles[i+1,1]-valid_circles[i,1]),(valid_circles[i+1,0]- valid_circles[i,0]))
#                 #print(slope,local_slope)
#                 if (local_slope > positive_rotated_slope - math.radians(20) and local_slope < positive_rotated_slope - math.radians(20)) or (local_slope < negative_rotated_slope + math.radians(20) and local_slope > negative_rotated_slope - math.radians(20)):
#                 #if (local_slope > slope[0] - math.radians(20) and local_slope < slope[0] - math.radians(20)) or (local_slope < slope[0] + math.radians(20) and local_slope > slope[0] - math.radians(20)):       
#                     a.append(((valid_circles[i,0] + valid_circles[i+1,0])/2,(valid_circles[i,1] + valid_circles[i+1,1])/2))
#                     valid_slope=slope
#                     if len(a) > 1:
#                         mat=np.zeros(len(a),)
#                         for i in range(len(a)):
#                             mat[i]=math.sqrt(a[i][0]**2+(a[i][1]**2))
#                         middle_pose=a[np.argmin(mat)]
#                     else: 
#                         middle_pose=a[0]                   

#                     if middle_pose is not None and valid_slope is not None:
#                         goal.pose.position.x=middle_pose[0]
#                         goal.pose.position.y=middle_pose[1]
#                         goal.pose.position.z= -1.36

#                         if valid_slope > 0:
#                             goal.pose.orientation.z=np.sin((valid_slope + math.pi)/2.0)
#                             goal.pose.orientation.w=np.cos((valid_slope + math.pi) /2.0)
#                         elif valid_slope < 0:
#                             goal.pose.orientation.z=np.sin((valid_slope-math.pi)/2.0)
#                             goal.pose.orientation.w=np.cos((valid_slope-math.pi)/2.0)
                        
                        
#                     pub_goal_midle.publish(goal)
                


    

def pub():
    global valid_circles,pub_valid_circles,pub_goal_midle,first_run,transform_received,pub_slope,topic_name,pub_sloper,pub_slopel
    rospy.init_node('circle_fitting')
    srv=Server(SlalomConfig,config_callback)
    tf_callback()
    rospy.Subscriber(topic_name, senmsg.PointCloud2, point_cloud_callback)
    
    rospy.Subscriber("/left_lane", vismsg.Marker, left_lane_callback)
    rospy.Subscriber("/right_lane", vismsg.Marker, right_lane_callback)
    pub_valid_circles= rospy.Publisher("/valid_circles",vismsg.MarkerArray,queue_size=1)
    pub_goal_midle=rospy.Publisher("/goal_middle",geomsg.PoseStamped,queue_size=1)
    pub_slope=rospy.Publisher("/slope",vismsg.Marker, queue_size=1)
    pub_sloper=rospy.Publisher("/slope_r",vismsg.Marker, queue_size=1)
    pub_slopel=rospy.Publisher("/slope_l",vismsg.Marker, queue_size=1)

    rospy.spin()
       
    

if __name__ == '__main__':
    try:
        pub()
    except rospy.ROSInterruptException:
        pass