import matplotlib.pyplot as plt 
import numpy as np
import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

class confiden():
    def __init__(self):
        rospy.init_node('confiden',anonymous=False)

        self.map = a 
        self.map_size = [377, 392]
        self.init_map = [-9.298303, -9.747517]
        self.theta = 0.0
        self.max_pixel = 100
        self.resolution = 0.05
        self.steps = 1
        self.ranges_of_scan = 360
        self.max_err = 0.1
        self.max_key_scans = 40
        self.visual_mode = 2 # 0-key, 1-match and false
        self.pose_oxy = Pose()
        self.pose_pixel = Pose()
        self.confiden_visual = Pose()
        self.err_pose = Pose()
        self.odom = Odometry()
        self.pose_element = []

        self.amcl_topic = rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped,self.amcl_callback)
        self.scan_topic = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.scan_rviz_topic = rospy.Publisher("/scan_rviz",LaserScan,queue_size = 10000)
        self.markersKey_topic = rospy.Publisher("/markerKey_arr",MarkerArray,queue_size =10000)
        self.markersMatch_topic = rospy.Publisher("/markerMatch_arr",MarkerArray,queue_size =10000)
        self.markersFalse_topic = rospy.Publisher("/markerFalse_arr",MarkerArray,queue_size =10000)
        self.confidence_topic = rospy.Publisher("/confidence1",Pose,queue_size =10000)
        
        self.odom_topic = rospy.Subscriber("/odom",Odometry,self.odom_callback)

        self.amcl = PoseWithCovarianceStamped()
        self.scan = LaserScan()
        self.scan_rviz = LaserScan()
        self.scan_rviz.ranges = [0] * 180
        self.key_scan_ranges = []
        self.scan.ranges = [0] * 180
        self.match_scans_pose = []
        self.key_scans_pose = [0] * self.ranges_of_scan
        self.false_scans_pose = []
        self.count_confiden = 0

        self.markerArray_key = MarkerArray()
        self.markerArray_match = MarkerArray()
        self.markerArray_false = MarkerArray()

    def amcl_callback(self, msg):
        self.amcl = msg
        
        # self.pose_oxy.position.x = self.amcl.pose.pose.position.x
        # self.pose_oxy.position.y = self.amcl.pose.pose.position.y
        pose_oxy2pixel = self.oxy2pixel(self.amcl.pose.pose.position.x, self.amcl.pose.pose.position.y)
        self.pose_pixel.position.x = pose_oxy2pixel[0]
        self.pose_pixel.position.y = pose_oxy2pixel[1]

        self.theta = self.quaternion2yaw(self.amcl.pose.pose.orientation)
    
    def scan_callback(self, msg):
        self.scan = msg
        
        # self.key_scan = self.scan 
    
    def odom_callback(self, msg):
        self.odom = msg
    
    def oxy2pixel(self,x,y):
        pixel = []
        pixel.append(self.map_size[0]-1-int(((x-self.init_map[0])/0.05)))
        pixel.append(self.map_size[1]-1-int(((y-self.init_map[1])/0.05)))
        return pixel
    
    def pixel2oxy(self,x,y):
        oxy = []
        oxy.append(self.init_map[0]+(self.map_size[0]-1-x)*0.05)
        oxy.append(self.init_map[1]+(self.map_size[1]-1-y)*0.05)
        return oxy

    def dis_pixel(self,i,j):
        dis = 0.05*(math.sqrt(i*i+j*j))
        return dis

    def ang_pixel(self,i,j):
        if (i==0):
            if(j>0):
                return 1
            else:
                return 180
        if (j==0):
            if (i>0):
                return 90
            else:
                return 270
        if (i>0):
            if (j>0):
                ang = math.atan(abs(i)/abs(j))*180/math.pi
            else:
                ang = 90 + math.atan(abs(j)/abs(i))*180/math.pi
        if (i<0):
            if (j>0):
                ang = 360 - math.atan(abs(i)/abs(j))*180/math.pi
            else:
                ang = 180 + math.atan(abs(i)/abs(j))*180/math.pi
        
        return ang
        
    def quaternion2yaw(self,quaternion):
        q0 = quaternion.x 
        q1 = quaternion.y 
        q2 = quaternion.z 
        q3 = quaternion.w

        yaw = math.atan2(2.0*(q2*q3+q0*q1), 1.0-2.0*(q1*q1+q2*q2))
        return -yaw #rad
    
    def rad2deg(self, rad):
        deg = rad*180 / math.pi
        return deg 

    def find_obj(self):
        self.key_scans_pose.clear()
        self.key_scan_ranges=[0] * 360

        ang = self.rad2deg(self.quaternion2yaw(self.amcl.pose.pose.orientation))
        print(f'Angle: {ang}')
        pose_x = int(self.pose_pixel.position.x)
        pose_y = int(self.pose_pixel.position.y)
        print(f'Pose: {pose_x, pose_y}')
        #
        for i in range (0,self.max_pixel, self.steps):
            for j in range(self.max_pixel):
                if (pose_x-i<0 or pose_y+j>=self.map_size[1]):
                    break
                if (self.map[pose_x-i][pose_y+j]==100):
                    ang1 = int(self.ang_pixel(i,j)-1 + ang)
                    if (ang1>359):
                        ang1 -= 359
                    if (ang1<0):
                        ang1 = 359 - abs(ang1)
                    dis1 = self.dis_pixel(i,j)
                    if (ang1>0 and self.key_scan_ranges[ang1]==0 and (self.key_scan_ranges[ang1-1]==0 or abs(self.key_scan_ranges[ang1-1]-dis1)<self.max_err)):
                        self.key_scan_ranges[ang1] = dis1
                        self.key_scans_pose.append([pose_x-i, pose_y+j, ang1, dis1])
                        #print(f'ang1: {ang1}')
                    break

            for k in range(self.max_pixel):
                if (pose_x-i<0 or pose_y-k<0):
                    break
                if (self.map[pose_x-i][pose_y-k]==100):
                    ang2 = int(self.ang_pixel(i,-k)-1 +ang)
                    if (ang2>359):
                        ang2 -= 359
                    if (ang2<0):
                        ang2 = 359 - abs(ang2)
                    dis2 = self.dis_pixel(i,k)
                    if (ang2<359 and self.key_scan_ranges[ang2]==0 and (self.key_scan_ranges[ang2+1]==0 or abs(self.key_scan_ranges[ang2-1]-dis2)<self.max_err)):
                        self.key_scan_ranges[ang2] = dis2
                        self.key_scans_pose.append([pose_x-i, pose_y-k, ang2, dis2])
                        #print(f'ang2: {ang2}')
                    break
                    
            for g in range(self.max_pixel):
                if (pose_x+i>=self.map_size[0] or pose_y+g>=self.map_size[1]):
                    break
                if (self.map[pose_x+i][pose_y+g]==100):
                    ang4 = int(self.ang_pixel(-i,g)-1 + ang)
                    if (ang4>359):
                        ang4 -= 359
                    if (ang4<0):
                        ang4 = 359 - abs(ang4)
                    dis4 = self.dis_pixel(i,g)
                    if (ang4<359 and self.key_scan_ranges[ang4]==0 and (self.key_scan_ranges[ang4+1]==0 or abs(self.key_scan_ranges[ang4+1]-dis4)<self.max_err)):
                        self.key_scan_ranges[ang4] = dis4
                        self.key_scans_pose.append([pose_x+i, pose_y+g, ang4, dis4])
            
                        #print(pose_x+u, pose_y+g)
                    break
                    
            for h in range(self.max_pixel):
                if(pose_x+i>=self.map_size[0] or pose_y-h<0):
                    break
                if (self.map[pose_x+i][pose_y-h]==100):
                    ang3 = int(self.ang_pixel(-i,-h)-1 + ang)
                    if (ang3>359):
                        ang3 -= 359
                    if (ang3<0):
                        ang3 = 359 - abs(ang3)
                    dis3 = self.dis_pixel(i,h)
                    if (ang3>0 and self.key_scan_ranges[ang3]==0 and (self.key_scan_ranges[ang3-1]==0 or abs(self.key_scan_ranges[ang3-1]-dis3)<self.max_err)):
                        self.key_scan_ranges[ang3] = dis3
                        self.key_scans_pose.append([pose_x+i,pose_y-h,ang3,dis3])
                    break
        
        for i_ in range (0,self.max_pixel, self.steps):
            for j_ in range(self.max_pixel):
                if (pose_x-j_<0 or pose_y+i_>=self.map_size[1]):
                    break
                if (self.map[pose_x-j_][pose_y+i_]==100):
                    ang_1 = int(self.ang_pixel(j_,i_)-1 + ang)
                    if (ang_1>359):
                        ang_1 -= 359
                    if (ang_1<0):
                        ang_1 = 359 - abs(ang_1)
                    dis_1 = self.dis_pixel(i_,j_)
                    if (ang_1<359 and self.key_scan_ranges[ang_1]==0 and (self.key_scan_ranges[ang_1-1]==0 or abs(self.key_scan_ranges[ang_1-1]-dis_1)<self.max_err)):
                        self.key_scan_ranges[ang_1] = dis_1
                        self.key_scans_pose.append([pose_x-j_, pose_y+i_, ang_1, dis_1])
                        #print(f'ang1: {ang1}')
                    break
     
            for k_ in range(self.max_pixel):
                if (pose_x-k_<0 or pose_y-i_<0):
                    break
                if (self.map[pose_x-k_][pose_y-i_]==100):
                    ang_2 = int(self.ang_pixel(k_,-i_)-1 +ang)
                    if (ang_2>359):
                        ang_2 -= 359
                    if (ang_2<0):
                        ang_2 = 359 - abs(ang_2)
                    dis_2 = self.dis_pixel(i_,k_)
                    if (ang_2>0 and self.key_scan_ranges[ang_2]==0 and (self.key_scan_ranges[ang_2-1]==0 or abs(self.key_scan_ranges[ang_2-1]-dis_2)<self.max_err)):
                        # print(f'cur: {ang_2,dis_2}')
                        self.key_scan_ranges[ang_2] = dis_2
                        self.key_scans_pose.append([pose_x-k_, pose_y-i_, ang_2, dis_2])
                        #print(f'ang2: {ang2}')
                    break
            
            for g_ in range(self.max_pixel):
                if (pose_x+g_>=self.map_size[0] or pose_y+i_>=self.map_size[1]):
                    break
                if (self.map[pose_x+g_][pose_y+i_]==100):
                    ang_4 = int(self.ang_pixel(-g_,i_)-1 + ang)
                    if (ang_4>359):
                        ang_4 -= 359
                    if (ang_4<0):
                        ang_4 = 359 - abs(ang_4)
                    dis_4 = self.dis_pixel(g_,i_)
                    if (ang_4>0 and self.key_scan_ranges[ang_4]==0 and (self.key_scan_ranges[ang_4-1]==0 or abs(self.key_scan_ranges[ang_4-1]-dis_4)<self.max_err)):
                    
                        self.key_scan_ranges[ang_4] = dis_4
                        self.key_scans_pose.append([pose_x+g_, pose_y+i_, ang_4, dis_4])
            
                        #print(pose_x+u, pose_y+g)
                    break
                    
            
            for h_ in range(self.max_pixel):
                if(pose_x+h_>=self.map_size[0] or pose_y-i_<0):
                    break
                if (self.map[pose_x+h_][pose_y-i_]==100):
                    ang_3 = int(self.ang_pixel(-h_,-i_)-1 + ang)
                    if (ang_3>359):
                        ang_3 -= 359
                    if (ang_3<0):
                        ang_3 = 359 - abs(ang_3)
                    dis_3 = self.dis_pixel(i_,h_)
                    if (ang_3>0 and self.key_scan_ranges[ang_3]==0 and (self.key_scan_ranges[ang_3-1]==0 or abs(self.key_scan_ranges[ang_3-1]-dis_3)<self.max_err)):
                        self.key_scan_ranges[ang_3] = dis_3
                        self.key_scans_pose.append([pose_x+h_,pose_y-i_,ang_3,dis_3])
                    break
        
        #print(f'key_scans_pose: {self.key_scan_ranges}')
        return self.key_scans_pose, self.key_scan_ranges
        
    def find_confidence(self):
        self.match_scans_pose.clear()
        self.false_scans_pose.clear()
        key_scans_list, scan_rviz = self.find_obj()
        sum = 0
        count = 0
        
        for i in key_scans_list:
            if (i[2]<179):
                sum += 1
                err1 = self.scan.ranges[i[2]] - i[3] 
                err2 = self.scan.ranges[i[2]-1] - i[3]
                err3 = self.scan.ranges[i[2]+1] - i[3]
                if (abs(err1) < self.max_err or abs(err2) < self.max_err or abs(err3) < self.max_err ):
                    count = count + 1 
                    self.match_scans_pose.append(i)
                    self.scan_rviz.ranges[i[2]] = self.scan.ranges[i[2]]
                else:
                    self.false_scans_pose.append(i)
                    self.scan_rviz.ranges[i[2]] = 32.0
        self.confiden = 100* count / sum
        if (sum<self.max_key_scans):
            self.confiden = 0
            print("Not enough key scans!")
            print("---------------------------------------")
        else:
            print(f'Sum key scans:       {sum}')
            print(f'Sum match key scans: {count}')
            print(f'Confidence:           {self.confiden}')
            print("---------------------------------------")
        
        return self.confiden, self.match_scans_pose, self.false_scans_pose

    def visual_scans(self):
        self.markerArray_key.markers.clear()
        self.markerArray_match.markers.clear()
        self.markerArray_false.markers.clear()
        if (self.visual_mode==2):
            confiden_, pose_match_pixel, pose_false_pixel = self.find_confidence()
            self.scan_rviz.header.seq = self.scan.header.seq
            self.scan_rviz.header.stamp = self.scan.header.stamp
            self.scan_rviz.header.frame_id = self.scan.header.frame_id
            self.scan_rviz_topic.publish(self.scan_rviz)

        if (self.visual_mode==1):
            confiden_, pose_match_pixel, pose_false_pixel = self.find_confidence()
            #print(self.key_scan_ranges)
            #print(f'pose_match_pixel: {pose_match_pixel}')
            #print(f'pose_false_pixel: {pose_false_pixel}')
            for i in range(len(pose_match_pixel)):
                pose_match_oxy = self.pixel2oxy(pose_match_pixel[i][0],pose_match_pixel[i][1])
                marker = Marker()
                marker.header.frame_id = "map"
                marker.type = marker.SPHERE
                marker.action = marker.ADD
                marker.scale.x = 0.05
                marker.scale.y = 0.05
                marker.scale.z = 0.05
                marker.color.a = 1.0
                marker.color.r = 1.0 
                marker.color.g = 0.0 
                marker.color.b = 0.0
                marker.pose.orientation.w = 1.0 
                marker.pose.position.x = pose_match_oxy[0]
                marker.pose.position.y = pose_match_oxy[1]
                marker.pose.position.z = 0.0
                marker.id = i
                self.markerArray_match.markers.append(marker)
                
                
            for i in range(len(pose_false_pixel)):
                pose_false_oxy = self.pixel2oxy(pose_false_pixel[i][0],pose_false_pixel[i][1])
                marker = Marker()
                marker.header.frame_id = "map"
                marker.type = marker.SPHERE
                marker.action = marker.ADD
                marker.scale.x = 0.05
                marker.scale.y = 0.05
                marker.scale.z = 0.05
                marker.color.a = 1.0
                marker.color.r = 0.0 
                marker.color.g = 0.0 
                marker.color.b = 1.0
                marker.pose.orientation.w = 1.0 
                marker.pose.position.x = pose_false_oxy[0]
                marker.pose.position.y = pose_false_oxy[1]
                marker.pose.position.z = 0.0
                marker.id = i
                self.markerArray_false.markers.append(marker)

            self.view_error()
            #self.scan_rviz_topic.publish(self.scan_rviz)

            #self.markersMatch_topic.publish(self.markerArray_match)
            #self.markersFalse_topic.publish(self.markerArray_false)
        if(self.visual_mode==0):
            pose_key_pixel = self.find_obj()
            for i in range(len(pose_key_pixel)):
                pose_key_oxy = self.pixel2oxy(pose_key_pixel[i][0],pose_key_pixel[i][1])
                marker = Marker()
                marker.header.frame_id = "map"
                marker.type = marker.SPHERE
                marker.action = marker.ADD
                marker.scale.x = 0.05
                marker.scale.y = 0.05
                marker.scale.z = 0.05
                marker.color.a = 1.0
                marker.color.r = 0.0 
                marker.color.g = 1.0 
                marker.color.b = 0.0
                marker.pose.orientation.w = 1.0 
                marker.pose.position.x = pose_key_oxy[0]
                marker.pose.position.y = pose_key_oxy[1]
                marker.pose.position.z = 0.0
                marker.id = i
                self.markerArray_key.markers.append(marker)

            self.markersKey_topic.publish(self.markerArray_key)
            
    def view_error(self):
        if (abs(self.odom.pose.pose.position.x - self.amcl.pose.pose.position.x) < 0.5):
            #
            self.count_confiden += 1
            confiden,_,__ = self.find_confidence()
            confiden_err = (100-confiden)/100
            self.confiden_visual.position.x = confiden_err
            self.confiden_visual.position.z = self.count_confiden
            #
            self.confiden_visual.orientation.x = self.odom.pose.pose.position.x - self.amcl.pose.pose.position.x
            self.confiden_visual.orientation.y = self.odom.pose.pose.position.y - self.amcl.pose.pose.position.y
            
            self.confidence_topic.publish(self.confiden_visual)

if __name__ == "__main__":
    confidence = confiden()
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        confidence.visual_scans()
        rate.sleep()