#!/home/cxy/anaconda3/envs/yolov5/bin/python3.8

import rospy
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import math
import time
from hough_pkg.msg import CircleInfo  
from hough_pkg.msg import ImageInfo
from scipy import stats

class CircleDetector:

    def __init__(self):
        self.bridge = CvBridge()
        self.circles_pub = rospy.Publisher("detected_circle", CircleInfo, queue_size=1)
        self.view_image = True
        self.min_r = 10
        self.max_r = 800
        self.point_number = 30 
        self.view_depth_image = True
        self.minDist = 75 #圆心坐标的距离
        self.param2 = 60 #cv2.HOUGH_GRADIENT方法的累加器阈值。阈值越小，检测到的圆越多
        self.type = 1
        self.num = 0
        self.distance = 5000    #超出这个范围的圆形将检测不到

    def DepthChange(self,image_msg):
        return self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="16UC1")
    
    def CvChange(self, image_msg):
        return self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

    def HsvChange(self, cv_image):
        # 转换为HSV颜色空间
        hsv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)
        # 蓝色范围
        lower_blue = np.array([100, 150, 70])
        upper_blue = np.array([130, 255, 255])
    
        # 红色范围
        lower_red1 = np.array([0, 120, 50])  
        upper_red1 = np.array([15, 255, 255])  
        lower_red2 = np.array([165, 120, 50])  
        upper_red2 = np.array([180, 255, 255])

        # 白色范围
        lower_white = np.array([0, 0,130])
        upper_white = np.array([180, 70, 255])

        # 创建掩码，提取蓝色、红色和白色
        blue_mask = cv.inRange(hsv_image, lower_blue, upper_blue)
        red_mask1 = cv.inRange(hsv_image, lower_red1, upper_red1)
        red_mask2 = cv.inRange(hsv_image, lower_red2, upper_red2)
        red_mask = cv.bitwise_or(red_mask1, red_mask2)
        white_mask = cv.inRange(hsv_image, lower_white, upper_white)
        # 结合三种颜色的掩码
        combined_mask = cv.bitwise_or(blue_mask, red_mask)
        combined_mask = cv.bitwise_or(combined_mask, white_mask)
        result = cv.bitwise_and(cv_image, cv_image, mask=combined_mask)
        return  result 

    def ProcessImg(self, cv_image):
        gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
        gray = cv.medianBlur(gray, 5)
        return gray     
       
    def DetectCircle(self, gray):  
        circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1, self.minDist, param1=100, param2=self.param2, minRadius=self.min_r, maxRadius=self.max_r)
        return circles

    
    
    def GetDepth(self,x, y, radius,depth_image):                         
            depth_values = []
            points = []  # 用于存储采样点坐标
            for i in range(self.point_number):
                # 计算圆周上的点坐标
                angle = 2 * math.pi * i / self.point_number 
                point_x = int(x + radius * math.cos(angle))
                point_y = int(y + radius * math.sin(angle))

                # 检查点是否在深度图像的范围内
                if 0 <= point_x < depth_image.shape[1] and 0 <= point_y < depth_image.shape[0]:
                    depth_value = depth_image[point_y, point_x]  # 获取深度值
                    if depth_value > 0:  # 忽略深度为 0 的点
                        depth_values.append(depth_value)
                        points.append((point_x, point_y))  # 添加采样点坐标
            
            if depth_values:
                depth = stats.mode(depth_values, keepdims=True).mode[0]  # 计算众数
                print(int(depth))
            else:
                depth = 0   # 如果没有有效的深度值，返回0
                print('No valid depth value')
            if self.view_depth_image:
            # 深度图归一化
                depth_image_normalized = cv.normalize(depth_image, None, 0, 255, cv.NORM_MINMAX).astype(np.uint8)
          
            # 可视化深度图像
                depth_image_colored = cv.applyColorMap(depth_image_normalized, cv.COLORMAP_JET)

            # 在深度图上绘制采样点
                for point in points:
                    cv.circle(depth_image_colored, point, 5, (0, 255, 0), -1)  # 绘制绿色圆圈

            # 显示深度图像
                cv.imshow('Depth Image', depth_image_colored)
                cv.waitKey(1)  
            
            return int(depth)
    
    def DrawCircle(self, cv_image, circles,depth_image):
        if circles is not None:
            circles = np.uint16(np.around(circles))
            # 初始化变量用于存储深度最小的圆
            min_depth = float('inf')
            best_circle = None
            
            for i in circles[0, :]:
                center = (i[0], i[1])
                radius = i[2]
                depth = self.GetDepth(center[0], center[1], radius, depth_image)

                # 过滤掉深度大于 self.distance 的圆
                if depth <= self.distance:
                    if depth < min_depth:
                        min_depth = depth
                        best_circle = (center, radius, depth)

            # 如果有符合条件的圆，则绘制最小深度的圆
            if best_circle:
                center, radius, depth = best_circle
                cv.circle(cv_image, center, 1, (0, 100, 100), 3)
                cv.circle(cv_image, center, radius, (255, 0, 255), 3)

                # 发布圆的信息
                circle_info = CircleInfo()
                circle_info.x = int(center[0])
                circle_info.y = int(center[1])
                circle_info.radius = int(radius)
                circle_info.depth = depth
                circle_info.type = self.type
                circle_info.number = self.num
                self.circles_pub.publish(circle_info)  

    def CircleDetect(self, image_msg):
        #print(timestamp)
        t1=time.time()
        self.type = image_msg.type
        self.num = image_msg.number
        depth_image = self.DepthChange(image_msg.depth_image)
        cv_image = self.CvChange(image_msg.color_image)
        hsv_image = self.HsvChange(cv_image)
        gray = self.ProcessImg(hsv_image)
        circles = self.DetectCircle(gray)
        t2=time.time()
        fps = 1 / (t2 - t1)
        #print('fps',fps)
        self.DrawCircle(hsv_image, circles,depth_image)

        # Display the image
        if self.view_image:
            cv.imshow("Detected Circles", hsv_image)
            cv.waitKey(1)
        
        

if __name__ == "__main__":
    rospy.init_node('circle_detector', anonymous=True)
    
    detector = CircleDetector()
    rospy.Subscriber("image", ImageInfo, detector.CircleDetect)
    
    rospy.spin()