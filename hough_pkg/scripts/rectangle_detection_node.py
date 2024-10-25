#!/home/cxy/anaconda3/envs/yolov5/bin/python3.8
import rospy
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import time
from scipy import stats
from hough_pkg.msg import RectangleInfo
from hough_pkg.msg import ImageInfo

class RectangleDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("detected_rectangles", RectangleInfo, queue_size=1)
        self.view_image = True
        self.detect_way = 0
        self.min_width = 20
        self.max_width = 3000
        self.min_height = 20
        self.max_height = 3000
        self.vertices_x = []  # 初始化 vertices_x
        self.vertices_y = [] 
        self.point_number = 20
        self.view_depth_image = True
        self.middle_distance = 50
        self.depth_difference = 50
        self.type = 0  #1是左，2是右，3是上，4是下
        self.num = 0
        self.filter = 20 #去掉可能重叠的小矩形
        self.distance = 5000    #超出这个范围的矩形将检测不到
        
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

    def ProcessImg(self, hsv_image):
        gray = cv.cvtColor(hsv_image, cv.COLOR_BGR2GRAY)
        gray = cv.medianBlur(gray, 5)
        return gray
    

    def GetDepth(self,vertices_x, vertices_y,depth_image):
        depths = []
        points = []  # 用于存储采样点坐标
            # 在每条边上均匀采样点
        for i in range(len(vertices_x)):
            start_x = vertices_x[i]
            start_y = vertices_y[i]
            end_x = vertices_x[(i + 1) % len(vertices_x)]
            end_y = vertices_y[(i + 1) % len(vertices_y)]

            for j in range(self.point_number + 1):
                t = j / self.point_number
                sample_x = int(start_x * (1 - t) + end_x * t)
                sample_y = int(start_y * (1 - t) + end_y * t)

                    # 获取深度值并添加到列表
                depth = depth_image[sample_y, sample_x]  # 假设深度图为二维数组
                depths.append(depth)
                points.append((sample_x, sample_y))
            # 计算深度值的平均值
        if depths:
            depth = stats.mode(depth, keepdims=True).mode[0]  # 计算众数
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
    
    def filter_large_rectangles(self, rectangles):
        if not rectangles:
            return []  # 如果没有矩形，直接返回空列表

        areas = [cv.contourArea(rect) for rect in rectangles]
        
        if len(areas) == 0:
            return []  # 如果没有任何矩形的面积，直接返回空列表
        
        # 设置一个面积阈值，用来过滤掉可能是由两个小矩形合成的大矩形
        max_allowed_area = sum(areas) / len(areas) * 1.5  # 设置一个合理的阈值，例如平均面积的1.5倍
        
        filtered_rectangles = []
        for rect, area in zip(rectangles, areas):
            if area <= max_allowed_area:  # 只保留面积小于阈值的矩形
                filtered_rectangles.append(rect)
        
        return filtered_rectangles
    

    # 重叠过滤函数
    def filter_overlapping_rectangles(self, rectangles):
        filtered_rectangles = rectangles[:]
        
        for i in range(len(rectangles)):
            for j in range(i + 1, len(rectangles)):
                iou = self.calculate_iou(rectangles[i], rectangles[j])
                # 如果两个矩形的IoU值超过某个阈值（例如0.8），则认为其中一个是合成的大矩形
                if iou > 0.8:
                    # 根据面积过滤掉较大的那个矩形
                    area_i = cv.contourArea(rectangles[i])
                    area_j = cv.contourArea(rectangles[j])
                    
                    if area_i > area_j:
                        filtered_rectangles.remove(rectangles[i])
                    else:
                        filtered_rectangles.remove(rectangles[j])
        
        return filtered_rectangles

    # 计算IoU的辅助函数
    def calculate_iou(self, rect1, rect2):
        x1, y1, x2, y2 = cv.boundingRect(rect1)
        x3, y3, x4, y4 = cv.boundingRect(rect2)

        # 找到相交区域的边界
        xi1 = max(x1, x3)
        yi1 = max(y1, y3)
        xi2 = min(x2, x4)
        yi2 = min(y2, y4)

        inter_area = max(0, xi2 - xi1) * max(0, yi2 - yi1)

        # 计算两个矩形的面积
        rect1_area = (x2 - x1) * (y2 - y1)
        rect2_area = (x4 - x3) * (y4 - y3)

        # 计算并集的面积
        union_area = rect1_area + rect2_area - inter_area

        # 计算IoU
        return inter_area / union_area if union_area > 0 else 0
    def publish_rectangle(self, rectangle, depth_image):
        center_x = (rectangle[:, 0, 0].min() + rectangle[:, 0, 0].max()) // 2
        center_y = (rectangle[:, 0, 1].min() + rectangle[:, 0, 1].max()) // 2

        rectangle_info = RectangleInfo()
        rectangle_info.header.stamp = rospy.Time.now()  # 获取当前时间戳
        rectangle_info.x = center_x  # 中心点x坐标
        rectangle_info.y = center_y  # 中心点y坐标
        rectangle_info.type = self.type
        rectangle_info.number = self.num
        rectangle_info.depth = self.GetDepth([point[0, 0] for point in rectangle], 
                                          [point[0, 1] for point in rectangle], 
                                          depth_image)
        print('sucess publish')
        self.image_pub.publish(rectangle_info)
    
    def DetectRectangles(self, gray, depth_image):  
        # 使用自适应直方图均衡化来增强对比度
        clahe = cv.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        enhanced_gray = clahe.apply(gray)
        blurred = cv.GaussianBlur(enhanced_gray, (5, 5), 0)
    # 使用Canny边缘检测
        edges = cv.Canny(blurred, 50, 150, apertureSize=3)     
        edges = cv.GaussianBlur(edges, (5, 5), 0)  # 应用于Canny边缘后
    # 使用膨胀和腐蚀操作增强边缘
        kernel = np.ones((3,3), np.uint8)
        edges = cv.dilate(edges, kernel, iterations=1)
        edges = cv.erode(edges, kernel, iterations=1)
        
        cv.imshow("Edge Detection", edges)  # 显示边缘检测图像    
        cv.waitKey(1)  # 等待1毫秒
    # 找到直线图像中的轮廓
        contours, _ = cv.findContours(edges, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        rectangles = []
        
        for cnt in contours:
            epsilon = 0.02 * cv.arcLength(cnt, True)
            approx = cv.approxPolyDP(cnt, epsilon, True)

            if len(approx) == 4 and self.IsRightAngle(approx[:, 0]):
                xmin, ymin = np.min(approx[:, 0, 0]), np.min(approx[:, 0, 1])
                xmax, ymax = np.max(approx[:, 0, 0]), np.max(approx[:, 0, 1])

                width = xmax - xmin
                height = ymax - ymin

                if (self.min_width <= width <= self.max_width) and (self.min_height <= height <= self.max_height):
                    x_coords = [point[0, 0] for point in approx]
                    y_coords = [point[0, 1] for point in approx]
                    depth = self.GetDepth(x_coords, y_coords, depth_image)  # 获取中心点的深度
                    rectangles.append((approx, depth, x_coords, y_coords,))

        if not rectangles:
            return rectangles

        # 按照深度值从小到大排序，取最前方的矩形
        rectangles.sort(key=lambda r: r[1])

        # 获取最近的深度
        closest_depth = rectangles[0][1]
        # 筛选出距离最近且深度相同的矩形
        rectangles = [r for r in rectangles if r[1] == closest_depth and closest_depth < self.distance] #超出这个范围的矩形将检测不到
        
        
        if len(rectangles) == 1:
            print('1')
            self.publish_rectangle(rectangles[0][0], depth_image)  

        elif len(rectangles) >= 2:
            if abs(rectangles[0][0][:, 0, 0].min() - rectangles[1][0][:, 0, 0].min()) > abs(rectangles[0][0][:, 0, 1].min() - rectangles[1][0][:, 0, 1].min()):
                # 根据xmin排序，处理左右矩形
                rectangles.sort(key=lambda r: r[0][:, 0, 0].min())  # 根据xmin排序
                filtered_rectangles = []
                for i in range(len(rectangles) - 1):
                    if abs(rectangles[i][0][:, 0, 0].min() - rectangles[i + 1][0][:, 0, 0].min()) > self.filter:
                        filtered_rectangles.append(rectangles[i])

                if len(filtered_rectangles) >= 2:
                    left_rectangle = filtered_rectangles[0]
                    right_rectangle = filtered_rectangles[1]

                    xmin_left, ymin_left, xmax_left, ymax_left = left_rectangle[0][:, 0, 0].min(), left_rectangle[0][:, 0, 1].min(), left_rectangle[0][:, 0, 0].max(), left_rectangle[0][:, 0, 1].max()
                    xmin_right, ymin_right, xmax_right, ymax_right = right_rectangle[0][:, 0, 0].min(), right_rectangle[0][:, 0, 1].min(), right_rectangle[0][:, 0, 0].max(), right_rectangle[0][:, 0, 1].max()

                    if (abs(xmax_left - xmin_right) <= self.middle_distance):
                        if self.type == 1:
                            print('left')
                            self.publish_rectangle(left_rectangle[0], depth_image)  # 访问 approx
                            return left_rectangle  # 返回左边的矩形

                        elif self.type == 2:
                            print('right')
                            self.publish_rectangle(right_rectangle[0], depth_image)  # 访问 approx
                            return right_rectangle  # 返回右边的矩形
                else:
                    rospy.logwarn("No rectangles detected")
                    return rectangles

            else:
                # 根据ymin排序，处理上下矩形
                rectangles.sort(key=lambda r: r[0][:, 0, 1].min())  # 根据 ymin 排序
                
                # 去重
                filtered_rectangles = []
                for i in range(len(rectangles) - 1):
                    if abs(rectangles[i][0][:, 0, 1].min() - rectangles[i + 1][0][:, 0, 1].min()) > 20:
                        filtered_rectangles.append(rectangles[i])

                if len(filtered_rectangles) >= 2:
                    top_rectangle = filtered_rectangles[0]
                    bottom_rectangle = filtered_rectangles[1]

                    xmin_top, ymin_top, xmax_top, ymax_top = top_rectangle[0][:, 0, 0].min(), top_rectangle[0][:, 0, 1].min(), top_rectangle[0][:, 0, 0].max(), top_rectangle[0][:, 0, 1].max()
                    xmin_bottom, ymin_bottom, xmax_bottom, ymax_bottom = bottom_rectangle[0][:, 0, 0].min(), bottom_rectangle[0][:, 0, 1].min(), bottom_rectangle[0][:, 0, 0].max(), bottom_rectangle[0][:, 0, 1].max()

                    if (abs(ymax_top - ymin_bottom) <= self.middle_distance):
                        if self.type == 3:
                            print('top')
                            self.publish_rectangle(top_rectangle[0], depth_image)  # 访问 approx
                            return top_rectangle  # 返回上方的矩形

                        elif self.type == 4:
                            print('bottom')
                            self.publish_rectangle(bottom_rectangle[0], depth_image)  # 访问 approx
                            return bottom_rectangle  # 返回下方的矩形

                else:
                    rospy.logwarn("No rectangles detected")
                    return rectangles

        return rectangles  # 如果没有找到合适的矩形，返回空列表


    def IsRightAngle(self, points):
    # 计算角度
        angles = []
        for i in range(4):
            a = np.array(points[i]) - np.array(points[(i + 1) % 4])
            b = np.array(points[(i + 2) % 4]) - np.array(points[(i + 1) % 4])
            cosine_angle = np.clip(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)), -1.0, 1.0)
            angle = np.degrees(np.arccos(cosine_angle))
            angles.append(angle)
        return all(85 < angle < 95 for angle in angles)  # 确保每个角接近90度

    def DrawRectangles(self, cv_image, rectangles):
        for rect in rectangles:
            contours = rect[0]
            cv.drawContours(cv_image, [contours ], -1, (0, 255, 0), 2)

    def RectangleDetect(self, image_msg):
        t1=time.time()
        self.type = image_msg.type
        self.num = image_msg.number
        depth_image = self.DepthChange(image_msg.depth_image)
        cv_image = self.CvChange(image_msg.color_image)
        hsv_image = self.HsvChange(cv_image)
        gray = self.ProcessImg(hsv_image)
        rectangles = self.DetectRectangles(gray,depth_image)
        self.DrawRectangles(hsv_image, rectangles)
        t2=time.time()
        fps = 1 / (t2 - t1)
        #print('fps',fps)
        if self.view_image:
            cv.imshow("Detected Rectangles", hsv_image)
            cv.waitKey(1)           

if __name__ == "__main__":
    rospy.init_node('rectangle_detector', anonymous=True)
    detector = RectangleDetector()
    rospy.Subscriber("image", ImageInfo, detector.RectangleDetect)
    rospy.spin()