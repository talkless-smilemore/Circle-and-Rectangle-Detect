#!/home/cxy/anaconda3/envs/yolov5/bin/python3.8
import rospy
from cv_bridge import CvBridge
import numpy as np
from hough_pkg.msg import CircleInfo
from hough_pkg.msg import Circleresult
from hough_pkg.msg import RectangleInfo
from hough_pkg.msg import Rectangleresult

switch =  0#0是圆形检测，1是矩形检测
class ImageListener:
    def __init__(self):
        self.bridge = CvBridge()
        self.detected_figure = {}  # 存储检测到的的时间戳和信息
        self.circle_publisher = rospy.Publisher('detected_circles_coordinate', Circleresult, queue_size=10)
        self.rectangle_publisher = rospy.Publisher('detected_rectangle_coordinate', Rectangleresult, queue_size=10)
        self.point_number = 10        
        self.intrinsics = np.array([
            [379.7127380371094, 0.0, 322.19525146484375],
            [0.0, 379.7127380371094, 237.7461700439453],
            [0, 0, 1]
        ])
        self.sync = False
        self.type = 0
        self.num = 0

    def CircleTransformCoordinate(self, x, y, radius,depth):
        
            z_c = depth
            x_c = (x - self.intrinsics[0][2]) * z_c / self.intrinsics[0][0]
            y_c = (y - self.intrinsics[1][2]) * z_c / self.intrinsics[1][1]
            
            circle_info = Circleresult()
            circle_info.x = int(x_c)
            circle_info.y = int(y_c)
            circle_info.z = int(z_c)
            circle_info.radius = int(radius)
            circle_info.type = self.type
            circle_info.number = self.num
            #print(z_c)
            self.circle_publisher.publish(circle_info)
            
        
        
    def RectangleTransformCoordinate(self,x , y ,depth):
            z_c = depth
            x_c = (x - self.intrinsics[0][2]) * z_c / self.intrinsics[0][0]
            y_c = (y - self.intrinsics[1][2]) * z_c / self.intrinsics[1][1]
            rectangle_info = Rectangleresult()
            rectangle_info.x = int(x_c)
            rectangle_info.y = int(y_c)
            rectangle_info.z= z_c
            rectangle_info.type = self.type
            rectangle_info.number = self.num
            self.rectangle_publisher.publish(rectangle_info)    
            
    def Callback(self,coordinates_msg):
        global switch
        self.type = coordinates_msg.type
        self.num = coordinates_msg.number
        if self.type == 0:
            switch = 0
        else:
            switch = 1
        
        if switch == 0:
            self.CircleTransformCoordinate(coordinates_msg.x,coordinates_msg.y,coordinates_msg.radius,coordinates_msg.depth)
        else:
             self.RectangleTransformCoordinate(coordinates_msg.x,coordinates_msg.y,coordinates_msg.depth)

if __name__ == '__main__':
    rospy.init_node('show_depth', anonymous=True)
    listener = ImageListener()
    

    if switch == 0 :
        rospy.Subscriber('detected_circle', CircleInfo, listener.Callback)
    
    else:
        rospy.Subscriber('detected_rectangles', RectangleInfo, listener.Callback) 
    
    # while not rospy.is_shutdown():

    #     pass
    rospy.spin()
    #t1 = time.tims()
    #t2 = time.tims()
    #fps = 1 / (t2 - t1)