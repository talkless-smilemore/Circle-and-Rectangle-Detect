#!/home/cxy/anaconda3/envs/yolov5/bin/python3.8

import rospy
from circle_detection_node import CircleDetector
from rectangle_detection_node import RectangleDetector
from hough_pkg.msg import ImageInfo

class Detector:
    def __init__(self):
        self.type = 1
        self.circle_detector = CircleDetector()  
        self.rectangle_detector = RectangleDetector()  
   
    def IsDetect(self, obs_msg):
            if self.type == 0:
               
                self.circle_detector.CircleDetect(obs_msg)
            else:
               
                self.rectangle_detector.RectangleDetect(obs_msg)


    def detect(self,obs_msg):
       
        self.type = obs_msg.type
        if self.type != -1:
             self.IsDetect(obs_msg)
        
                 

if __name__ == "__main__":
    rospy.init_node('detector', anonymous=True)
    detect = Detector()
    rospy.Subscriber("image", ImageInfo, detect.detect)
    rospy.spin()