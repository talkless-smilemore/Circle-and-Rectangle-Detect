#!/home/cxy/anaconda3/envs/yolov5/bin/python3.8
import rospy
from cv_bridge import CvBridge
import pyrealsense2 as rs
import cv2 as cv
import numpy as np
import time
from hough_pkg.msg import ImageInfo
from hough_pkg.msg import text

class RealsenseNode:
    def __init__(self):
        # 创建发布者
        self.image_pub = rospy.Publisher('image', ImageInfo, queue_size=10)
        
        self.bridge = CvBridge()

        # 配置 RealSense Pipeline
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # 对齐设置
        self.align_to = rs.stream.color
        self.alignedFs = rs.align(self.align_to)

        self.profile = self.pipeline.start(cfg)

        self.view_image = False

        self.type = 0
        self.num = 0

    def align(self):
        try:
            while not rospy.is_shutdown():
                fs = self.pipeline.wait_for_frames()
                t1 = time.time()
                aligned_frames = self.alignedFs.process(fs)

                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()

                if not depth_frame or not color_frame:
                    continue

                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())
                # 计算 FPS
                t2 = time.time()
                fps = 1 / (t2 - t1)
                #print('fps',fps)
                # 发布图像
                image_info=ImageInfo()
                image_info.header.stamp = rospy.Time.now()
                image_info.color_image = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
                image_info.depth_image = self.bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")
                image_info.type = self.type
                image_info.number = self.num
                self.image_pub.publish(image_info)


                # 显示图像（可选）
                if self.view_image:
                    depth_image = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.03), cv.COLORMAP_JET)
                    images = np.hstack((color_image, depth_image))
                    cv.imshow('window', images)
                    cv.waitKey(1)
        finally:
            self.pipeline.stop()

    def callback(self, obs_msg):
        self.type = obs_msg.type
        print(self.type)
        self.num = obs_msg.number


if __name__ == '__main__':
    rospy.init_node('realsense_node', anonymous=True)
    detect = RealsenseNode()
    rospy.Subscriber("obstacle", text, detect.callback)
    detect.align()
    rospy.spin()
