#!/home/cxy/anaconda3/envs/yolov5/bin/python3.8
import rospy
from hough_pkg.msg import text

class Text:
    def __init__(self):
        self.pub = rospy.Publisher('obstacle', text, queue_size=10)
        self.type = 0
        self.num = 0
    def publish_message(self):
        t_info = text()
        t_info.type = self.type
        t_info.number = self.num
        self.pub.publish(t_info)
        rospy.loginfo("Published: type=%d, number=%d", self.type, self.num)

if __name__ == '__main__':
    rospy.init_node('text_node', anonymous=True)
    detect = Text()

    rate = rospy.Rate(10)  # 设置发布频率为 10Hz
    while not rospy.is_shutdown():
        detect.publish_message()  # 调用发布函数
        rate.sleep()  # 控制发布频率