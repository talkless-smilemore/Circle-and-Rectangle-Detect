# Circle-and-Rectangle-Detect
这个项目能够检测到圆形和矩形框，并且利用深度相机d435，获取深度值，最后发布圆形或者矩形框在相机系下的中心坐标
## text.py
发布以话题的形式发布障碍物信息，包括障碍物的类型和编号
## align.py
接受障碍物的信息，同时接受深度相机传入的深度流和彩色流信息，将深度图向彩色图对齐，随后发布对齐后的深度图和彩色图及障碍物信息
## detect.py
接受align.py发出的话题后，根据障碍物的类型来进行什么检测(调用circle_detection_node.py或者rectangle_detection_node.py进行检测）
## circle_detection_node.py
圆环检测,并发布圆心在像素系下的坐标
## rectangle_detection_node.py
矩形框检测，并发布矩形中心在像素系下的坐标
## depth_node.py
接受到坐标信息后，通过坐标变换把在像素系下的坐标转换到相机系下，并发布出去
