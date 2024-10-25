# Circle-and-Rectangle-Detect
这个项目能够检测到圆形和矩形框，并且利用深度相机d435，获取深度值，最后发布圆形或者矩形框在相机系下的中心坐标
This project is able to detect both circular and rectangular frames, and uses the depth camera D435 to obtain the depth value, and finally publish the center coordinates of the circular or rectangular frame under the camera system
## text.py
以话题的形式的形式发布障碍物信息，包括障碍物的类型和编号
Publish obstacle information in the form of a topic, including the type and number of the obstacle
## align.py
接受障碍物的信息，同时接受深度相机传入的深度流和彩色流信息，将深度图向彩色图对齐，随后发布对齐后的深度图和彩色图及障碍物信息
Recept the information of the obstacle, Recept the depth stream and color stream information from the depth camera at the same time, aligns the depth map to the color map, and then publishes the aligned depth map and color map and obstacle information
## detect.py
接受align.py发出的话题后，根据障碍物的类型来进行什么检测(调用circle_detection_node.py或者rectangle_detection_node.py进行检测）
After recept the topic, what detection is done depending on the type of obstacle (call circle_detection_node.py or rectangle_detection_node.py to detect)
## circle_detection_node.py
圆环检测,并发布圆心在像素系下的坐标
The circle is detected and the coordinates of its center in the pixel system are published.
## rectangle_detection_node.py
矩形框检测，并发布矩形中心在像素系下的坐标
Rectangle detection and publish the coordinates of the rectangle center in pixel system.
## depth_node.py
接受到坐标信息后，通过坐标变换把在像素系下的坐标转换到相机系下，并发布出去
After receiving the coordinate information, the coordinate transformation is performed to convert the coordinates in the pixel system to the camera system, and the result is published.
