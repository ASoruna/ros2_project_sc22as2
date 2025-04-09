
import threading
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.exceptions import ROSInterruptException
import signal

import random
import math
from math import sin, cos
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String




class Robot(Node):
    def __init__(self):
        super().__init__('robot')
        
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.subscription  # prevent unused variable warning

        self.sensitivity = 10
        
        self.green_found = False
        self.blue_found = False 
        self.red_found = False 
        self.blue_contour_area = 0
        self.blue_centre = None
        self.image_width = None 
                
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10)  # 10 Hz
        self.too_close = False
        
        self.min_x, self.max_x = -9.0, 8.0
        self.min_y, self.max_y = -13.0, 5.0
        
        self.goal_publisher = self.create_publisher(PoseStamped, 'project', 10)
        self.cancel_goal = self.create_publisher(String, 'cancel2', 10)


    def callback(self, data):
        image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        self.image_width = image.shape[1]


        # Set the upper and lower bounds for the three colours
        hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
        hsv_red_lower = np.array([0 - self.sensitivity, 100, 100])
        hsv_red_upper = np.array([0 + self.sensitivity, 255, 255])
        hsv_blue_lower = np.array([110 - self.sensitivity, 100, 100]) 
        hsv_blue_upper = np.array([110 + self.sensitivity, 255, 255]) 
        # Convert the rgb image into a hsv image
        Hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Filter out everything but the 3 colours
        green_image = cv2.inRange(Hsv_image, hsv_green_lower, hsv_green_upper)
        red_image = cv2.inRange(Hsv_image, hsv_red_lower, hsv_red_upper)
        blue_image = cv2.inRange(Hsv_image, hsv_blue_lower, hsv_blue_upper) 
        

        contours, _ = cv2.findContours(green_image,mode = cv2.RETR_TREE, method = cv2.CHAIN_APPROX_SIMPLE )
        
        blue_contours, _ = cv2.findContours(blue_image, mode = cv2.RETR_TREE, method = cv2.CHAIN_APPROX_SIMPLE) 
        
        red_contours, _ = cv2.findContours(red_image, mode = cv2.RETR_TREE, method = cv2.CHAIN_APPROX_SIMPLE) 
        
        self.green_found = False
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 100: 
                (x, y), radius = cv2.minEnclosingCircle(c)
                center = (int(x),int(y)) 
                radius = int(radius) 

                cv2.circle(image,center,radius,(255,255,0) ,1)
                cv2.putText(image, "Green",(int(x) + int(radius),  int(y) + int(radius)), cv2.FONT_HERSHEY_SIMPLEX, 3.0, (255, 255, 255), thickness = 3)
                self.green_found = True
            else:
                self.green_found = False
                
        self.blue_found = False 
        if len(blue_contours) > 0:
            c_blue = max(blue_contours, key=cv2.contourArea)
            self.blue_contour_area = cv2.contourArea(c_blue)
            if self.blue_contour_area > 100:
                #Cancel current goal if blue box is found
                cancel_msg = String()
                cancel_msg.data = "cancel"
                self.cancel_goal.publish(cancel_msg)

                (x_blue, y_blue), radius_blue = cv2.minEnclosingCircle(c_blue)
                centre_blue = (int(x_blue),int(y_blue)) 
                self.blue_centre = centre_blue
                radius_blue = int(radius_blue) 

                cv2.circle(image,centre_blue,radius_blue,(255,255,0) ,1)
                cv2.putText(image, "Blue",(int(x_blue) + int(radius_blue),  int(y_blue) + int(radius_blue)), cv2.FONT_HERSHEY_SIMPLEX, 3.0, (255, 255, 255), thickness = 3)
                self.blue_found = True
            else:
                self.blue_found = False 
                
        self.red_found = False 
        if len(red_contours) > 0:
        	
            c_red = max(red_contours, key=cv2.contourArea)
            self.red_contour_area = cv2.contourArea(c_red)
            if self.red_contour_area > 100: 
                (x_red, y_red), radius_red = cv2.minEnclosingCircle(c_red)
                centre_red = (int(x_red),int(y_red)) 
                radius_red = int(radius_red) 

                cv2.circle(image,centre_red,radius_red,(255,255,0) ,1)
                cv2.putText(image, "Red",(int(x_red) + int(radius_red),  int(y_red) + int(radius_red)), cv2.FONT_HERSHEY_SIMPLEX, 3.0, (255, 255, 255), thickness = 3)

                self.red_found = True
            else:
                self.red_found = False 
                
        #Show the resultant images you have created. You can show all of them or just the end result if you wish to.
        cv2.namedWindow('robot_camera',cv2.WINDOW_NORMAL) 
        cv2.resizeWindow('robot_camera',320,240)
        cv2.imshow('robot_camera', image)
        cv2.waitKey(1)
        

    def approach_blue(self):

        desired_area = 580000  
        centre_x = self.image_width // 2  # the center of the image
	    
        if not self.blue_found or self.blue_centre is None:
            self.stop()
            return
	    
        c_x, _ = self.blue_centre
        area = self.blue_contour_area
	    
        offset = c_x - centre_x  
        ang_vel = -0.002 * offset  
	    
        tolerance = 20000  # field to start to slow robot down
        if area < desired_area - tolerance:
            lin_vel = 0.2  
        elif area > desired_area + tolerance:
            lin_vel = -0.1 
        else:
            lin_vel = 0.0 
	    
        twist = Twist()
        twist.linear.x = lin_vel
        twist.angular.z = ang_vel
        self.publisher.publish(twist)
    
    def explore_random_goal(self):
        random_x = random.uniform(self.min_x, self.max_x)
        random_y = random.uniform(self.min_y, self.max_y)
        random_yaw = random.uniform(-math.pi, math.pi)
        
        goal_msg = PoseStamped()
        goal_msg.pose.position.x = random_x
        goal_msg.pose.position.y = random_y
        goal_msg.pose.orientation.z = sin(random_yaw / 2)
        goal_msg.pose.orientation.w = cos(random_yaw / 2)
        
        self.goal_publisher.publish(goal_msg)
        

def main():
    def signal_handler(sig, frame):
        robot.stop()
        rclpy.shutdown()
    
    rclpy.init(args=None)
    robot = Robot()
    
    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            if robot.blue_found:
                robot.approach_blue()
            else:
                robot.explore_random_goal()
            rclpy.spin_once(robot, timeout_sec=0.1)

    except ROSInterruptException:
        pass

    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    main()


