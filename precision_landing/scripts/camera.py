#!/usr/bin/env python3
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from custom_msgs.msg import CPoints
from geometry_msgs.msg import Point
import cv2 # OpenCV library
import numpy as np

def draw_and_get_centers(current_frame, corners, ids):
  centers = []
  if len(corners) > 0:
    # flatten the ArUco IDs list
    ids = ids.flatten()
    # loop over the detected ArUCo corners
    for (markerCorner, markerID) in zip(corners, ids):
      # extract the marker corners (which are always returned in
      # top-left, top-right, bottom-right, and bottom-left order)
      corners = markerCorner.reshape((4, 2))
      (topLeft, topRight, bottomRight, bottomLeft) = corners
      # convert each of the (x, y)-coordinate pairs to integers
      topRight = (int(topRight[0]), int(topRight[1]))
      bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
      bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
      topLeft = (int(topLeft[0]), int(topLeft[1]))
      # draw the bounding box of the ArUCo detection
      cv2.line(current_frame, topLeft, topRight, (0, 255, 0), 2)
      cv2.line(current_frame, topRight, bottomRight, (0, 255, 0), 2)
      cv2.line(current_frame, bottomRight, bottomLeft, (0, 255, 0), 2)
      cv2.line(current_frame, bottomLeft, topLeft, (0, 255, 0), 2)
      # compute and draw the center (x, y)-coordinates of the ArUco
      # marker
      cX = int((topLeft[0] + bottomRight[0]) / 2.0)
      cY = int((topLeft[1] + bottomRight[1]) / 2.0)
      centers.append((cX, cY))
      cv2.circle(current_frame, (cX, cY), 4, (0, 0, 255), -1)
      # draw the ArUco marker ID on the image
      cv2.putText(current_frame, str(markerID),
        (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
        0.5, (0, 255, 0), 2)
      print("[INFO] ArUco marker ID: {}".format(markerID))
    return centers

class Camera(Node):
  def __init__(self):
    super().__init__('camera_node')
    self.cX, self.cY = 0, 0 
    self.frcX, self.frcY = 0, 0
    self.width, self.height = 1, 1 # Prevent division by zero
    self.subscription = self.create_subscription(
      Image, 
      f'/camera', 
      self.listener_callback, 
      10)
    self.publisher_ = self.create_publisher(CPoints, '/aruco_center', 10)
    self.br = CvBridge()
    self.timer = self.create_timer(0.1, self.timer_callback)
    self.is_exists = False

  def listener_callback(self, data):
    """
    Callback function.
    """
    if data is None:
      self.get_logger().info("No image data received.")
      return
    
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)
    
    gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
    self.width = gray.shape[1]
    self.height = gray.shape[0]
    self.frcX, self.frcY = self.width // 2, self.height // 2
    cv2.circle(current_frame, (self.frcX, self.frcY), 4, (255, 0, 0), -1) # Draw center of the frame for reference

    # aruco detection processes
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, rejected = detector.detectMarkers(gray)
    self.is_exists = False
    if ids is not None:
      self.is_exists = True
      # We need to get centers of detected aruco, so we will publish to flight controller for precision landing.    
      centers = draw_and_get_centers(current_frame, corners, ids)
      first_C = centers[0]
      self.cX, self.cY = first_C
      self.get_logger().info(f"Center of the first detected ArUco marker: {self.cX}, {self.cY}")  
    
    cv2.imshow(f"camera", current_frame)
    cv2.waitKey(1)

  def timer_callback(self):
    point_msg = CPoints()
    point_msg.detected = True
    if not self.is_exists:
      point_msg.detected = False
  
    
    # Sayilarin 0'a bolunmesini onlemek ve tam float oldugundan emin olmak icin:
    width = max(1.0, float(self.width))
    height = max(1.0, float(self.height))
    
    # Piksellerin (0 ile resim boyutu) -> (0.0 ile 2.0) araligina standardize edilmesi
    point_msg.box_cx = float(self.cX) / width * 2.0
    point_msg.box_cy = float(self.cY) / height * 2.0
    point_msg.cx = float(self.frcX) / width * 2.0
    point_msg.cy = float(self.frcY) / height * 2.0
    
    
    # Gonderilen veriyi logla:
    self.get_logger().info(f"Publishing CPoints: cx={point_msg.cx:.2f}, cy={point_msg.cy:.2f}, box_cx={point_msg.box_cx:.2f}, box_cy={point_msg.box_cy:.2f}")
    
    self.publisher_.publish(point_msg)

def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  camera_bridge = Camera()
  
  # Spin the node so the callback function is called.
  rclpy.spin(camera_bridge)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  camera_bridge.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()