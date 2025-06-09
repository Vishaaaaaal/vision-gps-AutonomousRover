# === ROS 2 DEPTH + OBJECT DETECTION WITH SOCKET PUBLISH ===

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
from ultralytics import YOLO
import torch
import os
import socket

DEPTH_THRESHOLD = 2.5
ROAD_LENGTH = 4.0
ROAD_WIDTH = 2.0

class YOLODepthObstacleDetector(Node):
    def __init__(self):
        super().__init__('yolo_depth_obstacle_detector')
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(self.device)

        self.prev_state = None  # track "stop" or "start"
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.socket.connect(('127.0.0.1', 5050))
            self.get_logger().info("✅ Connected to ROS 1 socket listener")
        except:
            self.get_logger().warn("⚠️ Could not connect to socket")

        self.marker_pub = self.create_publisher(MarkerArray, '/detected_objects', 10)
        self.lane_pub = self.create_publisher(MarkerArray, '/lane_markers', 10)

        self.depth_sub = self.create_subscription(Image, '/zed/zed_node/depth/depth_registered', self.depth_callback, 10)
        self.image_sub = self.create_subscription(Image, '/zed/zed_node/rgb/image_rect_color', self.image_callback, 10)

        self.latest_image = None
        self.latest_depth = None
        cv2.namedWindow("Obstacle Detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Obstacle Detection", 1280, 720)

    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.latest_depth is not None:
            self.detect_obstacles()
            self.visualize_lane()

    def send_signal(self, state):
        if state != self.prev_state:
            try:
                self.socket.sendall(state.encode())
                self.get_logger().info(f"📤 Sent signal: {state}")
                self.prev_state = state
            except:
                self.get_logger().warn("⚠️ Socket send failed")

    def detect_obstacles(self):
        vis_image = self.latest_image.copy()
        results = self.model(vis_image, conf=0.5)
        marker_array = MarkerArray()
        stop = False

        for idx, box in enumerate(results[0].boxes):
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2
            object_depth = float(self.latest_depth[center_y, center_x])

            if 0 < object_depth <= DEPTH_THRESHOLD:
                stop = True
                label = self.model.names[int(box.cls[0])]
                cv2.rectangle(vis_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(vis_image, f"{label} ({object_depth:.2f}m)", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        self.send_signal("stop" if stop else "start")

        if stop:
            cv2.putText(vis_image, "STOP", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)

        cv2.imshow("Obstacle Detection", vis_image)
        cv2.waitKey(1)

    def visualize_lane(self):
        lane_markers = MarkerArray()
        for i in range(10):
            marker = Marker()
            marker.header.frame_id = "zed_camera_center"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = ROAD_WIDTH
            marker.scale.z = 0.05
            marker.color.a = 0.5
            marker.color.b = 1.0
            marker.pose.position.x = i * 0.4
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0
            lane_markers.markers.append(marker)
        self.lane_pub.publish(lane_markers)

def main(args=None):
    rclpy.init(args=args)
    node = YOLODepthObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

