#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraStream(Node):
    def __init__(self):
        super().__init__("camera_stream")

        self.publisher = self.create_publisher(Image, "camera_frames", 10)

        
        self.declare_parameter('camera_source', '0')  
        self.declare_parameter('frame_rate', 15.0)

        source_param = self.get_parameter('camera_source').value
        self.frame_rate = self.get_parameter('frame_rate').value

        
        try:
            source = int(source_param)
            self.get_logger().info(f"✅ Using webcam index: {source}")
        except ValueError:
            source = source_param 
            self.get_logger().info(f"✅ Using video file: {source}")

        self.capture = cv2.VideoCapture(source)
        if not self.capture.isOpened():
            self.get_logger().error(f"❌ Failed to open source: {source}")
            return

        self.bridge = CvBridge()
        self.frame_id = 0

        
        self.timer = self.create_timer(1.0 / self.frame_rate, self.publish_frame)
        self.get_logger().info(f"🚀 Camera Node Started at {self.frame_rate} FPS")

    def publish_frame(self):
        ret, frame = self.capture.read()
        if not ret:
            self.get_logger().warn("⚠️ Video ended or frame not received")
            return

       
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.frame_id)

        self.publisher.publish(msg)
        self.get_logger().info(f"📤 Published frame {self.frame_id}")
        self.frame_id += 1

def main(args=None):
    rclpy.init(args=args)
    node = CameraStream()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.capture.release()
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
