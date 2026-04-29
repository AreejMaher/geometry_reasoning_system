import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from geometry_reasoning_interface.msg import Descriptor, DescriptorList
import numpy as np

class DescriptorExtraction(Node):
    def __init__(self):
        super().__init__("descriptor_extraction")

        self.kp_sub = self.create_subscription(Float32MultiArray, '/keypoints', self.keypoint_callback, 10)
        self.cam_sub = self.create_subscription(Image, "/camera_frames", self.camera_callback, 10)

        self.publisher_ = self.create_publisher(DescriptorList, '/descriptors', 10)

        self.bridge = CvBridge()
        self.orb = cv2.ORB_create()
        self.current_frame = None

        self.get_logger().info("Descriptor Extraction Node Started")

    def camera_callback(self, msg):
        self.current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def keypoint_callback(self, msg):
        if self.current_frame is None:
            return
        
        raw_data = msg.data
        keypoints = []
        for i in range(0, len(raw_data), 2):
            kp = cv2.KeyPoint(x=float(raw_data[i]), y=float(raw_data[i+1]), size=20.0)
            keypoints.append(kp)

        gray_frame = cv2.cvtColor(self.current_frame,cv2.COLOR_BGR2GRAY)
        keypoints, descriptors = self.orb.compute(gray_frame, keypoints)

        if descriptors is not None:
            list_msg = DescriptorList()
            list_msg.header.stamp = self.get_clock().now().to_msg()
            list_msg.header.frame_id = "camera"

            for i in range(len(keypoints)):
                desc_obj = Descriptor()
                desc_obj.x = float(keypoints[i].pt[0])
                desc_obj.y = float(keypoints[i].pt[1])
                desc_obj.descriptor = descriptors[i].tolist()

                list_msg.descriptors.append(desc_obj)

            self.publisher_.publish(list_msg)
            self.get_logger().info(f"Published {len(list_msg.descriptors)} descriptors")

        img2 = cv2.drawKeypoints(self.current_frame, keypoints, None, color=(0,255,0), flags=0)

        cv2.imshow("Descriptor Extraction Stream", img2)
        cv2.waitKey(1)



def main(args=None):
    rclpy.init(args=args)
    node = DescriptorExtraction()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()