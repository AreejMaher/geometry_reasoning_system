import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

class KeypointDetection(Node):
    def __init__(self):
        super().__init__("keypoint_detection")

        #subscriber
        self.subscription = self.create_subscription(Image, "/camera_frames", self.camera_callback, 10)
        self.bridge = CvBridge()

        #publisher
        self.publisher_ = self.create_publisher(Float32MultiArray, '/keypoints', 10)

        #parameter
        self.declare_parameter("max_keypoints", 500)
        max_kp = self.get_parameter("max_keypoints").value

        #orb initialization
        self.orb = cv2.ORB_create(nfeatures=max_kp)

        self.get_logger().info("Keypoint Detection Node Started")


    def camera_callback(self, img):
        current_frame = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
        gray_frame = cv2.cvtColor(current_frame,cv2.COLOR_BGR2GRAY)

        keypoints = self.orb.detect(gray_frame, None)

        kp_coords = []
        for kp in keypoints:
            kp_coords.extend([float(kp.pt[0]), float(kp.pt[1])])

        msg = Float32MultiArray()
        msg.data = kp_coords
        self.publisher_.publish(msg)
        self.get_logger().info(f"Detected {len(keypoints)} keypoints")

        if len(keypoints) < 20:
            self.get_logger().warn(f"RELIABILITY ALERT: Only {len(keypoints)} keypoints (Minimum 20 required)")
        else:
            self.get_logger().info(f"Published {len(keypoints)} keypoints")
            
def main(args=None):
    rclpy.init(args=args)
    node = KeypointDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()