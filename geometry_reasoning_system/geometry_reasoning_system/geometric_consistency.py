import rclpy
from rclpy.node import Node
import cv2
import numpy as np


from geometry_reasoning_interface.msg import FeatureMatchList, GeometricInliers
from geometry_reasoning_interface.srv import CheckGeometry

class GeometricConsistency(Node):
    def __init__(self):
        super().__init__("geometric_consistency")

        # Parameters
        # Threshold for RANSAC (maximum distance in pixels for a point to be an inlier)
        self.declare_parameter('inlier_threshold', 3.0)

        # Subscribers
        self.subscription = self.create_subscription(
            FeatureMatchList, 
            '/filtered_matches', 
            self.match_callback, 
            10
        )

        # Publishers
        self.publisher = self.create_publisher(
            GeometricInliers, 
            '/geometric_inliers', 
            10
        )

        # Services Provided
        self.srv = self.create_service(
            CheckGeometry, 
            '/check_geometry', 
            self.check_geometry_callback
        )

        # Internal state for the service to report
        self.latest_inlier_count = 0
        self.is_consistent = False

        self.get_logger().info("Geometric Consistency Node Started")

    def match_callback(self, msg):
        # We need at least 15 points for RANSAC to be stable (mathematical min is 8)
        if len(msg.matches) < 15:
            self.get_logger().warn("Not enough matches to check geometric consistency.")
            self.latest_inlier_count = 0
            self.is_consistent = False
            return

        # Extract points from the FeatureMatchList into NumPy arrays
        # Extract points and FORCE float32 type for OpenCV
        pts1 = np.array([[m.x_old, m.y_old] for m in msg.matches], dtype=np.float32)
        pts2 = np.array([[m.x_new, m.y_new] for m in msg.matches], dtype=np.float32)

        # Get the threshold parameter
        inlier_thresh = self.get_parameter('inlier_threshold').value

        # Wrap the OpenCV call in a try-except to catch internal C++ assertion errors
        try:
            F, mask = cv2.findFundamentalMat(pts1, pts2, cv2.FM_RANSAC, inlier_thresh)
        except cv2.error as e:
            self.get_logger().warn(f"OpenCV Error calculating Fundamental Matrix (Skipping Frame). Details: {str(e)}")
            self.latest_inlier_count = 0
            self.is_consistent = False
            return

        if F is None or mask is None:
            self.get_logger().warn("Failed to find Fundamental Matrix.")
            self.latest_inlier_count = 0
            self.is_consistent = False
            return

        # Flatten the mask to create a boolean array
        inliers_mask = mask.ravel() == 1

        # Filter the original points to keep only the geometric inliers
        inliers1 = pts1[inliers_mask]
        inliers2 = pts2[inliers_mask]

        self.latest_inlier_count = len(inliers1)
        
        # Consider the geometry consistent if we have a reasonable amount of inliers
        self.is_consistent = self.latest_inlier_count >= 8

        # Create and publish the GeometricInliers message for Student 2
        inlier_msg = GeometricInliers()
        
        # Convert numpy arrays to flat Python lists of floats
        inlier_msg.previous_x = [float(p[0]) for p in inliers1]
        inlier_msg.previous_y = [float(p[1]) for p in inliers1]
        inlier_msg.current_x = [float(p[0]) for p in inliers2]
        inlier_msg.current_y = [float(p[1]) for p in inliers2]

        self.publisher.publish(inlier_msg)
        
        self.get_logger().info(
            f"Geometry Checked: {len(msg.matches)} matches -> {self.latest_inlier_count} inliers"
        )

    def check_geometry_callback(self, request, response):
        # When a node calls this service, return the current state
        response.is_consistent = self.is_consistent
        response.inlier_count = self.latest_inlier_count
        
        self.get_logger().info(f"Service Call: Responded with {self.latest_inlier_count} inliers.")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = GeometricConsistency()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()