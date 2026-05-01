import rclpy
from rclpy.node import Node
from geometry_reasoning_interface.msg import FeatureMatch, FeatureMatchList, Descriptor, DescriptorList
import cv2
import numpy as np
import time

class FeatureMatching(Node):
    def __init__(self):
        super().__init__("feature_matching")

        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.prev_descriptors = None

        self.create_subscription(DescriptorList,'/descriptors', self.matching_callback, 10)
        self.matching_pub = self.create_publisher(FeatureMatchList, '/raw_matches', 10)
        self.declare_parameter('match_threshold', 30)

        self.get_logger().info(f"Feature Matching Node has started")

    def matching_callback(self, msg):
        if self.prev_descriptors is None:
            self.prev_descriptors = msg.descriptors
            return
    
        threshold = self.get_parameter('match_threshold').value
               
        current_arr_descriptors = np.array([d.descriptor for d in msg.descriptors], dtype=np.uint8)
        prev_arr_descriptors = np.array([d.descriptor for d in self.prev_descriptors], dtype=np.uint8)

        matches = self.bf.knnMatch(prev_arr_descriptors, current_arr_descriptors, k=2)
        # matches = self.bf.match(prev_arr_descriptors, current_arr_descriptors)

        match_list = FeatureMatchList()
        match_list.header = msg.header

        for pair in matches:
            if len(pair) < 2:
                continue

            m, n = pair
            if m.distance <= threshold:
                for match in [m, n]:
                    # queryIdx gives keypoint index from target image
                    query_idx = match.queryIdx
                    # trainIdx gives keypoint index from current frame 
                    train_idx = match.trainIdx
                    match_score = match.distance

                    # Map matched keypoint indices back to their pixel coordinates from the previous and current frame
                    pt1_x = self.prev_descriptors[query_idx].x
                    pt1_y = self.prev_descriptors[query_idx].y

                    pt2_x = msg.descriptors[train_idx].x
                    pt2_y = msg.descriptors[train_idx].y

                    feature_match = FeatureMatch()

                    feature_match.x_old = pt1_x
                    feature_match.y_old = pt1_y
                    feature_match.x_new = pt2_x
                    feature_match.y_new = pt2_y
                    feature_match.distance = match_score

                    match_list.matches.append(feature_match)

        self.get_logger().info(f"Frame {msg.header.frame_id}: Found {len(matches)} potential pairs.")
        self.matching_pub.publish(match_list)
        self.prev_descriptors = msg.descriptors

def main(args=None):
    try:
        rclpy.init(args=args)
        node = FeatureMatching()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()