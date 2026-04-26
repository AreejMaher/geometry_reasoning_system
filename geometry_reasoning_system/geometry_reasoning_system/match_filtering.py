import rclpy
from rclpy.node import Node
from geometry_reasoning_interface.msg import FeatureMatch, FeatureMatchList

class MatchFiltering(Node):
    def __init__(self):
        super().__init__("match_filtering")

        self.prev_descriptors = None

        self.create_subscription(FeatureMatchList,'/raw_matches', self.filtering_callback, 10)
        self.matching_pub = self.create_publisher(FeatureMatchList, '/filtered_matches', 10)
        self.declare_parameter('ratio_test_threshold', 75)

        self.get_logger().info(f"Match Filtering Node has started")

    def filtering_callback(self, msg):
        ratio_thresh = self.get_parameter('ratio_test_threshold').value / 100.0
        
        filtered_msg = FeatureMatchList()
        filtered_msg.header = msg.header
        
        for i in range(0, len(msg.matches) - 1, 2):
            m = msg.matches[i]
            n = msg.matches[i+1]
            
            # Apply the Ratio Test
            if m.distance < ratio_thresh * n.distance:
                filtered_msg.matches.append(m)

        self.matching_pub.publish(filtered_msg)
        self.get_logger().info(f"Ratio Test: {len(filtered_msg.matches)} physical anchors preserved.")

def main(args=None):
    try:
        rclpy.init(args=args)
        node = MatchFiltering()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()