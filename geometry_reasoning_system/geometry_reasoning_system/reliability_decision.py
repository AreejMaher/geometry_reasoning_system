import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32

from rclpy.action import ActionServer
from geometry_reasoning_interface.action import Report


class ReliabilityNode(Node):

    def __init__(self):
        super().__init__('reliability_decision')

        # Parameter
        self.declare_parameter('min_inliers', 15)
        self.min_inliers = self.get_parameter('min_inliers').value

        # Subscribers
        self.create_subscription(Int32, '/geometric_inliers', self.inliers_callback, 10)
        self.create_subscription(String, '/camera_motion', self.motion_callback, 10)

        # Publisher
        self.publisher = self.create_publisher(String, '/system_state', 10)

        # Action Server
        self._action_server = ActionServer(
            self,
            Report,
            '/report_action',
            self.execute_callback
        )

        # Variables
        self.inliers = 0
        self.motion = ""
        self.state = "UNRELIABLE"

    def inliers_callback(self, msg):
        self.inliers = msg.data
        self.evaluate()

    def motion_callback(self, msg):
        self.motion = msg.data
        self.evaluate()

    def evaluate(self):
        # 1. LOW FEATURES
        if self.inliers < self.min_inliers:
            self.state = "LOW_FEATURES"

        # 2. UNRELIABLE (few matches or weak consistency)
        elif self.inliers < (self.min_inliers * 2):
            self.state = "UNRELIABLE"

        # 3. RELIABLE
        else:
            self.state = "RELIABLE"

        # Publish result
        msg = String()
        msg.data = self.state
        self.publisher.publish(msg)

        self.get_logger().info(f"System State: {self.state}")

    def execute_callback(self, goal_handle):
        self.get_logger().info("Report Action Requested")

        # Feedback (اختياري)
        feedback_msg = Report.Feedback()
        feedback_msg.feedback = "Processing..."
        goal_handle.publish_feedback(feedback_msg)

        # Result
        result = Report.Result()
        result.state = self.state

        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ReliabilityNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()