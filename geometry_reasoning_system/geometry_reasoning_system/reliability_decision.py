import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_reasoning_interface.msg import GeometricInliers, CameraMotion
from geometry_reasoning_interface.action import Report
from rclpy.action import ActionServer


class ReliabilityNode(Node):

    def __init__(self):
        super().__init__('reliability_decision')

        self.get_logger().info('Reliability Decision Node Started')

        # Parameters
        self.declare_parameter('min_inliers', 15)
        self.min_inliers = self.get_parameter('min_inliers') \
            .get_parameter_value().integer_value

        # Subscribers
        self.create_subscription(
            GeometricInliers,
            '/geometric_inliers',
            self.inliers_callback,
            10
        )

        self.create_subscription(
            CameraMotion,
            '/camera_motion',
            self.motion_callback,
            10
        )

        # Publisher
        self.state_pub = self.create_publisher(String, '/system_state', 10)

        # Action Server
        self._action_server = ActionServer(
            self,
            Report,
            '/report_action',
            self.execute_callback
        )

        # Variables
        self.inliers_count = 0
        self.motion_type = "NONE"
        self.state = "UNRELIABLE"

    def inliers_callback(self, msg):
        self.inliers_count = len(msg.current_x)
        self.get_logger().info(f'Inliers Received: {self.inliers_count}')
        self.update_state()

    def motion_callback(self, msg):
        if hasattr(msg, 'motion_direction'):
            self.motion_type = msg.motion_direction
        elif hasattr(msg, 'motion'):
            self.motion_type = msg.motion
        elif hasattr(msg, 'direction'):
            self.motion_type = msg.direction
        else:
            self.motion_type = "NONE"
            self.get_logger().warn(
                "Field 'motion' not found in CameraMotion message."
            )

        self.update_state()

    def update_state(self):

        # Decide state
        if self.inliers_count < self.min_inliers:
            self.state = "LOW_FEATURES"

        elif self.inliers_count >= self.min_inliers and self.motion_type != "NONE":
            self.state = "RELIABLE"

        else:
            self.state = "UNRELIABLE"

        # Publish state
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)

        # Log
        self.get_logger().info(
            f'Status: {self.state} | Inliers: {self.inliers_count} | Motion: {self.motion_type}'
        )

    async def execute_callback(self, goal_handle):
        self.get_logger().info(
            f'Received report request: {goal_handle.request.request}'
        )

        result = Report.Result()
        result.state = self.state

        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ReliabilityNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user (Ctrl+C)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()