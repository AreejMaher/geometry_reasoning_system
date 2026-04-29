import rclpy  # ros2 Python client library (ros1: rospy)
from rclpy.node import Node  # base class for ros2 nodes (ros1: no class required)
from geometry_reasoning_interface.msg import GeometricInliers  
from geometry_reasoning_interface.msg import CameraMotion  
# custom message to publish estimated motion
# ros1: same concept

class MotionEstimationNode(Node):  # define ros2 node class
    def __init__(self):
        super().__init__('motion_estimation_node')
        # initialize node
        # ros1: rospy.init_node("motion_estimation_node")

        # Parameters
        self.declare_parameter('focal_length', 500.0)
        # camera focal length in pixels
        # ros1: rospy.get_param("~focal_length", 500.0)

        self.focal_length = self.get_parameter('focal_length').value # read parameter value

        # Subscriber
        self.subscription = self.create_subscription(
            GeometricInliers,              # message type
            '/geometric_inliers',          # topic name
            self.motion_callback,          # callback function
            10                             # queue size
        )
        # ros1: rospy.Subscriber(...)

        # Publisher
        self.publisher = self.create_publisher(
            CameraMotion,                  # message type
            '/camera_motion',              # topic name
            10                             # queue size
        )
        # ros1: rospy.Publisher(...)

        self.get_logger().info("Motion Estimation Node Started")
        # ros1: rospy.loginfo(...)

    def motion_callback(self, msg):
        # called whenever data arrives from /geometric_inliers

        direction, magnitude = self.calculate_relative_movement(msg)# estimate motion from received inlier points
        motion_msg = CameraMotion() # create output message       
        motion_msg.direction = direction # example: LEFT / RIGHT / FORWARD
        motion_msg.magnitude = magnitude # estimated motion strength
        motion_msg.scale_ambiguity = True # monocular camera cannot know exact scale
        self.publisher.publish(motion_msg) # publish result
        self.get_logger().info(f"Motion: {direction}, Magnitude: {magnitude:.2f}") # print output for debugging

    def calculate_relative_movement(self, msg):
        # simple motion estimation logic
        # compares average movement of feature points

        if len(msg.current_x) == 0: # if no points received
            return "NONE", 0.0

        total_shift = 0.0
        point_count = len(msg.current_x)

        for i in range(point_count):  # calculate horizontal movement
            shift = msg.current_x[i] - msg.previous_x[i]
            total_shift += shift

        average_shift = total_shift / point_count # average pixel movement
        magnitude = abs(average_shift) / self.focal_length # normalize using focal length
      
        if average_shift > 5:
            direction = "LEFT" # features moved right → camera moved left
        elif average_shift < -5:
            direction = "RIGHT" # features moved left → camera moved right
        else:
            direction = "FORWARD" # small horizontal change
        return direction, magnitude

def main(args=None):
    rclpy.init(args=args)
    # initialize ros2
    # ros1: rospy.init_node()
    node = MotionEstimationNode() # create node object
    try:
        rclpy.spin(node)
        # keep node alive
        # ros1: rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # destroy node
        # ros2 only
        rclpy.shutdown()
        # shutdown ros2
        # ros1 handles automatically

if __name__ == '__main__':
    main()

# command to run node
# ros2 run my_py_pkg motion_estimation_node