import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Header
from rcl_interfaces.msg import ParameterDescriptor


class TwistToTwistStamped(Node):

    def __init__(self):
        super().__init__('twist_to_twist_stamped')
        self.twist_subscriber = self.create_subscription(Twist, '/cmd_vel_in', self.twist_callback, 10)
        self.twist_publisher = self.create_publisher(TwistStamped, '/cmd_vel_out', 10)
        parameter_descriptor = ParameterDescriptor(description='frame_id to geometry_msgs/TwistStamped message')
        self.declare_parameter('frame_id', 'base_link', parameter_descriptor)

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

    def twist_callback(self, twist):
        # Create the header with the current time and frame id
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        # Create the TwistStamped message
        twist_stamped = TwistStamped()
        twist_stamped.header = header
        twist_stamped.twist = twist

        # Publish the TwistStamped message
        self.twist_publisher.publish(twist_stamped)

def main(args=None):
    rclpy.init(args=args)
    node = TwistToTwistStamped()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
