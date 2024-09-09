import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class QuadcopterController(Node):
    def __init__(self):
        super().__init__('quadcopter_controller')

        # Publisher for the velocity commands
        self.publisher_ = self.create_publisher(Twist, '/quadcopter_with_camera/cmd_vel', 10)

        # Timer to control the rate of sending velocity commands
        timer_period = 1.0  # seconds, slower to visualize the movement steps
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize state variables
        self.state = 0
        self.get_logger().info('Quadcopter Controller has been started')

    def timer_callback(self):
        # Create a Twist message
        msg = Twist()

        # State machine to determine the motion sequence
        if self.state == 0:  # Move up
            msg.linear.z = 1.0  # Move up at 1 m/s
            self.get_logger().info('Moving up')
        elif self.state == 1:  # Move down
            msg.linear.z = -1.0  # Move down at 1 m/s
            self.get_logger().info('Moving down')
        elif self.state == 2:  # Move forward
            msg.linear.x = 1.0  # Move forward at 1 m/s
            self.get_logger().info('Moving forward')
        else:
            # Stop the quadcopter
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            self.get_logger().info('Stopping quadcopter')
        
        # Publish the velocity command
        self.publisher_.publish(msg)

        # Increment state to proceed to the next motion
        self.state += 1
        if self.state > 3:  # After moving up, down, and forward, stop
            self.state = 0  # Reset state to repeat the sequence

def main(args=None):
    # Initialize the ROS client library
    rclpy.init(args=args)

    # Create an instance of the QuadcopterController
    quadcopter_controller = QuadcopterController()

    # Spin the node to keep it active
    try:
        rclpy.spin(quadcopter_controller)
    except KeyboardInterrupt:
        quadcopter_controller.get_logger().info('Node stopped cleanly')
    except Exception as e:
        quadcopter_controller.get_logger().error(f'Exception in node: {e}')
    finally:
        # Cleanup and shutdown
        quadcopter_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
