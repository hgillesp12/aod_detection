import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class QuadcopterController(Node):
    def __init__(self):
        super().__init__('quadcopter_controller')

        # Publisher for the velocity commands
        self.publisher_ = self.create_publisher(Twist, '/quadcopter_with_camera/cmd_vel', 10)

        # Timer to control the rate of sending velocity commands
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize desired velocities
        self.linear_velocity = 0.5   # Move forward at 0.5 m/s
        self.angular_velocity = 0.1  # Rotate at 0.1 rad/s
        self.get_logger().info('Quadcopter Controller has been started')

    def timer_callback(self):
        # Create a Twist message with desired velocities
        msg = Twist()
        msg.linear.x = self.linear_velocity  # Forward/backward velocity
        msg.linear.y = 0.0                   # Left/right velocity
        msg.linear.z = 0.0                   # Up/down velocity
        msg.angular.x = 0.0                  # Roll
        msg.angular.y = 0.0                  # Pitch
        msg.angular.z = self.angular_velocity  # Yaw (turn rate)

        # Publish the velocity command
        self.publisher_.publish(msg)
        #self.get_logger().info(f'Publishing velocity command: linear_x={msg.linear.x}, angular_z={msg.angular.z}')

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