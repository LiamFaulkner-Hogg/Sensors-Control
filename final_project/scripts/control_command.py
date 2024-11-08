#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped

class MoveItVisualController(Node):
    def __init__(self):
        super().__init__('moveit_visual_controller')
        
        # Safety parameters
        self.max_step = 0.01  # Maximum step in meters for target adjustments
        self.gain = 0.0005    # Reduced gain
        self.dead_zone = 10   # Ignore small errors

        # QR code target parameter
        self.declare_parameter('target_name', 'TARGET1')
        self.target_name = self.get_parameter('target_name').value
        
        # Subscribe to QR positions
        self.create_subscription(
            String,
            '/qr_code_positions',
            self.control_callback,
            10)

        # Publisher for twist commands
        self.twist_publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)

        self.get_logger().info(f'Visual Controller Started - Tracking: {self.target_name}')

    def control_callback(self, msg):
        try:
            data = eval(msg.data)
            
            # Log what QR codes are detected
            self.get_logger().info(f"Detected QR codes: {list(data.keys())}")
            
            if self.target_name in data:
                x_error = data[self.target_name]['x']
                y_error = data[self.target_name]['y']
                
                # Apply dead zone
                if abs(x_error) < self.dead_zone: x_error = 0
                if abs(y_error) < self.dead_zone: y_error = 0
                
                # Calculate position adjustments with safety limits
                step_x = max(min(-self.gain * x_error, self.max_step), -self.max_step)
                step_y = max(min(-self.gain * y_error, self.max_step), -self.max_step)

                # Create and publish the twist message
                twist_msg = TwistStamped()
                twist_msg.header.frame_id = "base_link"
                twist_msg.twist.linear.x = step_x
                twist_msg.twist.linear.y = step_y
                twist_msg.twist.linear.z = 0.0  # Keep Z constant
                self.twist_publisher.publish(twist_msg)

                self.get_logger().info(
                    f'\nTracking {self.target_name}:'
                    f'\n  Errors (pixels) - X: {x_error:.2f}, Y: {y_error:.2f}'
                    f'\n  Pose Adjustments - X: {step_x:.4f}, Y: {step_y:.4f}'
                )
            else:
                self.get_logger().info(f"Waiting for QR code: {self.target_name}")
                
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

def main():
    rclpy.init()
    controller = MoveItVisualController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


