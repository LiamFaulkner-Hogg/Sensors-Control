#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class BasicVisualController(Node):
    def __init__(self):
        super().__init__('basic_visual_controller')
        
        # Safety parameters
        self.max_velocity = 0.1  # Max 0.1 m/s
        self.gain = 0.0005      # Reduced gain
        self.dead_zone = 10     # Ignore small errors
        
        # QR code target parameter
        self.declare_parameter('target_name', 'TARGET1')
        self.target_name = self.get_parameter('target_name').value
        
        # Subscribe to QR positions
        self.create_subscription(
            String,
            '/qr_code_positions',
            self.control_callback,
            10)
        
        # Publisher for UR3
        self.vel_publisher = self.create_publisher(
            Twist,
            '/ur3/scaled_joint_trajectory_controller/velocity_cmd',  
            10)
        
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
                
                # Calculate velocities with safety limits
                vel_x = -self.gain * x_error
                vel_y = -self.gain * y_error
                
                # Apply velocity limits
                vel_x = max(min(vel_x, self.max_velocity), -self.max_velocity)
                vel_y = max(min(vel_y, self.max_velocity), -self.max_velocity)
                
                # Create command
                cmd = Twist()
                cmd.linear.x = vel_x
                cmd.linear.y = vel_y
                cmd.linear.z = 0.0
                cmd.angular.x = 0.0
                cmd.angular.y = 0.0
                cmd.angular.z = 0.0
                
                # Log tracking info
                self.get_logger().info(
                    f'\nTracking {self.target_name}:'
                    f'\n  Errors (pixels) - X: {x_error:.2f}, Y: {y_error:.2f}'
                    f'\n  Safe Velocities - X: {vel_x:.4f}, Y: {vel_y:.4f}'
                )
                
                self.vel_publisher.publish(cmd)
            else:
                self.get_logger().info(f"Waiting for QR code: {self.target_name}")
                
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

def main():
    rclpy.init()
    controller = BasicVisualController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()