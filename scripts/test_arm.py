#!/usr/bin/env python3
"""
Simple test script for arm control (4 joints only - NO gripper).
Based on reference repository GV-ing/HM1.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time


class ArmTester(Node):
    def __init__(self):
        super().__init__('arm_tester')
        
        # Publisher for position_controller (controls only 4 arm joints)
        self._position_pub = self.create_publisher(
            Float64MultiArray,
            '/position_controller/commands',
            10
        )
        
        time.sleep(1)  # Wait for publisher to be ready
        self.get_logger().info('Arm Test Node ready!')
        self.get_logger().info('='*50)
        
    def test_sequence(self):
        """Run test sequence similar to reference repository."""
        self.get_logger().info('TESTING ARM MOVEMENT (4 joints: j0, j1, j2, j3)')
        self.get_logger().info('='*50)
        
        # Test positions from reference repository
        test_positions = [
            [0.0, 0.0, 0.0, 0.0],      # Home position
            [1.0, 0.5, -1.0, -0.5],    # Test position 1
            [-0.5, -0.3, 0.8, 0.3],    # Test position 2
            [0.0, 0.0, 0.0, 0.0],      # Back to home
        ]
        
        for i, pos in enumerate(test_positions):
            self.get_logger().info(f'\n[TEST {i+1}] Moving to: {pos}')
            msg = Float64MultiArray()
            msg.data = pos
            self._position_pub.publish(msg)
            time.sleep(4)  # Wait for movement
        
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('ARM TEST COMPLETED!')
        self.get_logger().info('Check Gazebo - arm should have moved')
        self.get_logger().info('='*50)


def main(args=None):
    rclpy.init(args=args)
    
    arm_tester = ArmTester()
    
    try:
        arm_tester.test_sequence()
    except KeyboardInterrupt:
        arm_tester.get_logger().info('Test interrupted by user')
    finally:
        arm_tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
