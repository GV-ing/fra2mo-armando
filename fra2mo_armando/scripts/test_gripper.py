#!/usr/bin/env python3
"""
Test script for gripper control.
Tests opening and closing the gripper using position_controller.
The gripper is now part of the position_controller (joint index 4).
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import time


class GripperTester(Node):
    def __init__(self):
        super().__init__('gripper_tester')
        
        # Publisher for position_controller (controls all 5 joints: arm_j0-j3 + gripper)
        self._position_pub = self.create_publisher(
            Float64MultiArray,
            '/position_controller/commands',
            10
        )
        
        # Subscribe to joint states to get current positions
        self._joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Current arm position (we keep arm still while moving gripper)
        self.arm_positions = [0.0, 0.0, 0.0, 0.0]  # j0, j1, j2, j3
        self.current_gripper_pos = 0.03  # Initial gripper position
        self.joint_states_received = False
        
        # Wait for publisher to be ready and receive joint states
        self.get_logger().info('Waiting for joint states...')
        timeout = 10.0
        start_time = time.time()
        while not self.joint_states_received and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.joint_states_received:
            self.get_logger().info('Position controller publisher ready!')
            self.get_logger().info(f'Current arm positions: {self.arm_positions}')
            self.get_logger().info(f'Current gripper position: {self.current_gripper_pos:.3f}m')
        else:
            self.get_logger().warn('Timeout waiting for joint states, using defaults')

    def joint_state_callback(self, msg):
        """Read current joint positions from /joint_states."""
        try:
            # Find indices for our joints
            arm_joints = ['arm_j0', 'arm_j1', 'arm_j2', 'arm_j3']
            for i, joint_name in enumerate(arm_joints):
                if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    self.arm_positions[i] = msg.position[idx]
            
            # Get gripper position
            if 'arm_gripper_right_joint' in msg.name:
                idx = msg.name.index('arm_gripper_right_joint')
                self.current_gripper_pos = msg.position[idx]
            
            self.joint_states_received = True
        except Exception as e:
            self.get_logger().error(f'Error reading joint states: {e}')

    def send_gripper_command(self, gripper_position):
        """
        Send gripper position command while keeping arm still.
        
        Args:
            gripper_position: Target gripper position (0.0 = closed, 0.06 = fully open)
        """
        msg = Float64MultiArray()
        # Command format: [j0, j1, j2, j3, gripper]
        msg.data = self.arm_positions + [float(gripper_position)]
        
        self.get_logger().info(f'Sending command: {msg.data}')
        self.get_logger().info(f'  → Gripper: {gripper_position:.3f}m')
        self._position_pub.publish(msg)

    def test_sequence(self):
        """Run test sequence: close -> open -> half -> close."""
        self.get_logger().info('='*50)
        self.get_logger().info('GRIPPER TEST SEQUENCE')
        self.get_logger().info('Position controller controls 5 joints:')
        self.get_logger().info('  [j0, j1, j2, j3, gripper]')
        self.get_logger().info('Gripper starts at 0.03m (open)')
        self.get_logger().info('='*50)
        
        # Wait to see initial position
        self.get_logger().info('\n[INITIAL] Gripper at starting position (0.03m - open)...')
        time.sleep(2)
        
        # Test 1: Close gripper
        self.get_logger().info('\n[TEST 1] Closing gripper completely...')
        self.send_gripper_command(gripper_position=0.0)
        time.sleep(3)
        
        # Test 2: Open gripper fully
        self.get_logger().info('\n[TEST 2] Opening gripper fully (max 0.06m)...')
        self.send_gripper_command(gripper_position=0.06)
        time.sleep(3)
        
        # Test 3: Half open
        self.get_logger().info('\n[TEST 3] Half open position...')
        self.send_gripper_command(gripper_position=0.02)
        time.sleep(3)
        
        # Test 4: Back to initial open position
        self.get_logger().info('\n[TEST 4] Back to initial open position...')
        self.send_gripper_command(gripper_position=0.03)
        time.sleep(2)
        
        # Test 5: Close again
        self.get_logger().info('\n[TEST 5] Final close...')
        self.send_gripper_command(gripper_position=0.0)
        time.sleep(2)
        
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('GRIPPER TEST COMPLETED!')
        self.get_logger().info('Watch Gazebo to verify gripper movement!')
        self.get_logger().info('='*50)


def main(args=None):
    rclpy.init(args=args)
    
    gripper_tester = GripperTester()
    
    try:
        gripper_tester.test_sequence()
    except KeyboardInterrupt:
        gripper_tester.get_logger().info('Test interrupted by user')
    finally:
        gripper_tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
