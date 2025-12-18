#!/usr/bin/env python3
"""
Demo pick & place:
- Naviga tra pilastri e box usando nav2_simple_commander (come follow_waypoints.py)
- Quando il scan rileva un ostacolo vicino (pilastro), si ferma e comanda il braccio
- Dopo il pick, naviga verso la box e simula il rilascio
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import yaml
import time

waypoints = [
    {
        'position': {'x': 2.0, 'y': 1.5, 'z': 0.0},
        'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
    },
    {
        'position': {'x': 2.0, 'y': -1.5, 'z': 0.0},
        'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
    }
    # Per ora 2 pilastri
]
LIDAR_STOP_DIST = 0.03  # metri

class PickPlaceDemo(Node):
    def __init__(self):
        super().__init__('pick_place_demo')
        self.navigator = BasicNavigator()
        self.arm_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.state = 'idle'
        self.current_wp_idx = 0
        self.poses = [self.create_pose(wp) for wp in waypoints]
        self.box_pose = self.create_pose({'position': {'x': -2.0, 'y': 0.0, 'z': 0.0}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}})
        self.get_logger().info('PickPlaceDemo node ready')

    def create_pose(self, wp):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = wp['position']['x']
        pose.pose.position.y = wp['position']['y']
        pose.pose.position.z = wp['position'].get('z', 0.0)
        pose.pose.orientation.x = wp['orientation']['x']
        pose.pose.orientation.y = wp['orientation']['y']
        pose.pose.orientation.z = wp['orientation']['z']
        pose.pose.orientation.w = wp['orientation']['w']
        return pose

    def lidar_callback(self, msg):
        # Aggiorna la distanza minima letta dal scan
        self.lidar_min_dist = min(msg.ranges)

    def run(self):
        rclpy.spin_once(self, timeout_sec=1.0)
        self.navigator.waitUntilNav2Active()
        while rclpy.ok() and self.current_wp_idx < len(self.poses):
            self.state = 'navigating'
            goal = self.poses[self.current_wp_idx]
            self.get_logger().info(f'Navigazione verso pilastro {self.current_wp_idx+1}')
            self.navigator.goToPose(goal)
            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.1)
                # Se il scan vede un ostacolo troppo vicino, ferma la navigazione
                if self.lidar_min_dist < LIDAR_STOP_DIST:
                    self.get_logger().warn('Pilastro rilevato vicino! Stop e pick')
                    self.navigator.cancelTask()
                    self.state = 'pick'
                    break
            if self.state == 'pick':
                self.pick_sequence()
                self.state = 'to_box'
                # Naviga verso la box
                self.get_logger().info('Navigazione verso la box')
                self.navigator.goToPose(self.box_pose)
                while not self.navigator.isTaskComplete():
                    rclpy.spin_once(self, timeout_sec=0.1)
                self.place_sequence()
            self.current_wp_idx += 1
        self.get_logger().info('Pick & Place completato!')

    def pick_sequence(self):
        self.get_logger().info('Simulo comando braccio per PICK')
        msg = Float64MultiArray()
        msg.data = [1.0, 0.5, -1.0, -0.5, 0.0]  # Placeholder
        self.arm_pub.publish(msg)
        time.sleep(2)

    def place_sequence(self):
        self.get_logger().info('Simulo comando braccio per PLACE')
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 0.0, 0.0, 0.06]  # Placeholder
        self.arm_pub.publish(msg)
        time.sleep(2)

def main():
    rclpy.init()
    node = PickPlaceDemo()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
