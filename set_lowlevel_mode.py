#!/usr/bin/env python3
"""
Script to set G1 robot to low-level control mode (mode 5)
"""

import rclpy
from rclpy.node import Node
from unitree_go.msg import SportModeState
from unitree_api.msg import Request
import json
import time

class SetModeNode(Node):
    def __init__(self):
        super().__init__('set_mode_node')

        # Publisher for sport API
        self.sport_pub = self.create_publisher(Request, '/api/sport/request', 1)

        # Subscriber to check current state
        self.state_sub = self.create_subscription(
            SportModeState,
            '/sportmodestate',
            self.state_callback,
            1
        )

        self.current_mode = None

    def state_callback(self, msg):
        """Check current robot mode"""
        if hasattr(msg, 'mode'):
            self.current_mode = msg.mode
            self.get_logger().info(f'Current mode: {self.current_mode}')

    def set_low_level_mode(self):
        """Try to set robot to low-level mode"""

        # Method 1: Try using sport API to switch to advanced mode
        request = Request()
        request.header.identity.id = 1001
        request.header.identity.api_id = 1002

        # Try different commands that might enable low-level mode
        commands = [
            {"id": 1002, "mode": 5},  # Direct mode change
            {"id": 1001, "cmd": "mode", "data": {"mode_machine": 5}},  # Alternative format
            {"id": 2001, "cmd": "advanced_mode", "enable": True},  # Enable advanced mode
        ]

        for i, cmd in enumerate(commands):
            self.get_logger().info(f'Trying command {i+1}: {cmd}')
            request.parameter = json.dumps(cmd)
            self.sport_pub.publish(request)
            time.sleep(1.0)

            # Check if mode changed
            rclpy.spin_once(self, timeout_sec=0.5)

        self.get_logger().info('Commands sent. Check robot state.')

def main():
    print("=" * 50)
    print("G1 Robot Mode Setter")
    print("=" * 50)
    print()
    print("This script attempts to set the robot to low-level control mode.")
    print("Current robot mode should be 5 for low-level control.")
    print()

    rclpy.init()
    node = SetModeNode()

    # Check current state first
    print("Checking current mode...")
    for _ in range(3):
        rclpy.spin_once(node, timeout_sec=1.0)

    # Try to set mode
    print("\nAttempting to set low-level mode...")
    node.set_low_level_mode()

    # Check state again
    print("\nChecking mode after commands...")
    for _ in range(3):
        rclpy.spin_once(node, timeout_sec=1.0)

    node.destroy_node()
    rclpy.shutdown()

    print("\n" + "=" * 50)
    print("IMPORTANT:")
    print("If mode didn't change to 5, you may need to:")
    print("1. Use the physical remote control to enable low-level mode")
    print("2. Check if robot is in emergency stop state")
    print("3. Power cycle the robot and try again")
    print("4. Consult G1 manual for mode switching procedure")
    print("=" * 50)

if __name__ == '__main__':
    main()