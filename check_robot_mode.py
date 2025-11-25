#!/usr/bin/env python3
"""
Simple script to check robot mode and provide instructions
"""

import rclpy
from rclpy.node import Node
from unitree_hg.msg import LowState
import time

class CheckModeNode(Node):
    def __init__(self):
        super().__init__('check_mode_node')

        # Subscriber to low state
        self.state_sub = self.create_subscription(
            LowState,
            '/lowstate',
            self.state_callback,
            1
        )

        self.mode_machine = None
        self.mode_pr = None

    def state_callback(self, msg):
        """Check current robot mode from lowstate"""
        self.mode_machine = msg.mode_machine
        self.mode_pr = msg.mode_pr

        print(f"\rRobot State - mode_pr: {self.mode_pr}, mode_machine: {self.mode_machine} ", end="")

        if self.mode_machine == 5:
            print("✅ LOW-LEVEL MODE ACTIVE!", end="")
        elif self.mode_machine == 6:
            print("⚠️  Mode 6 - Need to switch to mode 5!", end="")
        else:
            print(f"❓ Unknown mode {self.mode_machine}", end="")

def main():
    print("=" * 60)
    print("G1 ROBOT MODE CHECKER")
    print("=" * 60)
    print()

    rclpy.init()
    node = CheckModeNode()

    print("Monitoring robot mode (Press Ctrl+C to stop)...")
    print()
    print("Current Status:")

    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

    print("\n\n" + "=" * 60)
    print("MODE SWITCHING INSTRUCTIONS:")
    print("=" * 60)
    print()
    print("The robot is currently in mode:", node.mode_machine)
    print()
    print("To enable LOW-LEVEL CONTROL (mode 5), try:")
    print()
    print("1. USING WIRELESS REMOTE:")
    print("   - Press and hold L2 + R2 simultaneously")
    print("   - Then press START button")
    print("   - Release all buttons")
    print()
    print("2. EMERGENCY STOP RESET:")
    print("   - Press emergency stop button on robot")
    print("   - Release emergency stop")
    print("   - Press L1 on remote to re-enable")
    print()
    print("3. POWER CYCLE:")
    print("   - Turn off robot completely")
    print("   - Wait 10 seconds")
    print("   - Turn on robot")
    print("   - Wait for initialization")
    print("   - Press L2+R2+START on remote")
    print()
    print("4. CHECK ROBOT LED INDICATORS:")
    print("   - GREEN = Normal operation mode")
    print("   - BLUE = Low-level control mode")
    print("   - RED = Emergency/Error state")
    print()
    print("If mode is 5, the robot should respond to commands!")
    print("=" * 60)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()