import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String  # Change according to your message type
import time
from teleop.crc import CRC

import numpy as np
from unitree_hg.msg import (
    LowState,
    MotorState,
    IMUState,
    LowCmd,
    MotorCmd,
)

HZ = 1000
crc = CRC()

dt = (1 / HZ) - 8e-6


class MessageRelayNode(Node):
    def __init__(self):
        super().__init__("message_relay_node")

        # Create QoS profile matching robot's expectation (BEST_EFFORT)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create a subscriber to listen to an input topic (such as 'input_topic')
        self.subscription = self.create_subscription(
            LowCmd,  # Replace with your message type
            "lowcmd_buffer",  # Replace with your input topic name
            self.listener_callback,
            1,
        )

        # Create a publisher to publish messages to the 'lowcmd' topic
        # IMPORTANT: Robot expects BEST_EFFORT, not RELIABLE!
        self.publisher = self.create_publisher(LowCmd, "lowcmd", qos_profile)

        # Initialize a variable to store received messages
        self.last_msg = None
        self.last_last_msg = None
        self.counter = 0

        # init motor command
        self.new_msg = LowCmd()
        self.new_msg.mode_pr = 0
        self.new_msg.mode_machine = 5
        self.motor_cmd = []
        for id in range(29):
            cmd = MotorCmd(q=0.0, dq=0.0, tau=0.0, kp=0.0, kd=0.0, mode=1, reserve=0)
            self.motor_cmd.append(cmd)
        for id in range(29, 35):
            cmd = MotorCmd(q=0.0, dq=0.0, tau=0.0, kp=0.0, kd=0.0, mode=0, reserve=0)
            self.motor_cmd.append(cmd)
        self.new_msg.motor_cmd = self.motor_cmd.copy()

    def listener_callback(self, msg):
        # Save the message whenever one is received
        if self.last_msg is None:
            print("[Lowcmd Publisher] First message received from g1_server!")
            self.counter = 0
            self.last_msg = LowCmd()
            self.last_last_msg = LowCmd()
            self.last_msg.motor_cmd = msg.motor_cmd.copy()
            self.last_last_msg.motor_cmd = msg.motor_cmd.copy()
        else:
            self.counter = 0
            self.last_last_msg.motor_cmd = self.last_msg.motor_cmd.copy()
            self.last_msg.motor_cmd = msg.motor_cmd.copy()

    def set_motor_position(
        self,
    ):
        for i in range(29):
            count = np.clip(self.counter, 0, 15)
            # self.motor_cmd[i].q = self.last_msg.motor_cmd[i].q * (count/15) + self.last_last_msg.motor_cmd[i].q * (1-count/15)
            self.motor_cmd[i].q = self.last_msg.motor_cmd[i].q
            self.motor_cmd[i].kp = self.last_msg.motor_cmd[i].kp
            self.motor_cmd[i].kd = self.last_msg.motor_cmd[i].kd
        self.new_msg.motor_cmd = self.motor_cmd.copy()
        # self.cmd_msg.crc = get_crc(self.cmd_msg)
        self.new_msg.crc = crc.Crc(self.new_msg)
        self.counter += 1

    def relay_message(self):
        # If a message has been received, publish a message to the lowcmd topic each time this function is called
        if self.last_msg is not None:
            self.set_motor_position()
            self.publisher.publish(self.new_msg)
            # breakpoint()

            # print("########################################")
            # for i in range(29):
            #     print(self.new_msg.motor_cmd[i].q)
            # print(self.new_msg.motor_cmd[18].q)


def main(args=None):
    rclpy.init(args=args)

    # Create node instance
    node = MessageRelayNode()

    print("Initialized")
    print("[Lowcmd Publisher] Waiting for commands from g1_server on topic 'lowcmd_buffer'...")

    # Add logging variables
    msg_count = 0
    last_log_time = time.monotonic()

    try:
        loop_start_time = time.monotonic()
        while rclpy.ok():
            # Process subscription callback functions
            rclpy.spin_once(node, timeout_sec=0)

            # Forward message to lowcmd topic
            node.relay_message()

            # Log message rate every 2 seconds
            if node.last_msg is not None:
                msg_count += 1
                current_time = time.monotonic()
                if current_time - last_log_time >= 2.0:
                    msg_rate = msg_count / 2.0
                    print(f"[Lowcmd Publisher] Receiving & publishing commands at {msg_rate:.1f} Hz")
                    # Print sample joint position
                    if len(node.new_msg.motor_cmd) > 0:
                        print(f"[Lowcmd Publisher] Sample joint 0 position: {node.new_msg.motor_cmd[0].q:.4f}")
                    msg_count = 0
                    last_log_time = current_time

            while (
                dt - time.monotonic() + loop_start_time > 0
            ):  # 0.012473  0.019963 # Create 1000Hz publishing frequency
                pass
            loop_start_time = time.monotonic()

    except KeyboardInterrupt:
        pass

    # Close node
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
