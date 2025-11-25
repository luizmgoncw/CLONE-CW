#!/usr/bin/env python3
"""
ROS2 Localization & PointCloud Bridge
Receives localization and point cloud data via ZeroMQ from G1 PC2 and republishes as ROS2 topics.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
import zmq
import pickle
import numpy as np
import struct


class LocalizationBridge(Node):
    def __init__(self):
        super().__init__('localization_bridge')

        # Declare parameters
        self.declare_parameter('server_ip', '192.168.123.164')
        self.declare_parameter('localization_port', 6006)
        self.declare_parameter('cloud_port', 6007)
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('path_length', 500)
        self.declare_parameter('enable_cloud', True)

        # Get parameters
        server_ip = self.get_parameter('server_ip').value
        loc_port = self.get_parameter('localization_port').value
        cloud_port = self.get_parameter('cloud_port').value
        self.path_max_length = self.get_parameter('path_length').value
        self.enable_cloud = self.get_parameter('enable_cloud').value

        # ROS2 Publishers
        self.odom_pub = self.create_publisher(Odometry, '/localization', 10)
        self.path_pub = self.create_publisher(Path, '/localization_path', 10)

        if self.enable_cloud:
            self.cloud_pub = self.create_publisher(PointCloud2, '/cloud_registered', 10)
            self.get_logger().info('PointCloud publishing ENABLED')
        else:
            self.get_logger().info('PointCloud publishing DISABLED')

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Path storage
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'

        # ZeroMQ setup - Localization
        self.get_logger().info(f'Connecting to localization at {server_ip}:{loc_port}...')
        self.loc_context = zmq.Context()
        self.loc_socket = self.loc_context.socket(zmq.SUB)
        self.loc_socket.connect(f"tcp://{server_ip}:{loc_port}")
        self.loc_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.loc_socket.setsockopt(zmq.RCVTIMEO, 1000)  # 1 second timeout

        # ZeroMQ setup - PointCloud (if enabled)
        if self.enable_cloud:
            self.get_logger().info(f'Connecting to cloud at {server_ip}:{cloud_port}...')
            self.cloud_context = zmq.Context()
            self.cloud_socket = self.cloud_context.socket(zmq.SUB)
            self.cloud_socket.connect(f"tcp://{server_ip}:{cloud_port}")
            self.cloud_socket.setsockopt_string(zmq.SUBSCRIBE, "")
            self.cloud_socket.setsockopt(zmq.RCVTIMEO, 100)  # 100ms timeout

        # Statistics
        self.loc_msg_count = 0
        self.cloud_msg_count = 0
        self.last_log_time = self.get_clock().now()

        # Start receiving loops
        self.create_timer(0.02, self.receive_localization)  # 50 Hz
        if self.enable_cloud:
            self.create_timer(0.05, self.receive_cloud)  # 20 Hz

        self.get_logger().info('Localization Bridge started!')
        topics = '/localization, /localization_path'
        if self.enable_cloud:
            topics += ', /cloud_registered'
        self.get_logger().info(f'Publishing to: {topics}')

    def receive_localization(self):
        """Receive localization data from ZeroMQ and publish to ROS2"""
        try:
            # Non-blocking receive
            message = self.loc_socket.recv(flags=zmq.NOBLOCK)
            position, quat = pickle.loads(message)

            # Create timestamp
            current_time = self.get_clock().now().to_msg()

            # Publish Odometry
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time
            odom_msg.header.frame_id = 'map'
            odom_msg.child_frame_id = 'body'

            # Position
            odom_msg.pose.pose.position.x = float(position[0])
            odom_msg.pose.pose.position.y = float(position[1])
            odom_msg.pose.pose.position.z = float(position[2])

            # Orientation (quaternion)
            odom_msg.pose.pose.orientation.x = float(quat[0])
            odom_msg.pose.pose.orientation.y = float(quat[1])
            odom_msg.pose.pose.orientation.z = float(quat[2])
            odom_msg.pose.pose.orientation.w = float(quat[3])

            self.odom_pub.publish(odom_msg)

            # Add to path
            pose_stamped = PoseStamped()
            pose_stamped.header = odom_msg.header
            pose_stamped.pose = odom_msg.pose.pose
            self.path_msg.poses.append(pose_stamped)

            # Limit path length
            if len(self.path_msg.poses) > self.path_max_length:
                self.path_msg.poses = self.path_msg.poses[-self.path_max_length:]

            # Update path header and publish
            self.path_msg.header.stamp = current_time
            self.path_pub.publish(self.path_msg)

            # Broadcast TF
            t = TransformStamped()
            t.header.stamp = current_time
            t.header.frame_id = 'map'
            t.child_frame_id = 'body'
            t.transform.translation.x = float(position[0])
            t.transform.translation.y = float(position[1])
            t.transform.translation.z = float(position[2])
            t.transform.rotation.x = float(quat[0])
            t.transform.rotation.y = float(quat[1])
            t.transform.rotation.z = float(quat[2])
            t.transform.rotation.w = float(quat[3])
            self.tf_broadcaster.sendTransform(t)

            # Statistics
            self.loc_msg_count += 1

        except zmq.Again:
            # No message available
            pass
        except Exception as e:
            self.get_logger().error(f'Error receiving localization: {e}')

    def receive_cloud(self):
        """Receive point cloud data from ZeroMQ and publish to ROS2"""
        if not self.enable_cloud:
            return

        try:
            # Non-blocking receive
            message = self.cloud_socket.recv(flags=zmq.NOBLOCK)
            data = pickle.loads(message)

            header_data = data['header']
            points = data['points']  # Nx4 numpy array (x, y, z, intensity)

            # Validate points array shape
            if points.ndim != 2 or points.shape[1] != 4:
                self.get_logger().error(f'Invalid points shape: {points.shape}, expected (N, 4)')
                return

            num_points = points.shape[0]

            # Create ROS2 PointCloud2 message
            cloud_msg = PointCloud2()

            # Header
            cloud_msg.header.stamp = self.get_clock().now().to_msg()
            cloud_msg.header.frame_id = header_data['frame_id']

            # PointCloud2 fields (x, y, z, intensity as float32)
            cloud_msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            ]

            # PointCloud2 data
            cloud_msg.height = 1
            cloud_msg.width = num_points
            cloud_msg.point_step = 16  # 4 floats * 4 bytes
            cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
            cloud_msg.is_bigendian = False
            cloud_msg.is_dense = True

            # Convert points to bytes
            cloud_data = points.astype(np.float32).tobytes()

            # Validate data size
            expected_size = cloud_msg.height * cloud_msg.row_step
            if len(cloud_data) != expected_size:
                self.get_logger().error(
                    f'Data size mismatch: {len(cloud_data)} bytes, '
                    f'expected {expected_size} bytes '
                    f'(height={cloud_msg.height}, width={cloud_msg.width}, '
                    f'point_step={cloud_msg.point_step}, points_shape={points.shape})'
                )
                return

            cloud_msg.data = cloud_data

            # Publish
            self.cloud_pub.publish(cloud_msg)

            # Statistics
            self.cloud_msg_count += 1

            # Log every 2 seconds
            current_time = self.get_clock().now()
            if (current_time - self.last_log_time).nanoseconds / 1e9 >= 2.0:
                loc_rate = self.loc_msg_count / 2.0
                cloud_rate = self.cloud_msg_count / 2.0
                self.get_logger().info(
                    f'Loc: {loc_rate:.1f} Hz | Cloud: {cloud_rate:.1f} Hz ({len(points)} pts)'
                )
                self.loc_msg_count = 0
                self.cloud_msg_count = 0
                self.last_log_time = current_time

        except zmq.Again:
            # No message available
            pass
        except Exception as e:
            self.get_logger().error(f'Error receiving cloud: {e}')
            import traceback
            traceback.print_exc()

    def destroy_node(self):
        """Cleanup"""
        self.get_logger().info('Shutting down Localization Bridge...')
        self.loc_socket.close()
        self.loc_context.term()
        if self.enable_cloud:
            self.cloud_socket.close()
            self.cloud_context.term()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    bridge = LocalizationBridge()

    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
