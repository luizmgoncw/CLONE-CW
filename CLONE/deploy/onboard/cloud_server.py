#!/usr/bin/env python3
"""
Cloud Server - G1 PC2
Subscribes to ROS1 PointCloud2 topics and publishes via ZeroMQ to Server PC.
"""

import zmq
import rospy
from sensor_msgs.msg import PointCloud2
import numpy as np
import struct
import time
from collections import deque


class CloudServer:
    def __init__(self, config: dict = None):
        self.port = config.get('port', 6007)
        self.server_ip = config.get('server_ip', '192.168.123.164')
        self.publish_rate = config.get('publish_rate', 20.0)  # Hz
        self.downsample_factor = config.get('downsample_factor', 1)  # Keep 1 in N points

        # Statistics
        self.msg_count = 0
        self.last_log_time = time.time()
        self.last_publish_time = 0
        self.publish_interval = 1.0 / self.publish_rate
        self.first_message = True

        # Latest cloud data
        self.latest_cloud = None

        # ZeroMQ Publisher
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(f"tcp://*:{self.port}")

        print(f"[Cloud Server] Started on port {self.port}")
        print(f"[Cloud Server] Publishing at {self.publish_rate} Hz")
        print(f"[Cloud Server] Downsampling: keeping 1 in {self.downsample_factor} points")

    def cloud_callback(self, msg):
        """Callback for PointCloud2 messages"""
        current_time = time.time()

        # Rate limiting - only process if enough time has passed
        if current_time - self.last_publish_time < self.publish_interval:
            return

        try:
            # Extract basic info
            height = msg.height
            width = msg.width
            point_step = msg.point_step
            row_step = msg.row_step

            # Parse point cloud data
            # PointCloud2 format: each point has x, y, z (float32) + intensity (float32) + other fields
            data = np.frombuffer(msg.data, dtype=np.uint8)

            # Find intensity field offset (if it exists)
            intensity_offset = None
            for field in msg.fields:
                if field.name == 'intensity':
                    intensity_offset = field.offset
                    break

            # Log format on first message
            if self.first_message:
                print(f"[Cloud Server] PointCloud2 format:")
                print(f"  - point_step: {point_step} bytes")
                print(f"  - fields: {[f.name for f in msg.fields]}")
                if intensity_offset is not None:
                    print(f"  - intensity offset: {intensity_offset}")
                else:
                    print(f"  - intensity: NOT FOUND (will use default 0.0)")
                self.first_message = False

            # Downsample: keep every Nth point
            # This significantly reduces data size
            total_points = height * width
            step = self.downsample_factor

            # Extract x, y, z, intensity coordinates
            points = []
            for i in range(0, total_points, step):
                offset = i * point_step

                # Always try to extract x, y, z (first 12 bytes)
                if offset + 12 > len(data):
                    continue

                x, y, z = struct.unpack_from('fff', data, offset)

                # Filter out invalid points (nan, inf)
                if np.isnan(x) or np.isnan(y) or np.isnan(z) or \
                   np.isinf(x) or np.isinf(y) or np.isinf(z):
                    continue

                # Try to extract intensity if available
                intensity = 0.0
                if intensity_offset is not None and offset + intensity_offset + 4 <= len(data):
                    intensity = struct.unpack_from('f', data, offset + intensity_offset)[0]
                    # Clamp intensity to reasonable range
                    if np.isnan(intensity) or np.isinf(intensity):
                        intensity = 0.0

                points.append([x, y, z, intensity])

            if len(points) == 0:
                return

            # Convert to numpy array with explicit shape
            points_array = np.array(points, dtype=np.float32).reshape(-1, 4)

            # Verify shape
            if points_array.shape[1] != 4:
                print(f"[Cloud Server] ERROR: Invalid points array shape: {points_array.shape}")
                return

            # Create header with metadata
            # Force frame_id to 'map' for compatibility with RViz2 Fixed Frame
            header = {
                'stamp': msg.header.stamp.to_sec(),
                'frame_id': 'map',  # Changed from msg.header.frame_id (was 'camera_init')
                'num_points': len(points),
                'original_points': total_points,
            }

            # Serialize and send
            # Format: header (dict) + points (numpy array)
            import pickle
            message = pickle.dumps({
                'header': header,
                'points': points_array
            })

            self.socket.send(message)

            self.last_publish_time = current_time
            self.msg_count += 1

            # Log statistics
            if current_time - self.last_log_time >= 2.0:
                rate = self.msg_count / (current_time - self.last_log_time)
                size_kb = len(message) / 1024.0
                compression_ratio = (total_points / len(points)) if len(points) > 0 else 0
                print(f"[Cloud Server] Rate: {rate:.1f} Hz | "
                      f"Points: {len(points)}/{total_points} ({compression_ratio:.1f}x) | "
                      f"Size: {size_kb:.1f} KB")
                self.msg_count = 0
                self.last_log_time = current_time

        except Exception as e:
            print(f"[Cloud Server] Error processing cloud: {e}")
            import traceback
            traceback.print_exc()

    def start(self):
        """Start ROS subscriber"""
        rospy.init_node('cloud_server', anonymous=True)

        # Subscribe to cloud_registered topic
        rospy.Subscriber('/cloud_registered', PointCloud2, self.cloud_callback)

        print("[Cloud Server] Subscribed to /cloud_registered")
        print("[Cloud Server] Waiting for point clouds...")

        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("\n[Cloud Server] Interrupted by user.")
        finally:
            self.close()

    def close(self):
        """Cleanup"""
        self.socket.close()
        self.context.term()
        print("[Cloud Server] Closed.")


if __name__ == "__main__":
    config = {
        'port': 6007,
        'server_ip': '192.168.123.164',
        'publish_rate': 20.0,  # Hz (increased for better quality)
        'downsample_factor': 1,  # Keep all points (100% of data for best quality)
    }

    server = CloudServer(config)
    server.start()
