"""
Inspire FTP Hand Controller for XR Teleoperation
Supports Inspire Robotics FTP hands with ModbusTCP protocol
INTEGRATED VERSION: Includes ModbusDataHandler directly (no separate drivers needed)
"""

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
import sys
import os

# Add inspire_hand_sdk to path
inspire_hand_ws_path = os.path.expanduser("~/Documents/Unitree/inspire_hand_ws")
sys.path.insert(0, os.path.join(inspire_hand_ws_path, "inspire_hand_sdk"))
sys.path.insert(0, os.path.join(inspire_hand_ws_path, "unitree_sdk2_python"))

from inspire_sdkpy import inspire_sdk, inspire_hand_defaut, inspire_dds

from teleop.robot_control.hand_retargeting import HandRetargeting, HandType
import numpy as np
from enum import IntEnum
import threading
import time
from multiprocessing import Process, Array

import logging_mp
logger_mp = logging_mp.get_logger(__name__)

# Tentar importar IPs do config, senão usar defaults
try:
    from config import INSPIRE_RIGHT_HAND_IP, INSPIRE_LEFT_HAND_IP
    kRightHandIP = INSPIRE_RIGHT_HAND_IP
    kLeftHandIP = INSPIRE_LEFT_HAND_IP
except ImportError:
    kRightHandIP = "192.168.123.211"
    kLeftHandIP = "192.168.123.210"

Inspire_Num_Motors = 6

class Inspire_FTP_Controller:
    def __init__(self, left_hand_array, right_hand_array, dual_hand_data_lock=None, dual_hand_state_array=None,
                 dual_hand_action_array=None, fps=100.0, Unit_Test=False, simulation_mode=False):
        logger_mp.info("Initialize Inspire_FTP_Controller...")
        logger_mp.info("INTEGRATED MODE: ModbusDataHandler included (no separate drivers needed)")
        self.fps = fps
        self.Unit_Test = Unit_Test
        self.simulation_mode = simulation_mode

        if not self.Unit_Test:
            self.hand_retargeting = HandRetargeting(HandType.INSPIRE_HAND)
        else:
            self.hand_retargeting = HandRetargeting(HandType.INSPIRE_HAND_Unit_Test)

        # Shared Arrays for hand states (filled by ModbusDataHandler threads)
        self.left_hand_state_array = Array('d', Inspire_Num_Motors, lock=True)
        self.right_hand_state_array = Array('d', Inspire_Num_Motors, lock=True)

        # NOTE: ModbusDataHandlers will initialize DDS internally
        # We create them in the control process to avoid multiprocessing issues

        # Start control process (will create ModbusDataHandlers inside)
        hand_control_process = Process(target=self.control_process, args=(left_hand_array, right_hand_array,
                                                                           self.left_hand_state_array, self.right_hand_state_array,
                                                                           dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array))
        hand_control_process.daemon = True
        hand_control_process.start()

        # Wait for ModbusTCP connection (only for physical robot)
        if not self.simulation_mode:
            logger_mp.info("[Inspire_FTP_Controller] Waiting for ModbusTCP connection...")
            timeout = 10  # seconds
            start_wait = time.time()
            while True:
                if any(self.right_hand_state_array) or any(self.left_hand_state_array):
                    break
                if time.time() - start_wait > timeout:
                    logger_mp.warning("[Inspire_FTP_Controller] Timeout waiting for hand connection. Continuing anyway...")
                    break
                time.sleep(0.1)
            logger_mp.info("[Inspire_FTP_Controller] Hands connected!")
        else:
            logger_mp.info("[Inspire_FTP_Controller] Simulation mode - no physical connection needed")
            time.sleep(0.5)  # Small delay to ensure DDS publishers are ready

        logger_mp.info("Initialize Inspire_FTP_Controller OK!\n")

    def _read_hand_states_thread(self, right_handler, left_handler, right_state_array, left_state_array):
        """
        Thread that continuously reads hand states from ModbusDataHandlers
        and updates shared arrays
        """
        while self.running:
            try:
                # Read right hand state
                right_data = right_handler.read()
                if right_data and 'ANGLE_ACT' in right_data:
                    with right_state_array.get_lock():
                        for idx in range(Inspire_Num_Motors):
                            # Normalize from 0-1000 to 0-1
                            right_state_array[idx] = right_data['ANGLE_ACT'][idx] / 1000.0

                # Read left hand state
                left_data = left_handler.read()
                if left_data and 'ANGLE_ACT' in left_data:
                    with left_state_array.get_lock():
                        for idx in range(Inspire_Num_Motors):
                            # Normalize from 0-1000 to 0-1
                            left_state_array[idx] = left_data['ANGLE_ACT'][idx] / 1000.0

                time.sleep(0.001)
            except Exception as e:
                logger_mp.error(f"[Inspire_FTP_Controller] Error reading hand states: {e}")
                time.sleep(0.1)

    def control_process(self, left_hand_array, right_hand_array, left_hand_state_array, right_hand_state_array,
                        dual_hand_data_lock=None, dual_hand_state_array=None, dual_hand_action_array=None):
        self.running = True

        try:
            # CRITICAL: Initialize DDS in this process (child process)
            # DDS must be initialized in the same process where publishers are created
            try:
                from config import NETWORK_INTERFACE
                iface = NETWORK_INTERFACE
            except ImportError:
                iface = "enx00e6021980f3"

            logger_mp.info(f"[Inspire_FTP_Controller] Initializing DDS in control process on interface: {iface}")
            ChannelFactoryInitialize(0, iface)

            # SIMULATION MODE: No ModbusTCP, only DDS
            if self.simulation_mode:
                logger_mp.info("[Inspire_FTP_Controller] SIMULATION MODE: Skipping ModbusTCP connection")
                logger_mp.info("[Inspire_FTP_Controller] Will publish DDS commands only")
                right_handler = None
                left_handler = None
            # PHYSICAL ROBOT MODE: Create ModbusTCP handlers
            else:
                logger_mp.info("[Inspire_FTP_Controller] Creating ModbusDataHandlers...")

                # Create ModbusDataHandlers for both hands (connects ModbusTCP + DDS)
                # DDS is now initialized in this process, so we use initDDS=False
                right_handler = inspire_sdk.ModbusDataHandler(
                    ip=kRightHandIP,
                    LR='r',
                    device_id=1,
                    initDDS=False  # DDS already initialized above
                )
                logger_mp.info(f"[Inspire_FTP_Controller] Right hand connected: {kRightHandIP}")

                left_handler = inspire_sdk.ModbusDataHandler(
                    ip=kLeftHandIP,
                    LR='l',
                    device_id=1,
                    initDDS=False  # DDS already initialized above
                )
                logger_mp.info(f"[Inspire_FTP_Controller] Left hand connected: {kLeftHandIP}")

                # Start thread to read hand states and update shared arrays (physical robot only)
                state_read_thread = threading.Thread(
                    target=self._read_hand_states_thread,
                    args=(right_handler, left_handler, right_hand_state_array, left_hand_state_array)
                )
                state_read_thread.daemon = True
                state_read_thread.start()

            # Create DDS publishers to send commands to hands (both simulation and physical)
            logger_mp.info("[Inspire_FTP_Controller] Creating DDS command publishers...")
            right_cmd_pub = ChannelPublisher("rt/inspire_hand/ctrl/r", inspire_dds.inspire_hand_ctrl)
            right_cmd_pub.Init()
            left_cmd_pub = ChannelPublisher("rt/inspire_hand/ctrl/l", inspire_dds.inspire_hand_ctrl)
            left_cmd_pub.Init()
            time.sleep(0.5)  # Wait for publishers to initialize
            logger_mp.info("[Inspire_FTP_Controller] DDS publishers ready")

            # Control loop
            left_q_target = np.full(Inspire_Num_Motors, 0.0)   # Start open
            right_q_target = np.full(Inspire_Num_Motors, 0.0)  # Start open

            logger_mp.info("[Inspire_FTP_Controller] Control loop started")

            while self.running:
                start_time = time.time()

                # Get dual hand XR tracking data
                with left_hand_array.get_lock():
                    left_hand_data = np.array(left_hand_array[:]).reshape(25, 3).copy()
                with right_hand_array.get_lock():
                    right_hand_data = np.array(right_hand_array[:]).reshape(25, 3).copy()

                # Read left and right q_state from shared arrays
                state_data = np.concatenate((np.array(left_hand_state_array[:]), np.array(right_hand_state_array[:])))

                # Check if hand data has been initialized
                if not np.all(right_hand_data == 0.0) and not np.all(left_hand_data[4] == np.array([-1.13, 0.3, 0.15])):
                    # Apply hand retargeting
                    ref_left_value = left_hand_data[self.hand_retargeting.left_indices[1, :]] - left_hand_data[self.hand_retargeting.left_indices[0, :]]
                    ref_right_value = right_hand_data[self.hand_retargeting.right_indices[1, :]] - right_hand_data[self.hand_retargeting.right_indices[0, :]]

                    left_q_target = self.hand_retargeting.left_retargeting.retarget(ref_left_value)[self.hand_retargeting.left_dex_retargeting_to_hardware]
                    right_q_target = self.hand_retargeting.right_retargeting.retarget(ref_right_value)[self.hand_retargeting.right_dex_retargeting_to_hardware]

                    # Normalize values from radians to [0, 1]
                    def normalize(val, min_val, max_val):
                        return np.clip((max_val - val) / (max_val - min_val), 0.0, 1.0)

                    for idx in range(Inspire_Num_Motors):
                        if idx <= 3:
                            left_q_target[idx] = normalize(left_q_target[idx], 0.0, 1.7)
                            right_q_target[idx] = normalize(right_q_target[idx], 0.0, 1.7)
                        elif idx == 4:
                            left_q_target[idx] = normalize(left_q_target[idx], 0.0, 0.5)
                            right_q_target[idx] = normalize(right_q_target[idx], 0.0, 0.5)
                        elif idx == 5:
                            left_q_target[idx] = normalize(left_q_target[idx], -0.1, 1.3)
                            right_q_target[idx] = normalize(right_q_target[idx], -0.1, 1.3)

                # Get dual hand action
                action_data = np.concatenate((left_q_target, right_q_target))
                if dual_hand_state_array is not None and dual_hand_action_array is not None:
                    with dual_hand_data_lock:
                        dual_hand_state_array[:] = state_data
                        dual_hand_action_array[:] = action_data

                # Create and publish commands to hands
                # Convert normalized values [0-1] to angle_set range [0-1000]
                left_cmd = inspire_hand_defaut.get_inspire_hand_ctrl()
                left_cmd.angle_set = [int(val * 1000) for val in left_q_target]
                left_cmd.mode = 0b0001  # Mode 1: angle control

                right_cmd = inspire_hand_defaut.get_inspire_hand_ctrl()
                right_cmd.angle_set = [int(val * 1000) for val in right_q_target]
                right_cmd.mode = 0b0001  # Mode 1: angle control

                # Publish commands to DDS (ModbusDataHandlers will forward to ModbusTCP)
                left_cmd_pub.Write(left_cmd)
                right_cmd_pub.Write(right_cmd)

                # Maintain loop frequency
                current_time = time.time()
                time_elapsed = current_time - start_time
                sleep_time = max(0, (1 / self.fps) - time_elapsed)
                time.sleep(sleep_time)

        except Exception as e:
            logger_mp.error(f"[Inspire_FTP_Controller] Error in control process: {e}")
        finally:
            self.running = False
            logger_mp.info("Inspire_FTP_Controller has been closed.")
