#!/usr/bin/env python3
"""
Debug wrapper for g1_server.py - adds exception handling to identify crash cause
"""
import sys
import traceback

# Import everything from original g1_server
from g1_server import *

# Override the main_loop method with better error handling
original_main_loop = DeployNode.main_loop

@torch.inference_mode()
def debug_main_loop(self):
    """Wrapped main_loop with comprehensive error handling"""
    # keep stand up pose first
    _percent_1 = 0
    _duration_1 = 500
    firstRun = True
    init_success = False

    global COLLECT_SID_DATA
    temp_sid = COLLECT_SID_DATA
    COLLECT_SID_DATA = False

    try:
        while self.stand_up and not self.start_policy:
            if firstRun:
                firstRun = False
                rclpy.spin_once(self)
                start_pos = self.joint_pos
                self.reset_localization()
            else:
                self.set_gains(kp=self.env.p_gains, kd=self.env.d_gains)
                if _percent_1 < 1:
                    self.set_motor_position(q=(1 - _percent_1) * np.array(start_pos) + _percent_1 * np.array(self.env.default_dof_pos_np))
                    _percent_1 += 1 / _duration_1
                    _percent_1 = min(1, _percent_1)
                if _percent_1 == 1 and not init_success:
                    init_success = True
                    print("---Initialized---")
                if not NO_MOTOR:
                    self.motor_pub.publish(self.cmd_msg)
            rclpy.spin_once(self)
            self.update_mujoco()
            if USE_DEX:
                self.dex_shm_send_data[164] = -1.
    except Exception as e:
        print(f"\n❌ ERROR during stand-up phase:")
        print(f"   Error type: {type(e).__name__}")
        print(f"   Error message: {str(e)}")
        print("\n📋 Traceback:")
        traceback.print_exc()
        sys.exit(1)

    COLLECT_SID_DATA = temp_sid
    if USE_DEX:
        self.dex_shm_send_data[164] = 1.

    cnt = 0
    fps_ckt = time.monotonic()

    self.get_logger().info("start main loop")

    try:
        self.reset_localization()
    except Exception as e:
        print(f"\n❌ ERROR in reset_localization:")
        print(f"   Error: {e}")
        traceback.print_exc()
        sys.exit(1)

    rclpy.spin_once(self)

    loop_start_time = time.monotonic()
    iteration = 0

    while rclpy.ok() or NO_ROS:
        iteration += 1

        try:
            mujoco_step_start = time.time()
            if self.Emergency_stop:
                self.stop = True
            if self.stop:
                _percent_1 = 0
                _duration_1 = 1000
                start_pos = self.joint_pos
                while _percent_1 < 1:
                    self.set_motor_position(q=(1 - _percent_1) * np.array(start_pos) + _percent_1 * np.array(start_pos))
                    _percent_1 += 1 / _duration_1
                    _percent_1 = min(1, _percent_1)
                    if not NO_MOTOR:
                        self.motor_pub.publish(self.cmd_msg)
                if COLLECT_MOTION_DATA:
                    now = datetime.datetime.now()
                    torch.save(self.motion_data, f'./rec_motion/motion_data_{now.strftime("%m_%d_%H_%M_%S")}.pkl')
                    print('Motion Data Saved')
                self.get_logger().info("Program exit")
                if COLLECT_SID_DATA:
                    import pickle
                    now = datetime.datetime.now()
                    with open(f'./sys_id/sid_data_{now.strftime("%m_%d_%H_%M_%S")}.pkl', 'wb') as f:
                        pickle.dump(self.sid_data, f)
                    print('SID Data Saved')
                break

            # spin stuff
            while self.control_dt > time.monotonic() - loop_start_time:
                time.sleep(max(0., self.control_dt - (time.monotonic() - loop_start_time) - 0.001))
                pass
            loop_start_time = time.monotonic()
            rclpy.spin_once(self, timeout_sec=0.001)

            # CRITICAL SECTION - wrap in try-except to catch the crash
            try:
                self.compute_observations()
            except Exception as e:
                print(f"\n❌ ERROR in compute_observations (iteration {iteration}):")
                print(f"   Error type: {type(e).__name__}")
                print(f"   Error message: {str(e)}")
                print(f"\n📊 Debug Info:")
                print(f"   obs_joint_pos shape: {self.obs_joint_pos.shape}, dtype: {self.obs_joint_pos.dtype}")
                print(f"   obs_joint_vel shape: {self.obs_joint_vel.shape}, dtype: {self.obs_joint_vel.dtype}")
                print(f"   obs_ang_vel shape: {self.obs_ang_vel.shape}, dtype: {self.obs_ang_vel.dtype}")
                print(f"   projected_gravity shape: {self.projected_gravity.shape}, dtype: {self.projected_gravity.dtype}")
                print(f"   prev_action shape: {self.prev_action.shape}, dtype: {self.prev_action.dtype}")
                print(f"   body_pos_extend shape: {self.body_pos_extend.shape}, dtype: {self.body_pos_extend.dtype}")
                print(f"   trajectories shape: {self.env.trajectories.shape}, dtype: {self.env.trajectories.dtype}")
                print("\n📋 Traceback:")
                traceback.print_exc()
                self.stop = True
                continue

            try:
                raw_actions = self.policy(self.env.obs_buf)
            except Exception as e:
                print(f"\n❌ ERROR in policy inference (iteration {iteration}):")
                print(f"   Error type: {type(e).__name__}")
                print(f"   Error message: {str(e)}")
                print(f"   obs_buf shape: {self.env.obs_buf.shape}, dtype: {self.env.obs_buf.dtype}")
                print("\n📋 Traceback:")
                traceback.print_exc()
                self.stop = True
                continue

            if torch.any(torch.isnan(raw_actions)):
                self.get_logger().info("Emergency stop due to NaN")
                self.set_motor_position(q=self.env.default_dof_pos_np)
                raise SystemExit

            raw_actions_np = raw_actions.clone().detach().cpu().numpy().squeeze(0)
            angles = raw_actions_np * self.env.scale_action + self.env.default_dof_pos_np
            angles = np.clip(angles, self.env.joint_limit_lo_, self.env.joint_limit_hi_)

            self.angles = angles
            self.set_motor_position(self.angles)
            if not NO_MOTOR and not NO_ROS:
                self.motor_pub.publish(self.cmd_msg)

                cnt += 1
                if cnt == 10:
                    dt = (time.monotonic()-fps_ckt)/cnt
                    cnt = 0
                    fps_ckt = time.monotonic()
                    print(f"POL FREQ: {1/dt}")

                    if 1/dt < self.motor_pub_freq:
                        self.control_dt -= 2e-5
                    else:
                        self.control_dt += 2e-5

            obs_buf = self.env.obs_buf.clone().cpu()
            dof = obs_buf[0, :HW_DOF]
            dof_vel = obs_buf[0, HW_DOF:2 * HW_DOF]
            base_ang_vel = obs_buf[0, 2 * HW_DOF:2 * HW_DOF + 3]
            base_gravity = obs_buf[0, 2 * HW_DOF + 3:2 * HW_DOF + 6]
            task_obs = obs_buf[0, 2 * HW_DOF + 6:2 * HW_DOF + 6 + 3 * 3 * 3 + 2 * 4]
            self.env.record_trajectory(dof, dof_vel, base_ang_vel, base_gravity, torch.from_numpy(self.prev_action), task_obs)
            self.prev_action = raw_actions_np

            if self.control_dt > time.monotonic() - loop_start_time + 0.005:
               self.update_mujoco()

            if self.recording:
                cur_motion = torch.cat([
                        self.head_pos - self.rec_delta_pos,
                        self.left_wrist_pos - self.rec_delta_pos,
                        self.right_wrist_pos - self.rec_delta_pos,
                        self.head_vel,
                        self.left_wrist_vel,
                        self.right_wrist_vel,
                        self.left_hand_rot,
                        self.right_hand_rot
                    ]).float()
                if USE_DEX:
                    cur_motion = torch.cat([cur_motion, torch.from_numpy(self.dex_shm_recv_data[:14]).float().clone()])
                cur_motion = cur_motion.clone()
                self.motion_data['seq'].append(cur_motion)

        except KeyboardInterrupt:
            print("\n⚠️  Keyboard interrupt detected")
            self.stop = True
        except SystemExit:
            raise
        except Exception as e:
            print(f"\n❌ UNEXPECTED ERROR in main loop (iteration {iteration}):")
            print(f"   Error type: {type(e).__name__}")
            print(f"   Error message: {str(e)}")
            print("\n📋 Traceback:")
            traceback.print_exc()
            self.stop = True

# Replace the original method
DeployNode.main_loop = debug_main_loop

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--task_name', action='store', type=str, help='Task name: stand, stand_w_waist, wb, squat', required=False, default='stand')
    args = parser.parse_args()

    print("🐛 Running g1_server in DEBUG mode")
    print("=" * 50)

    try:
        rclpy.init(args=None)
        dp_node = DeployNode(args.task_name)
        dp_node.prepare()

        dp_node.get_logger().info("Deploy node started")

        thread = Thread(target=dp_node.vp_loop)
        thread.start()

        dp_node.main_loop()
        dp_node.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        print(f"\n❌ FATAL ERROR:")
        print(f"   Error type: {type(e).__name__}")
        print(f"   Error message: {str(e)}")
        print("\n📋 Full Traceback:")
        traceback.print_exc()
        sys.exit(1)
