#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Test script for robotic hand hardware control without policy inference.
Tests motor communication, position reading, and command sending.
"""

import os, sys, time
import argparse
from collections import deque
import numpy as np
from dynamixel_sdk import *

# Import utilities
from utils.config_loader import Config
from utils.transform import reorder_isaaclab_to_mujoco, reorder_mujoco_to_isaaclab, get_default_joint_positions_sim_mj
from utils.hardware_converter import HardwareConverter
from utils.input import setup_input, getch

# =========================
# Argument parsing
# =========================
def parse_args():
    parser = argparse.ArgumentParser(description='Test robotic hand hardware control')
    parser.add_argument('--config', '-c',
                       type=str,
                       default="config/anthro_standard.yaml",
                       help='Path to configuration file')
    parser.add_argument('--test-mode', '-t',
                       type=str,
                       default="wave",
                       choices=['static', 'wave', 'sine', 'incremental'],
                       help='Test mode: static (hold pose), wave (sequential fingers), sine (smooth motion), incremental (step through positions)')
    parser.add_argument('--speed', '-s',
                       type=float,
                       default=1.0,
                       help='Speed multiplier for motion tests (default: 1.0)')
    return parser.parse_args()

args = parse_args()

# =========================
# Load configuration
# =========================
config = Config(args.config)
print(f"Using config: {args.config}")
print(f"Test mode: {args.test_mode}")

# =========================
# Config
# =========================
CONTROL_HZ = config.control_hz
TEST_SPEED = args.speed

# =========================
# Set up 
# =========================
hardware_converter = HardwareConverter(config)
fd, old_settings = setup_input()

# =========================
# Dynamixel constants from config
# =========================
ADDR_TORQUE_ENABLE = config.get_dynamixel_address('torque_enable')
ADDR_GOAL_POSITION = config.get_dynamixel_address('goal_position')
LEN_GOAL_POSITION = config.get_data_length('goal_position')
ADDR_PRESENT_POSITION = config.get_dynamixel_address('present_position')
LEN_PRESENT_POSITION = config.get_data_length('present_position')
ADDR_MIN_POSITION_LIMIT = config.get_dynamixel_address('min_position_limit')
ADDR_MAX_POSITION_LIMIT = config.get_dynamixel_address('max_position_limit')
ADDR_PROFILE_VELOCITY = config.get_dynamixel_address('profile_velocity')
ADDR_PROFILE_ACCELERATION = config.get_dynamixel_address('profile_acceleration')
ADDR_POSITION_P_GAIN = config.get_dynamixel_address('position_p_gain')
ADDR_POSITION_I_GAIN = config.get_dynamixel_address('position_i_gain')
ADDR_POSITION_D_GAIN = config.get_dynamixel_address('position_d_gain')
ADDR_FIRMWARE_VERSION = config.get_dynamixel_address('firmware_version')

BAUDRATE = config.dynamixel_baudrate
PROTOCOL_VERSION = config.dynamixel_protocol_version

DXL_ID_ALL = config.motor_ids
DXL_ID_MOVE = DXL_ID_ALL[:]

PALM_IDS = config.palm_ids
LIMIT_OFFSETS = config.limit_offsets
DXL_PID_GAINS = config.pid_gains

DEVICENAME = config.dynamixel_device_name
TORQUE_ENABLE = config.get('dynamixel.torque_enable_value')
TORQUE_DISABLE = config.get('dynamixel.torque_disable_value')
INVALID_U32 = config.get('dynamixel.invalid_u32')

# =========================
# Init comms
# =========================
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

if not portHandler.openPort(): 
    print("Failed to open the port")
    sys.exit(1)
print("✓ Succeeded to open the port")

if not portHandler.setBaudRate(BAUDRATE): 
    print("Failed to set the baudrate")
    sys.exit(1)
print("✓ Succeeded to change the baudrate")

def all_fw_ok(ids):
    for i in ids:
        fw, _, _ = packetHandler.read1ByteTxRx(portHandler, i, ADDR_FIRMWARE_VERSION)
        if fw is None or fw < 46:
            return False
    return True

FAST_API = hasattr(groupSyncRead, 'fastSyncRead')
FAST_OK = FAST_API and all_fw_ok(DXL_ID_ALL)
print(f"✓ Fast Sync Read available: {FAST_OK}")

# =========================
# Configure motors + limits
# =========================
motor_limits = {}
profile_velocity = config.get('dynamixel.profile_velocity')
profile_acceleration = config.get('dynamixel.profile_acceleration')

print("\nConfiguring motors...")
for i, motor_id in enumerate(DXL_ID_ALL):
    packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    min_limit, _, _ = packetHandler.read4ByteTxRx(portHandler, motor_id, ADDR_MIN_POSITION_LIMIT)
    max_limit, _, _ = packetHandler.read4ByteTxRx(portHandler, motor_id, ADDR_MAX_POSITION_LIMIT)
    tight_lo = max(0, min(min_limit + LIMIT_OFFSETS[i][0], 4095))
    tight_hi = max(0, min(max_limit + LIMIT_OFFSETS[i][1], 4095))
    if tight_lo >= tight_hi:
        center = (min_limit + max_limit) // 2
        tight_lo, tight_hi = center - 512, center + 512

    motor_limits[motor_id] = {
        'lo': min_limit, 'hi': max_limit,
        'tight_lo': tight_lo, 'tight_hi': tight_hi,
        'mid': (tight_lo + tight_hi) // 2
    }

    packetHandler.write2ByteTxRx(portHandler, motor_id, ADDR_POSITION_P_GAIN, DXL_PID_GAINS[i][0])
    packetHandler.write2ByteTxRx(portHandler, motor_id, ADDR_POSITION_I_GAIN, DXL_PID_GAINS[i][1])
    packetHandler.write2ByteTxRx(portHandler, motor_id, ADDR_POSITION_D_GAIN, DXL_PID_GAINS[i][2])
    packetHandler.write4ByteTxRx(portHandler, motor_id, ADDR_PROFILE_VELOCITY, profile_velocity)
    packetHandler.write4ByteTxRx(portHandler, motor_id, ADDR_PROFILE_ACCELERATION, profile_acceleration)

    print(f"  Motor {motor_id:2d}: tight [{tight_lo:4d}, {tight_hi:4d}], mid={motor_limits[motor_id]['mid']:4d}")

# Enable torque
print("\nEnabling torque...")
for motor_id in DXL_ID_MOVE:
    packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
print("✓ Torque enabled on all motors")

# SyncRead: read ALL for observations
for motor_id in DXL_ID_ALL:
    if not groupSyncRead.addParam(motor_id):
        print(f"[ID:{motor_id:03d}] groupSyncRead addparam failed")
        sys.exit(1)

# last-known ticks cache for robust reads
last_known_ticks = {mid: motor_limits[mid]['mid'] for mid in DXL_ID_ALL}

# =========================
# Helper functions
# =========================
def read_all_positions_sim_mj() -> np.ndarray:
    """Read all motor positions and convert to simulation radians"""
    _ = groupSyncRead.fastSyncRead() if FAST_OK else groupSyncRead.txRxPacket()
    qpos_sim = np.zeros(20, dtype=np.float32)
    for i, motor_id in enumerate(DXL_ID_ALL):
        val = INVALID_U32
        if groupSyncRead.isAvailable(motor_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
            val = groupSyncRead.getData(motor_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        ticks = last_known_ticks.get(motor_id, motor_limits[motor_id]['mid'])
        if val != INVALID_U32:
            ticks = val
        # clamp and cache
        lo = motor_limits[motor_id]['tight_lo']
        hi = motor_limits[motor_id]['tight_hi']
        ticks = min(max(int(ticks), lo), hi)
        last_known_ticks[motor_id] = ticks
        # to sim-rad with per-joint range
        qpos_sim[i] = hardware_converter.ticks_to_sim_rad(motor_limits, motor_id, ticks)
    return qpos_sim

def send_goals_ticks_mj(targets_sim_mj: np.ndarray):
    """Send goal positions in simulation radians (MuJoCo order)"""
    for idx, motor_id in enumerate(DXL_ID_ALL):
        if motor_id not in DXL_ID_MOVE: 
            continue
        goal_ticks = hardware_converter.sim_rad_to_ticks(motor_limits, motor_id, float(targets_sim_mj[idx]))
        param = [
            DXL_LOBYTE(DXL_LOWORD(goal_ticks)), DXL_HIBYTE(DXL_LOWORD(goal_ticks)),
            DXL_LOBYTE(DXL_HIWORD(goal_ticks)), DXL_HIBYTE(DXL_HIWORD(goal_ticks))
        ]
        if not groupSyncWrite.addParam(motor_id, param):
            print(f"[ID:{motor_id:03d}] groupSyncWrite addparam failed")
            sys.exit(1)
    dxl_comm_result = groupSyncWrite.txPacket()
    groupSyncWrite.clearParam()
    if dxl_comm_result != COMM_SUCCESS:
        print(PacketHandler(PROTOCOL_VERSION).getTxRxResult(dxl_comm_result))

# =========================
# Test motion generators
# =========================
def get_test_positions(mode: str, t: float, base_pose: np.ndarray) -> np.ndarray:
    """
    Generate test positions based on mode.
    
    Args:
        mode: Test mode ('static', 'wave', 'sine', 'incremental')
        t: Time in seconds
        base_pose: Base position array (20,)
    
    Returns:
        Target positions in simulation radians (MuJoCo order)
    """
    if mode == "static":
        # Just return base pose
        return base_pose.copy()
    
    elif mode == "wave":
        # Wave through fingers sequentially
        targets = base_pose.copy()
        period = 5.0 / TEST_SPEED  # 5 second cycle
        phase = (t % period) / period
        
        # Each finger gets its turn (5 fingers, but only 4 active based on config)
        finger_idx = int(phase * 4)  # 0-3 for 4 fingers
        
        # Move all joints of the active finger
        start_idx = finger_idx * 5
        amplitude = 0.5  # radians
        for j in range(5):
            if start_idx + j < 20:
                # Sine wave for smooth motion
                targets[start_idx + j] += amplitude * np.sin(2 * np.pi * phase * 4)
        
        return targets
    
    elif mode == "sine":
        # Smooth sinusoidal motion on all joints
        targets = base_pose.copy()
        freq = 0.5 * TEST_SPEED  # Hz
        amplitude = 0.3  # radians
        
        # Different phase for each joint for interesting motion
        for i in range(20):
            targets[i] += amplitude * np.sin(2 * np.pi * freq * t + i * 0.1)
        
        return targets
    
    elif mode == "incremental":
        # Step through predefined poses
        poses = [
            base_pose.copy(),  # Rest pose
            base_pose.copy() + 0.3,  # All joints +0.3 rad
            base_pose.copy() + 0.6,  # All joints +0.6 rad
            base_pose.copy(),  # Back to rest
        ]
        
        pose_duration = 3.0 / TEST_SPEED  # seconds per pose
        pose_idx = int(t / pose_duration) % len(poses)
        return poses[pose_idx]
    
    else:
        return base_pose.copy()

# =========================
# Main test loop
# =========================
PERIOD = 1.0 / CONTROL_HZ
next_tick = time.perf_counter()

cmd_periods = deque(maxlen=120)
read_periods = deque(maxlen=120)
t_prev_tx = None
read_t_prev = None
last_print = time.perf_counter()
missed_ticks = 0
start_time = time.perf_counter()

# Get base pose from config (in MuJoCo order)
base_pose_sim = get_default_joint_positions_sim_mj(config).astype(np.float32)
cur_targets_sim = base_pose_sim.copy()

# Clamp to valid range
sim_min = hardware_converter.sim_min
sim_max = hardware_converter.sim_max
cur_targets_sim = np.minimum(np.maximum(cur_targets_sim, sim_min), sim_max)

# Send initial pose
send_goals_ticks_mj(cur_targets_sim)
time.sleep(0.5)  # Give time to reach initial pose

print("\n" + "="*60)
print(f"TEST MODE: {args.test_mode.upper()}")
print("="*60)
print("Commands:")
print("  'q' or ESC - Quit")
print("  '1' - Static mode")
print("  '2' - Wave mode")
print("  '3' - Sine mode")
print("  '4' - Incremental mode")
print("  'r' - Reset to base pose")
print("  SPACE - Pause/Resume motion")
print("="*60)
print("\nPress any key to start test...\n")

# Block once for activation
ch_block = getch(block=True, timeout=None, flush=True)
if ch_block in ('\x1b', 'q'):
    # Clean shutdown
    groupSyncRead.clearParam()
    for motor_id in DXL_ID_MOVE:
        packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    portHandler.closePort()
    print("Exiting...")
    sys.exit(0)

test_mode = args.test_mode
test_active = True
print(f"\n[START] Test mode: {test_mode}")

try:
    while True:
        # Check for keyboard input
        ch = getch(block=False, timeout=0)
        if ch in ('\x1b', 'q'):
            print("\nQuitting...")
            break
        elif ch == '1':
            test_mode = 'static'
            print(f"\n[MODE] Switched to: {test_mode}")
        elif ch == '2':
            test_mode = 'wave'
            print(f"\n[MODE] Switched to: {test_mode}")
        elif ch == '3':
            test_mode = 'sine'
            print(f"\n[MODE] Switched to: {test_mode}")
        elif ch == '4':
            test_mode = 'incremental'
            print(f"\n[MODE] Switched to: {test_mode}")
        elif ch == 'r':
            print("\n[RESET] Returning to base pose...")
            cur_targets_sim = base_pose_sim.copy()
            send_goals_ticks_mj(cur_targets_sim)
        elif ch == ' ':
            test_active = not test_active
            print(f"\n[PAUSE] Motion {'resumed' if test_active else 'paused'}")

        # Read current positions
        t_before_read = time.perf_counter()
        qpos_sim = read_all_positions_sim_mj()
        t_after_read = time.perf_counter()
        if read_t_prev is not None:
            read_periods.append(t_after_read - read_t_prev)
        read_t_prev = t_after_read

        # Generate test motion
        if test_active:
            elapsed_time = time.perf_counter() - start_time
            new_targets_sim = get_test_positions(test_mode, elapsed_time, base_pose_sim)
            # Clamp to valid range
            new_targets_sim = np.minimum(np.maximum(new_targets_sim, sim_min), sim_max)
        else:
            new_targets_sim = cur_targets_sim

        # Send commands
        now = time.perf_counter()
        if t_prev_tx is not None:
            cmd_periods.append(now - t_prev_tx)
        t_prev_tx = now
        send_goals_ticks_mj(new_targets_sim)
        t_after_write = time.perf_counter()

        cur_targets_sim = new_targets_sim

        # Print status every second
        if cmd_periods and (time.perf_counter() - last_print) >= 1.0:
            avg_cmd = 1.0 / (sum(cmd_periods) / len(cmd_periods))
            avg_read = 1.0 / (sum(read_periods) / len(read_periods)) if read_periods else 0.0
            
            # Calculate position errors
            errors = np.abs(qpos_sim - cur_targets_sim)
            max_error = np.max(errors)
            avg_error = np.mean(errors)
            
            print(f"[STATUS] Mode={test_mode:12s} | "
                  f"Cmd≈{avg_cmd:.1f}Hz | Read≈{avg_read:.1f}Hz | "
                  f"Error: avg={avg_error:.3f}rad max={max_error:.3f}rad | "
                  f"Missed={missed_ticks} | Active={test_active}")
            
            # Show sample joint values
            print(f"         Joints[0-4]: pos=[{qpos_sim[0]:+.2f}, {qpos_sim[1]:+.2f}, {qpos_sim[2]:+.2f}, {qpos_sim[3]:+.2f}, {qpos_sim[4]:+.2f}] "
                  f"tgt=[{cur_targets_sim[0]:+.2f}, {cur_targets_sim[1]:+.2f}, {cur_targets_sim[2]:+.2f}, {cur_targets_sim[3]:+.2f}, {cur_targets_sim[4]:+.2f}]")
            
            last_print = time.perf_counter()

        # Maintain control frequency
        next_tick += PERIOD
        sleep_dt = next_tick - time.perf_counter()
        if sleep_dt > 0:
            time.sleep(sleep_dt)
        else:
            missed_ticks += 1
            next_tick = time.perf_counter()

except KeyboardInterrupt:
    print("\n\nInterrupted by user")

finally:
    print("\nCleaning up...")
    # Return to base pose before disabling
    print("Returning to base pose...")
    send_goals_ticks_mj(base_pose_sim)
    time.sleep(1.0)
    
    # Disable torque
    print("Disabling torque...")
    groupSyncRead.clearParam()
    for motor_id in DXL_ID_MOVE:
        packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    portHandler.closePort()
    print("✓ Test complete. Port closed.")