#!/usr/bin/env python3
"""
Data Collection Pipeline for Robot Grasping - STANDALONE VERSION

This is a self-contained version with all utilities inlined.
Copy/paste this entire file into Isaac Sim Script Editor.

This script orchestrates the full data collection process:
1. Randomize mug position in workspace
2. Calculate goal end-effector pose (above mug)
3. Randomize robot starting pose
4. Command robot to move to goal pose
5. Record camera data during motion
6. Repeat for N episodes

USAGE: Run INSIDE an already-running Isaac Sim instance using Script Editor:
  1. Start Isaac Sim with stage loaded (/workspace/isaaclab/content/stage-19.usd)
  2. Start IK service in container: ros2 run openpi_kinova_ros2 ik_service.py
     IMPORTANT: Must use 'ros2 run', NOT 'python3'!
  3. Open Script Editor (Window → Script Editor)
  4. Copy/paste this ENTIRE file
  5. Click "Run"

COORDINATE FRAMES:
  - All positions in config are in WORLD frame
  - Robot base (cobot_base_link) at [0.0, -1.40562, -0.62762]
  - Table/ground at Z=0.0 is 62.8cm above robot base
  - IK service uses cobot_kinova.urdf (same as Isaac Sim)
  - Frame transformations handled automatically

VERSION: Standalone (all utilities inlined)
DATE: December 7, 2024
"""

import time
import numpy as np
import math
import os
from pathlib import Path
from datetime import datetime
from pxr import Gf, UsdGeom

# Isaac Sim imports
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core import SimulationContext
from omni.timeline import get_timeline_interface
import omni.usd
import omni.kit.app

# ROS2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

# Set ROS_DOMAIN_ID
os.environ['ROS_DOMAIN_ID'] = '0'


# ============================================================================
# CONFIGURATION
# ============================================================================

# Mug configuration
MUG_ASSET_URL = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac/Props/Mugs/SM_Mug_A2.usd"
MUG_PRIM_PATH = "/World/SM_Mug_A2"

# Workspace configuration - STATIC mug position and goal (world coordinates)
STATIC_MUG_POSITION = np.array([0.025, -0.85, 0.0])  # Fixed mug position on table

# FIXED GOAL JOINT ANGLES (bypasses IK solver entirely!)
# These are known-good joint angles that the robot CAN reach
USE_FIXED_GOAL_JOINTS = True  # Set to False to use IK solver instead
GOAL_JOINTS_FIXED = np.array([0.0, 0.72, 0.0, 0.93, 0.0, 1.54, 0.0])  # 7 DOF

# End-effector goal configuration (only used if USE_FIXED_GOAL_JOINTS = False)
GOAL_HEIGHT_ABOVE_MUG = 0.05  # Meters - how high above mug to position gripper
GOAL_ORIENTATION = [0, 90, 90]  # [roll, pitch, yaw] in degrees

# Starting pose configuration (all in WORLD frame)
START_POSE_RANDOMIZATION = True  # Whether to randomize starting end-effector poses
# Center: Y=-1.0 puts it ~0.4m from robot base in robot frame (Y_robot = -1.0-(-1.40562) = 0.406m)
# Center: Z=0.2 puts it ~0.83m above robot base in robot frame (Z_robot = 0.2-(-0.62762) = 0.828m)
START_POSE_WORKSPACE_CENTER = np.array([0.0, -1.0, 0.2])  # Center for quarter-circle randomization
START_POSE_RADIUS_MIN = 0.15  # Meters - minimum distance from center in XY plane
START_POSE_RADIUS_MAX = 0.30  # Meters - maximum distance from center in XY plane
START_POSE_Z_MIN = 0.25  # Meters - minimum height above table (z > 0, in world frame)
START_POSE_Z_MAX = 0.60  # Meters - maximum height above table (z range: 25-60cm above table)
START_POSE_ORIENTATION = [0, 90, 90]  # [roll, pitch, yaw] degrees - matches working goal orientation!
START_POSE_ORIENTATION_VARIATION = 30  # Degrees - max variation in each orientation axis

# Robot configuration
ROBOT_PRIM_PATH = "/World/kinova_arm"
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

# Robot base position in world coordinates
# This is cobot_base_link (roboticstoolbox's kinematic root)
ROBOT_BASE_WORLD = np.array([0.0, -1.40562, -0.62762])

# Data collection configuration
NUM_EPISODES = 3  # Total number of data collection episodes
SAVE_DIR = Path("/workspace/isaaclab/data/grasping_dataset")  # Where to save collected data

# Movement configuration
MOVE_TIMEOUT = 30.0  # Seconds - max time to wait for arm to reach goal
POSITION_TOLERANCE = 0.01  # Radians - how close to goal is "close enough"
GOAL_POSE_HOLD_TIME = 2.0  # Seconds - how long to hold at goal pose for recording


# ============================================================================
# COORDINATE TRANSFORMATION
# ============================================================================

def world_to_robot_frame(position_world):
    """Convert position from world frame to robot frame."""
    position_world = np.array(position_world)
    position_robot = position_world - ROBOT_BASE_WORLD
    return position_robot


def transform_pose_for_ik(pose_world):
    """
    Transform pose from world to robot frame for IK computation.
    
    Args:
        pose_world: Dict with 'position' [x,y,z] and 'orientation' [r,p,y] in world frame
    
    Returns:
        Dict with 'position' in robot frame, orientation unchanged
    """
    pose_robot = {
        'position': world_to_robot_frame(pose_world['position']).tolist(),
        'orientation': pose_world['orientation'],
    }
    
    if 'orientation_radians' in pose_world:
        pose_robot['orientation_radians'] = pose_world['orientation_radians']
    
    return pose_robot


# ============================================================================
# IK CLIENT (Embedded)
# ============================================================================

def euler_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    """Convert Euler angles (ZYX convention) to quaternion."""
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return [w, x, y, z]


class IKClient:
    """Client for IK service running in container."""
    
    def __init__(self, ros2_node):
        self.node = ros2_node
        self.latest_ik_response = None
        self.response_received = False
        
        self.ik_request_pub = self.node.create_publisher(PoseStamped, '/ik_request', 10)
        self.ik_response_sub = self.node.create_subscription(
            JointState, '/ik_response', self._ik_response_callback, 10)
        
        print("[IK Client] Initialized")
    
    def _ik_response_callback(self, msg):
        self.latest_ik_response = msg
        self.response_received = True
    
    def compute_ik(self, goal_pose, current_joints=None, timeout=3.0):
        """
        Compute IK for goal pose via ROS2 service.
        
        Args:
            goal_pose: Dict with 'position' [x,y,z] and 'orientation' [r,p,y] in degrees
            current_joints: Optional current joint positions (not used currently)
            timeout: Max time to wait for response (seconds)
        
        Returns:
            numpy array of joint positions (radians), or None if failed
        """
        self.response_received = False
        self.latest_ik_response = None
        
        # Create request message
        request = PoseStamped()
        request.header.stamp = self.node.get_clock().now().to_msg()
        request.header.frame_id = 'world'
        
        # Set position
        request.pose.position.x = goal_pose['position'][0]
        request.pose.position.y = goal_pose['position'][1]
        request.pose.position.z = goal_pose['position'][2]
        
        # Convert Euler angles to quaternion
        roll_deg, pitch_deg, yaw_deg = goal_pose['orientation']
        quat = euler_to_quaternion(roll_deg, pitch_deg, yaw_deg)
        
        request.pose.orientation.w = quat[0]
        request.pose.orientation.x = quat[1]
        request.pose.orientation.y = quat[2]
        request.pose.orientation.z = quat[3]
        
        # Publish request
        self.ik_request_pub.publish(request)
        
        # Wait for response
        start_time = time.time()
        iteration = 0
        while (time.time() - start_time) < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            
            # Update viewport occasionally to prevent white screen
            iteration += 1
            if iteration % 10 == 0:
                update_viewport(1)
            
            if self.response_received:
                if self.latest_ik_response.position:
                    joint_angles = np.array(self.latest_ik_response.position)
                    return joint_angles
                else:
                    return None
            
            time.sleep(0.01)
        
        # Timeout
        return None


# ============================================================================
# ROS2 IMPLEMENTATION (Embedded)
# ============================================================================

# Global ROS2 state
_ros2_node = None
_joint_command_publisher = None
_latest_joint_states = None
_joint_states_subscriber = None


def _joint_states_callback(msg):
    """Callback for /joint_states subscriber."""
    global _latest_joint_states
    joint_dict = {}
    for name, position in zip(msg.name, msg.position):
        joint_dict[name] = position
    _latest_joint_states = joint_dict


def initialize_ros2(node_name='isaac_sim_data_collection'):
    """Initialize ROS2 node and create publisher/subscriber."""
    global _ros2_node, _joint_command_publisher, _joint_states_subscriber
    
    print("Initializing ROS2...")
    
    try:
        if not rclpy.ok():
            rclpy.init()
            print("  ✓ ROS2 context initialized")
        else:
            print("  ✓ ROS2 already initialized")
        
        _ros2_node = Node(node_name)
        print(f"  ✓ Created node: {node_name}")
        
        _joint_command_publisher = _ros2_node.create_publisher(JointState, '/joint_command', 10)
        print("  ✓ Created publisher to /joint_command")
        
        _joint_states_subscriber = _ros2_node.create_subscription(
            JointState, '/joint_states', _joint_states_callback, 10)
        print("  ✓ Created subscriber to /joint_states")
        
        # Wait for initial joint states
        print("  Waiting for initial joint states...")
        timeout = 5.0
        start_time = time.time()
        iteration = 0
        
        while _latest_joint_states is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(_ros2_node, timeout_sec=0.1)
            
            # Update viewport to prevent white screen
            iteration += 1
            if iteration % 5 == 0:
                update_viewport(2)
            
            time.sleep(0.1)
        
        if _latest_joint_states is None:
            print("  ⚠️  Warning: No joint states received within timeout")
            return False
        
        print(f"  ✓ Received initial joint states: {len(_latest_joint_states)} joints")
        print("✓ ROS2 initialization complete")
        return True
        
    except Exception as e:
        print(f"✗ ROS2 initialization failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def spin_ros2():
    """Process ROS2 callbacks."""
    if _ros2_node is not None:
        rclpy.spin_once(_ros2_node, timeout_sec=0.0)


def send_joint_command(joint_positions, joint_names=None):
    """Send joint position command to robot via ROS2."""
    global _ros2_node, _joint_command_publisher
    
    if _joint_command_publisher is None:
        print("✗ ROS2 not initialized")
        return False
    
    if joint_names is None:
        joint_names = JOINT_NAMES
    
    try:
        msg = JointState()
        msg.header.stamp = _ros2_node.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.name = joint_names
        msg.position = list(joint_positions)
        
        _joint_command_publisher.publish(msg)
        spin_ros2()
        
        return True
        
    except Exception as e:
        print(f"✗ Failed to send joint command: {e}")
        return False


def get_current_joint_positions(joint_names=None):
    """Get current robot joint positions from /joint_states topic."""
    global _latest_joint_states
    
    if joint_names is None:
        joint_names = JOINT_NAMES
    
    spin_ros2()
    
    if _latest_joint_states is None:
        return None
    
    try:
        positions = []
        for name in joint_names:
            if name in _latest_joint_states:
                positions.append(_latest_joint_states[name])
            else:
                return None
        return positions
    except Exception as e:
        return None


def wait_for_motion_complete(target_positions, joint_names=None, timeout=30.0, 
                             tolerance=0.01, check_interval=0.05):
    """Wait for robot to reach target joint positions."""
    if joint_names is None:
        joint_names = JOINT_NAMES
    
    start_time = time.time()
    last_positions = None
    last_check_time = start_time
    update_counter = 0
    
    while (time.time() - start_time) < timeout:
        current_time = time.time()
        
        # Update viewport every few iterations to prevent white screen
        update_counter += 1
        if update_counter % 5 == 0:
            update_viewport(2)
        
        current_positions = get_current_joint_positions(joint_names)
        
        if current_positions is None:
            time.sleep(check_interval)
            continue
        
        # Check if within tolerance
        max_error = max(abs(c - t) for c, t in zip(current_positions, target_positions))
        all_within_tolerance = max_error <= tolerance
        
        # Check velocity (stopped moving)
        if all_within_tolerance and last_positions is not None:
            dt = current_time - last_check_time
            if dt > 0:
                max_velocity = max(abs(c - p) / dt for c, p in zip(current_positions, last_positions))
                
                if max_velocity < 0.01:  # Stopped
                    return True
        
        last_positions = current_positions
        last_check_time = current_time
        time.sleep(check_interval)
    
    return False


def move_to_joint_positions(joint_positions, joint_names=None, timeout=MOVE_TIMEOUT, 
                            tolerance=POSITION_TOLERANCE):
    """Move robot to target joint positions and wait for completion."""
    if not send_joint_command(joint_positions, joint_names):
        return False
    
    return wait_for_motion_complete(joint_positions, joint_names, timeout, tolerance)


def shutdown_ros2():
    """Clean shutdown of ROS2 node."""
    global _ros2_node
    
    if _ros2_node is not None:
        _ros2_node.destroy_node()
        print("✓ ROS2 node destroyed")
    
    if rclpy.ok():
        rclpy.shutdown()
        print("✓ ROS2 shutdown")


# ============================================================================
# VIEWPORT UPDATE
# ============================================================================

def update_viewport(num_frames=5):
    """Update viewport so changes are visible in livestream."""
    app = omni.kit.app.get_app()
    for _ in range(num_frames):
        app.update()


# ============================================================================
# MUG MANAGEMENT
# ============================================================================

def add_mug_to_scene(mug_url, prim_path, position):
    """Add mug from Isaac Sim cloud to scene."""
    print(f"Adding mug to scene at {position}")
    
    try:
        existing_prim = get_prim_at_path(prim_path)
        if existing_prim and existing_prim.IsValid():
            print(f"✓ Mug already exists at {prim_path}")
            return prim_path
        
        stage = omni.usd.get_context().get_stage()
        prim = stage.DefinePrim(prim_path, "Xform")
        prim.GetReferences().AddReference(mug_url)
        
        time.sleep(0.5)
        
        xformable = UsdGeom.Xformable(prim)
        translate_ops = [op for op in xformable.GetOrderedXformOps() 
                        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate]
        
        translate_op = translate_ops[0] if translate_ops else xformable.AddTranslateOp()
        translate_op.Set(Gf.Vec3d(position[0], position[1], position[2]))
        
        update_viewport()
        print(f"✓ Mug added successfully")
        return prim_path
        
    except Exception as e:
        print(f"✗ Failed to add mug: {e}")
        import traceback
        traceback.print_exc()
        return None


def set_mug_position(prim_path, position):
    """Set mug position."""
    prim = get_prim_at_path(prim_path)
    if not prim or not prim.IsValid():
        print(f"Error: Mug not found at {prim_path}")
        return False
    
    xformable = UsdGeom.Xformable(prim)
    translate_ops = [op for op in xformable.GetOrderedXformOps() 
                    if op.GetOpType() == UsdGeom.XformOp.TypeTranslate]
    
    translate_op = translate_ops[0] if translate_ops else xformable.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(position[0], position[1], position[2]))
    
    update_viewport()
    return True


def randomize_mug_position(origin_1, origin_2, radius):
    """Generate random mug position around one of two origins."""
    origin = origin_1 if np.random.rand() < 0.5 else origin_2
    
    angle = np.random.uniform(0, 2 * np.pi)
    r = np.random.uniform(0, radius)
    offset = np.array([
        r * np.cos(angle),
        r * np.sin(angle),
        0.0
    ])
    
    position = origin + offset
    print(f"  Randomized mug position: ({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f})")
    return position


# ============================================================================
# GOAL POSE CALCULATION
# ============================================================================

def calculate_goal_pose(mug_position, height_above_mug, orientation_deg):
    """Calculate end-effector goal pose based on mug position."""
    goal_position = [
        mug_position[0],
        mug_position[1],
        mug_position[2] + height_above_mug
    ]
    
    goal_orientation = list(orientation_deg)
    
    goal_pose = {
        'position': goal_position,
        'orientation': goal_orientation,
        'orientation_radians': [np.radians(angle) for angle in goal_orientation]
    }
    
    print(f"  Goal position: ({goal_position[0]:.3f}, {goal_position[1]:.3f}, {goal_position[2]:.3f}) m")
    print(f"  Goal orientation: ({goal_orientation[0]:.1f}, {goal_orientation[1]:.1f}, {goal_orientation[2]:.1f}) deg")
    
    return goal_pose


# ============================================================================
# START POSE GENERATION
# ============================================================================

def randomize_start_end_effector_pose(workspace_center, radius_min, radius_max, 
                                      z_min, z_max, base_orientation_deg, 
                                      orientation_variation_deg):
    """
    Generate random starting end-effector pose in a quarter circle.
    
    Constraints:
    - Quarter circle in XY plane (x can be ±, y must be > 0)
    - z > 0 (above table)
    - Distance from center: radius_min to radius_max
    """
    # Generate random angle in quarter circle (0 to 90 degrees in first/second quadrant)
    # 0° = +Y axis, 90° = ±X axis
    angle = np.random.uniform(-np.pi/2, np.pi/2)  # -90° to +90° (y always positive)
    
    # Generate random radius
    radius = np.random.uniform(radius_min, radius_max)
    
    # Calculate XY offset (quarter circle, y > 0)
    offset_x = radius * np.sin(angle)  # Can be positive or negative
    offset_y = radius * np.cos(angle)  # Always positive (cos is positive in [-π/2, π/2])
    
    # Generate random Z height (always above table, z > 0)
    z_height = np.random.uniform(z_min, z_max)
    
    # Create position
    ee_position = np.array([
        workspace_center[0] + offset_x,
        workspace_center[1] + offset_y,
        z_height  # Absolute height, not offset from center
    ])
    
    # Verify constraints
    assert ee_position[1] > workspace_center[1], f"Y must be positive offset from center! Got y={ee_position[1]:.3f}"
    assert ee_position[2] > 0, f"Z must be above table (z > 0)! Got z={ee_position[2]:.3f}"
    
    # Generate random orientation variation
    orientation_variation = np.random.uniform(
        -orientation_variation_deg,
        orientation_variation_deg,
        size=3
    )
    
    ee_orientation = [
        base_orientation_deg[0] + orientation_variation[0],
        base_orientation_deg[1] + orientation_variation[1],
        base_orientation_deg[2] + orientation_variation[2]
    ]
    
    start_pose = {
        'position': ee_position.tolist(),
        'orientation': ee_orientation,
        'orientation_radians': [np.radians(angle) for angle in ee_orientation]
    }
    
    print(f"  Random start end-effector pose (quarter circle):")
    print(f"    Position: ({ee_position[0]:.3f}, {ee_position[1]:.3f}, {ee_position[2]:.3f}) m")
    print(f"    Angle: {np.degrees(angle):.1f}°, Radius: {radius:.3f} m")
    print(f"    Orientation: ({ee_orientation[0]:.1f}, {ee_orientation[1]:.1f}, {ee_orientation[2]:.1f}) deg")
    
    return start_pose


# ============================================================================
# INVERSE KINEMATICS
# ============================================================================

def calculate_ik(goal_pose, ik_client, current_joint_positions=None):
    """
    Calculate joint angles to reach goal end-effector pose.
    
    Uses IK service via ROS2 (ik_service.py in container).
    Automatically transforms from world frame to robot frame.
    """
    print(f"  Calculating IK for world pose: {goal_pose['position']}")
    
    # Transform from world frame to robot frame
    goal_pose_robot = transform_pose_for_ik(goal_pose)
    print(f"  Robot frame: {goal_pose_robot['position']}")
    
    # Request IK from service
    joint_angles = ik_client.compute_ik(goal_pose_robot, current_joints=current_joint_positions, 
                                       timeout=3.0)
    
    if joint_angles is None:
        print(f"  ✗ IK failed to find solution")
        return None
    
    # IK returns 7 joints, take first 6 for our 6-DOF control
    joint_angles_arm = joint_angles[:6]
    
    print(f"  ✓ IK solution found:")
    for i, angle in enumerate(joint_angles_arm, 1):
        print(f"    joint_{i}: {angle:+.3f} rad ({np.degrees(angle):+.1f} deg)")
    
    return np.array(joint_angles_arm)


# ============================================================================
# DATA RECORDING
# ============================================================================

class DataRecorder:
    """Handles recording of camera data and metadata during episodes."""
    
    def __init__(self, save_dir):
        self.save_dir = Path(save_dir)
        self.save_dir.mkdir(parents=True, exist_ok=True)
        self.episode_count = 0
        self.is_recording = False
        
        print(f"Data recorder initialized. Save directory: {self.save_dir}")
    
    def start_episode(self, episode_num, metadata):
        """Start recording a new episode."""
        # TODO: Implement episode recording start
        print(f"  TODO: Start recording episode {episode_num}")
        self.is_recording = True
        self.episode_count = episode_num
    
    def record_frame(self):
        """Record a single frame (camera images, joint states, etc.)."""
        # TODO: Implement frame capture
        pass
    
    def stop_episode(self):
        """Stop recording current episode and save data."""
        # TODO: Implement episode recording stop
        print(f"  TODO: Stop recording episode {self.episode_count}")
        self.is_recording = False
    
    def get_episode_path(self, episode_num):
        """Get path for episode data."""
        return self.save_dir / f"episode_{episode_num:04d}"


# ============================================================================
# MAIN DATA COLLECTION LOOP
# ============================================================================

def collect_single_episode(episode_num, mug_path, recorder, ik_client):
    """Collect data for a single episode."""
    print(f"\n{'='*60}")
    print(f"EPISODE {episode_num}/{NUM_EPISODES}")
    print(f"{'='*60}")
    
    try:
        # Step 1: Use static mug position (no randomization)
        print("\n[1/5] Using static mug position...")
        mug_position = STATIC_MUG_POSITION
        print(f"  Static mug position: ({mug_position[0]:.3f}, {mug_position[1]:.3f}, {mug_position[2]:.3f})")
        # Mug is already positioned at initialization, no need to move it each episode
        print("  ✓ Mug position confirmed")
        
        # Step 2: Calculate goal pose
        print("\n[2/5] Calculating goal pose...")
        goal_pose = calculate_goal_pose(
            mug_position, 
            GOAL_HEIGHT_ABOVE_MUG, 
            GOAL_ORIENTATION
        )
        print("  ✓ Goal pose calculated")
        
        # Step 3: Get goal joint positions (fixed or via IK)
        print("\n[3/5] Getting goal joint positions...")
        if USE_FIXED_GOAL_JOINTS:
            goal_joints = GOAL_JOINTS_FIXED[:6]  # Use first 6 for 6-DOF control
            print("  Using fixed goal joint angles:")
            for i, angle in enumerate(goal_joints, 1):
                print(f"    joint_{i}: {angle:+.3f} rad ({np.degrees(angle):+.1f} deg)")
            print("  ✓ Using known-good configuration")
        else:
            goal_joints = calculate_ik(goal_pose, ik_client)
            if goal_joints is None:
                print("  ✗ IK failed to find solution")
                return False
            print("  ✓ IK solution found")
        
        # Step 4: Randomize starting end-effector pose and calculate IK
        print("\n[4/5] Generating random starting pose...")
        if START_POSE_RANDOMIZATION:
            # Try up to 5 times to find a reachable starting pose
            max_attempts = 5
            start_joints = None
            
            for attempt in range(1, max_attempts + 1):
                start_ee_pose = randomize_start_end_effector_pose(
                    START_POSE_WORKSPACE_CENTER,
                    START_POSE_RADIUS_MIN,
                    START_POSE_RADIUS_MAX,
                    START_POSE_Z_MIN,
                    START_POSE_Z_MAX,
                    START_POSE_ORIENTATION,
                    START_POSE_ORIENTATION_VARIATION
                )
                
                current_joints = get_current_joint_positions()
                start_joints = calculate_ik(start_ee_pose, ik_client, current_joints)
                
                if start_joints is not None:
                    print(f"  ✓ Found reachable starting pose (attempt {attempt}/{max_attempts})")
                    break
                else:
                    print(f"  ⚠️  Starting pose unreachable, retrying... (attempt {attempt}/{max_attempts})")
            
            if start_joints is None:
                print(f"  ✗ Could not find reachable starting pose after {max_attempts} attempts")
                return False
        else:
            start_joints = get_current_joint_positions()
            start_ee_pose = None
        
        # Move to starting pose (no recording)
        # Note: We accept wherever the robot ends up, even if it doesn't reach exactly
        print("  Moving toward starting pose...")
        move_to_joint_positions(start_joints)  # Try to reach it
        
        # Get actual starting position (wherever robot ended up)
        actual_start_joints = get_current_joint_positions()
        if actual_start_joints:
            print("  ✓ Using actual position as starting pose")
            start_joints = actual_start_joints  # Update to actual position
        else:
            print("  ⚠️  Could not read position, using commanded pose")
            # Continue anyway with commanded position
        
        # Step 5: Start recording and move to goal pose
        print("\n[5/5] Recording motion to goal...")
        metadata = {
            'episode': episode_num,
            'mug_position': mug_position.tolist(),
            'goal_pose': goal_pose if not USE_FIXED_GOAL_JOINTS else 'fixed_goal',
            'start_ee_pose': start_ee_pose if start_ee_pose is not None else 'current_position',
            'start_joints_actual': start_joints.tolist() if isinstance(start_joints, np.ndarray) else start_joints,  # Actual position used
            'goal_joints': goal_joints.tolist(),
            'use_fixed_goal': USE_FIXED_GOAL_JOINTS,
            'timestamp': datetime.now().isoformat()
        }
        recorder.start_episode(episode_num, metadata)
        
        # Move to goal while recording
        if not move_to_joint_positions(goal_joints):
            print("  ✗ Failed to reach goal pose")
            recorder.stop_episode()
            return False
        
        # Hold at goal pose for better data recording
        print(f"  ✓ Reached goal pose, holding for {GOAL_POSE_HOLD_TIME}s...")
        for i in range(int(GOAL_POSE_HOLD_TIME * 10)):
            update_viewport(2)
            # recorder.record_frame()  # TODO: Implement frame recording
            time.sleep(0.1)
        print("  ✓ Goal pose hold complete")
        
        recorder.stop_episode()
        print("  ✓ Motion recorded")
        
        print(f"\n✓ Episode {episode_num} completed successfully")
        return True
        
    except Exception as e:
        print(f"\n✗ Episode {episode_num} failed with error: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Main data collection pipeline."""
    print("="*60)
    print("ROBOT GRASPING DATA COLLECTION PIPELINE - STANDALONE")
    print("="*60)
    print(f"Episodes to collect: {NUM_EPISODES}")
    print(f"Save directory: {SAVE_DIR}")
    print("="*60)
    
    # Ensure simulation is playing
    print("\n[SETUP] Ensuring simulation is playing...")
    try:
        timeline = get_timeline_interface()
        if not timeline.is_playing():
            print("  ▶ Starting simulation...")
            timeline.play()
            time.sleep(1.0)
            update_viewport(10)
            print("  ✓ Simulation started")
        else:
            print("  ✓ Simulation already playing")
    except Exception as e:
        print(f"  ⚠️  Warning: Could not auto-start simulation: {e}")
    
    # Initialize mug at static position
    print("\n[SETUP] Adding mug to scene at static position...")
    print(f"  Position: ({STATIC_MUG_POSITION[0]:.3f}, {STATIC_MUG_POSITION[1]:.3f}, {STATIC_MUG_POSITION[2]:.3f})")
    mug_path = add_mug_to_scene(MUG_ASSET_URL, MUG_PRIM_PATH, position=STATIC_MUG_POSITION)
    if mug_path is None:
        print("✗ Failed to initialize mug")
        return
    print("✓ Mug ready at static position")
    
    # Initialize recorder
    print("\n[SETUP] Initializing data recorder...")
    recorder = DataRecorder(SAVE_DIR)
    print("✓ Recorder ready")
    
    # Initialize ROS2
    print("\n[SETUP] Initializing ROS2...")
    if not initialize_ros2(node_name='data_collection_node'):
        print("✗ ROS2 initialization failed")
        print("  Make sure:")
        print("  - Isaac Sim is running and publishing /joint_states")
        print("  - Simulation is playing (ActionGraphs active)")
        return
    print("✓ ROS2 ready")
    
    # Initialize IK client
    print("\n[SETUP] Initializing IK client...")
    print("  Make sure ik_service.py is running in container:")
    print("  ros2 run openpi_kinova_ros2 ik_service.py")
    
    global _ros2_node
    if _ros2_node is None:
        print("✗ ROS2 node not available")
        return
    
    ik_client = IKClient(_ros2_node)
    print("✓ IK client ready")
    
    # Brief wait for IK service
    print("  Waiting for IK service to be ready...")
    for i in range(10):
        update_viewport(1)
        time.sleep(0.1)
    
    # Data collection loop
    print("\n" + "="*60)
    print("STARTING DATA COLLECTION")
    print("="*60)
    
    successful_episodes = 0
    failed_episodes = 0
    
    for episode_num in range(1, NUM_EPISODES + 1):
        success = collect_single_episode(episode_num, mug_path, recorder, ik_client)
        
        if success:
            successful_episodes += 1
        else:
            failed_episodes += 1
            print(f"⚠️  Episode {episode_num} failed")
        
        # Reset simulation between episodes (if not last episode)
        if episode_num < NUM_EPISODES:
            print(f"\n[RESET] Resetting simulation for next episode...")
            try:
                timeline = get_timeline_interface()
                
                # Stop simulation
                if timeline.is_playing():
                    timeline.stop()
                    print("  ✓ Simulation stopped")
                    time.sleep(0.5)
                    update_viewport(5)
                
                # Restart simulation
                timeline.play()
                print("  ✓ Simulation restarted")
                time.sleep(1.0)
                update_viewport(10)
                
            except Exception as e:
                print(f"  ⚠️  Warning: Could not reset simulation: {e}")
                time.sleep(1.0)
        else:
            # Brief pause after last episode
            time.sleep(1.0)
    
    # Summary
    print("\n" + "="*60)
    print("DATA COLLECTION COMPLETE")
    print("="*60)
    print(f"Total episodes: {NUM_EPISODES}")
    print(f"Successful: {successful_episodes}")
    print(f"Failed: {failed_episodes}")
    print(f"Success rate: {100*successful_episodes/NUM_EPISODES:.1f}%")
    print(f"Data saved to: {SAVE_DIR}")
    print("="*60)
    
    # Cleanup
    print("\n[CLEANUP] Shutting down ROS2...")
    shutdown_ros2()
    print("✓ ROS2 shutdown complete")


if __name__ == "__main__":
    main()

