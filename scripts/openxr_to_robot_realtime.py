import argparse
import time
import threading
from functools import partial
from typing import Any
import queue
import numpy as np

from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting import RobotMotionViewer
from general_motion_retargeting.utils.openxr import _parse_openxr_frame, _estimate_human_height

# Import your OpenXR library components
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'References', 'XR-Robot-Teleop-Server'))

try:
    from xr_robot_teleop_server.schemas.body_pose import deserialize_pose_data
    from xr_robot_teleop_server.schemas.openxr_skeletons import FullBodyBoneId, SkeletonType, get_bone_label
    from xr_robot_teleop_server.streaming import WebRTCServer
    OPENXR_AVAILABLE = True
except ImportError as e:
    print(f"OpenXR library not available: {e}")
    print("Please install the XR-Robot-Teleop-Server library or check the path")
    OPENXR_AVAILABLE = False

# Global data queue for pose data
pose_data_queue = queue.Queue(maxsize=10)  # Limit queue size to prevent memory buildup

def convert_openxr_pose_to_gmr_format(pose_data):
    """
    Convert your OpenXR pose data format to GMR's expected format.
    
    Args:
        pose_data: List of bone objects from deserialize_pose_data
        
    Returns:
        Dictionary in format: {"BoneName": (position, orientation), ...}
    """
    # Create a frame data structure similar to what _parse_openxr_frame expects
    frame_data = {"bones": []}
    
    for bone in pose_data:
        bone_entry = {
            "id": bone.id,
            "pos": {"x": bone.position[0], "y": bone.position[1], "z": bone.position[2]},
            "rot": {"w": bone.rotation[0], "x": bone.rotation[1], 
                   "y": bone.rotation[2], "z": bone.rotation[3]}
        }
        frame_data["bones"].append(bone_entry)
    
    # Use existing GMR parsing logic
    return _parse_openxr_frame(frame_data)

class AppState:
    def __init__(self, retargeter=None, viewer=None):
        self.retargeter = retargeter
        self.viewer = viewer
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.fps_display_interval = 2.0

def on_body_pose_message(message: bytes, state: AppState):
    """Handle incoming OpenXR body pose data from your WebRTC server."""
    try:
        if isinstance(message, bytes):
            # Convert Unity coordinates (z_up=True matches your example)
            pose_data = deserialize_pose_data(message, z_up=True)
            # pose_data = deserialize_pose_data(message, z_up=False)
            
            # Convert to GMR format
            gmr_frame = convert_openxr_pose_to_gmr_format(pose_data)
            
            # Add to queue (non-blocking, drop old frames if queue is full)
            try:
                pose_data_queue.put_nowait(gmr_frame)
            except queue.Full:
                # Remove oldest frame and add new one
                try:
                    pose_data_queue.get_nowait()
                    pose_data_queue.put_nowait(gmr_frame)
                except queue.Empty:
                    pass
                    
    except Exception as e:
        print(f"Error processing body pose data: {e}")

def estimate_height_from_pose_data(pose_data):
    """Estimate human height from a single pose frame."""
    gmr_frame = convert_openxr_pose_to_gmr_format(pose_data)
    return _estimate_human_height(gmr_frame)

def main(args):
    if not OPENXR_AVAILABLE:
        print("OpenXR library not available. Please check installation.")
        return
    
    # Validate CLI args
    if args.position_only and args.rotation_only:
        print("ERROR: Cannot use both --position-only and --rotation-only at the same time")
        return
        
    debug_mode = ""
    if args.position_only:
        debug_mode = " (POSITION-ONLY MODE)"
    elif args.rotation_only:
        debug_mode = " (ROTATION-ONLY MODE)"
        
    print(f"Starting OpenXR real-time motion retargeting{debug_mode}...")
    print("Waiting for OpenXR connection...")
    
    # Initialize retargeting system (will be set up once we get first frame)
    retargeter = None
    viewer = None
    
    # Set up the WebRTC server with data handlers
    state = AppState()
    data_handlers = {
        "body_pose": partial(on_body_pose_message, state=state),
    }
    
    server = WebRTCServer(
        datachannel_handlers=data_handlers,
        state_factory=lambda: state,
    )
    
    # Start WebRTC server in a separate thread
    server_thread = threading.Thread(target=server.run, daemon=True)
    server_thread.start()
    
    print("WebRTC server started. Connect your VR headset...")
    
    # Wait for first frame to initialize the system
    first_frame_received = False
    fps_counter = 0
    fps_start_time = time.time()
    
    try:
        while True:
            try:
                # Get latest pose data (non-blocking)
                gmr_frame = pose_data_queue.get(timeout=0.1)
                
                # Initialize system on first frame
                if not first_frame_received:
                    print("First frame received, initializing retargeting system...")
                    
                    # Estimate human height from first frame
                    human_height = _estimate_human_height(gmr_frame)
                    print(f"Estimated human height: {human_height:.2f}m")
                    
                    # Initialize retargeting
                    retargeter = GMR(
                        src_human="openxr",
                        tgt_robot=args.robot,
                        actual_human_height=human_height,
                    )
                    
                    # Apply debugging weight overrides
                    if args.position_only:
                        print("Applying POSITION-ONLY constraints...")
                        for task in retargeter.tasks1 + retargeter.tasks2:
                            task.orientation_cost = 0
                    elif args.rotation_only:
                        print("Applying ROTATION-ONLY constraints...")
                        for task in retargeter.tasks1 + retargeter.tasks2:
                            task.position_cost = 0
                    
                    # Initialize viewer
                    viewer = RobotMotionViewer(
                        robot_type=args.robot,
                        motion_fps=60,  # Target 60 FPS for real-time
                        transparent_robot=0,
                    )
                    
                    state.retargeter = retargeter
                    state.viewer = viewer
                    first_frame_received = True
                    print("System initialized. Starting real-time retargeting...")
                
                if retargeter and viewer:
                    # Perform motion retargeting
                    qpos = retargeter.retarget(gmr_frame)
                    
                    # Visualize
                    viewer.step(
                        root_pos=qpos[:3],
                        root_rot=qpos[3:7],
                        dof_pos=qpos[7:],
                        rate_limit=False,  # No rate limiting for real-time
                    )
                    
                    # FPS tracking
                    fps_counter += 1
                    current_time = time.time()
                    if current_time - fps_start_time >= 2.0:  # Display every 2 seconds
                        actual_fps = fps_counter / (current_time - fps_start_time)
                        print(f"Retargeting FPS: {actual_fps:.1f}")
                        fps_counter = 0
                        fps_start_time = current_time
                        
            except queue.Empty:
                # No new data, continue waiting
                continue
                
    except KeyboardInterrupt:
        print("\nShutting down...")
        if viewer:
            viewer.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="OpenXR Real-time Motion Retargeting")
    parser.add_argument(
        "--robot",
        choices=["unitree_g1", "unitree_h1", "booster_t1", "stanford_toddy", "fourier_n1", 
                "engineai_pm01", "kuavo_s45", "hightorque_hi", "galaxea_r1pro", "berkeley_humanoid_lite", "booster_k1"],
        default="unitree_g1",
        help="Target robot for motion retargeting"
    )
    parser.add_argument(
        "--position-only",
        action="store_true",
        help="Use only position constraints for retargeting (ignore rotations)"
    )
    parser.add_argument(
        "--rotation-only", 
        action="store_true",
        help="Use only rotation constraints for retargeting (ignore positions)"
    )
    
    args = parser.parse_args()
    main(args)