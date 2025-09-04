import argparse
import pathlib
import os
import time

import numpy as np

from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting import RobotMotionViewer
from general_motion_retargeting.utils.openxr import load_openxr_file, get_openxr_data_offline_fast

from rich import print

if __name__ == "__main__":
    
    HERE = pathlib.Path(__file__).parent

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--openxr_file",
        help="OpenXR motion file to load (JSON or JSONL format).",
        type=str,
        required=True,
    )
    
    parser.add_argument(
        "--robot",
        choices=["unitree_g1", "unitree_g1_with_hands", "unitree_h1", "booster_t1", "stanford_toddy", "fourier_n1", 
                "engineai_pm01", "kuavo_s45", "hightorque_hi", "galaxea_r1pro", "berkeley_humanoid_lite", "booster_k1"],
        default="unitree_g1",
    )
    
    parser.add_argument(
        "--save_path",
        default=None,
        help="Path to save the robot motion.",
    )
    
    parser.add_argument(
        "--loop",
        default=False,
        action="store_true",
        help="Loop the motion.",
    )

    parser.add_argument(
        "--record_video",
        default=False,
        action="store_true",
        help="Record the video.",
    )

    parser.add_argument(
        "--rate_limit",
        default=False,
        action="store_true",
        help="Limit the rate of the retargeted robot motion to keep the same as the human motion.",
    )

    parser.add_argument(
        "--video_path",
        default=None,
        help="Path to save the video (if not provided, auto-generated).",
    )

    args = parser.parse_args()

    
    # Load OpenXR trajectory
    print(f"Loading OpenXR data from: {args.openxr_file}")
    try:
        openxr_frames, actual_human_height = load_openxr_file(args.openxr_file)
        print(f"Loaded {len(openxr_frames)} frames, estimated human height: {actual_human_height:.2f}m")
    except Exception as e:
        print(f"Error loading OpenXR file: {e}")
        exit(1)
    
    # align fps
    tgt_fps = 30
    openxr_data_frames, aligned_fps = get_openxr_data_offline_fast(openxr_frames, tgt_fps=tgt_fps)
    print(f"Aligned FPS: {aligned_fps:.2f}, frames: {len(openxr_data_frames)}")
    
   
    # Initialize the retargeting system
    print(f"Initializing retargeting system for robot: {args.robot}")
    retarget = GMR(
        actual_human_height=actual_human_height,
        src_human="openxr",
        tgt_robot=args.robot,
    )
    
    # Set up video path
    video_path = args.video_path
    if args.record_video and video_path is None:
        filename = os.path.basename(args.openxr_file).split('.')[0]
        video_path = f"videos/{args.robot}_{filename}.mp4"
    
    robot_motion_viewer = RobotMotionViewer(
        robot_type=args.robot,
        motion_fps=aligned_fps,
        transparent_robot=0,
        record_video=args.record_video,
        video_path=video_path,
    )

    curr_frame = 0
    # FPS measurement variables
    fps_counter = 0
    fps_start_time = time.time()
    fps_display_interval = 2.0  # Display FPS every 2 seconds
    
    if args.save_path is not None:
        save_dir = os.path.dirname(args.save_path)
        if save_dir:  # Only create directory if it's not empty
            os.makedirs(save_dir, exist_ok=True)
        qpos_list = []
    
    # Start the viewer
    i = 0
    print("Starting motion retargeting and visualization...")

    while True:
        if args.loop:
            i = (i + 1) % len(openxr_data_frames)
        else:
            i += 1
            if i >= len(openxr_data_frames):
                break
        
        # FPS measurement
        fps_counter += 1
        current_time = time.time()
        if current_time - fps_start_time >= fps_display_interval:
            actual_fps = fps_counter / (current_time - fps_start_time)
            print(f"Actual rendering FPS: {actual_fps:.2f}")
            fps_counter = 0
            fps_start_time = current_time
        
        # Update task targets.
        openxr_data = openxr_data_frames[i]

        # retarget
        qpos = retarget.retarget(openxr_data)

        # visualize
        robot_motion_viewer.step(
            root_pos=qpos[:3],
            root_rot=qpos[3:7],
            dof_pos=qpos[7:],
            human_motion_data=retarget.scaled_human_data,
            human_pos_offset=np.array([0.0, 0.0, 0.0]),
            show_human_body_name=False,
            rate_limit=args.rate_limit,
        )
        if args.save_path is not None:
            qpos_list.append(qpos)
            
    if args.save_path is not None:
        print(f"Saving motion data to: {args.save_path}")
        import pickle
        root_pos = np.array([qpos[:3] for qpos in qpos_list])
        # save from wxyz to xyzw
        root_rot = np.array([qpos[3:7][[1,2,3,0]] for qpos in qpos_list])
        dof_pos = np.array([qpos[7:] for qpos in qpos_list])
        local_body_pos = None
        body_names = None
        
        motion_data = {
            "fps": aligned_fps,
            "root_pos": root_pos,
            "root_rot": root_rot,
            "dof_pos": dof_pos,
            "local_body_pos": local_body_pos,
            "link_body_list": body_names,
        }
        with open(args.save_path, "wb") as f:
            pickle.dump(motion_data, f)
        print(f"Saved motion data to {args.save_path}")
            
    print("Motion retargeting completed.")
    robot_motion_viewer.close()