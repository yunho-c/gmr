#!/usr/bin/env python3
"""
Quick test script for the profiling functionality.
Creates a simple motion and tests the timing collection.
"""

import numpy as np
from general_motion_retargeting import GeneralMotionRetargeting as GMR

def create_simple_test_motion():
    """Create a simple test motion with human body poses."""
    # Simple test motion matching the SMPL-X body names from the config
    test_motion = {
        "pelvis": [
            np.array([0.0, 0.0, 1.0]),  # position
            np.array([1.0, 0.0, 0.0, 0.0])  # quaternion (w,x,y,z)
        ],
        "left_hip": [
            np.array([-0.1, 0.0, 0.9]),
            np.array([1.0, 0.0, 0.0, 0.0])
        ],
        "right_hip": [
            np.array([0.1, 0.0, 0.9]), 
            np.array([1.0, 0.0, 0.0, 0.0])
        ],
        "left_knee": [
            np.array([-0.1, 0.0, 0.5]),
            np.array([1.0, 0.0, 0.0, 0.0])
        ],
        "right_knee": [
            np.array([0.1, 0.0, 0.5]),
            np.array([1.0, 0.0, 0.0, 0.0])
        ],
        "left_foot": [
            np.array([-0.1, 0.0, 0.1]),
            np.array([1.0, 0.0, 0.0, 0.0])
        ],
        "right_foot": [
            np.array([0.1, 0.0, 0.1]),
            np.array([1.0, 0.0, 0.0, 0.0])
        ],
        "spine3": [
            np.array([0.0, 0.0, 1.2]),
            np.array([1.0, 0.0, 0.0, 0.0])
        ],
        "left_shoulder": [
            np.array([-0.2, 0.0, 1.3]),
            np.array([1.0, 0.0, 0.0, 0.0])
        ],
        "right_shoulder": [
            np.array([0.2, 0.0, 1.3]),
            np.array([1.0, 0.0, 0.0, 0.0])
        ],
        "left_elbow": [
            np.array([-0.4, 0.0, 1.2]),
            np.array([1.0, 0.0, 0.0, 0.0])
        ],
        "right_elbow": [
            np.array([0.4, 0.0, 1.2]),
            np.array([1.0, 0.0, 0.0, 0.0])
        ],
        "left_wrist": [
            np.array([-0.6, 0.0, 1.1]),
            np.array([1.0, 0.0, 0.0, 0.0])
        ],
        "right_wrist": [
            np.array([0.6, 0.0, 1.1]),
            np.array([1.0, 0.0, 0.0, 0.0])
        ]
    }
    return test_motion

if __name__ == "__main__":
    print("Testing profiling functionality...")
    
    # Initialize retargeting for G1 robot
    try:
        retarget = GMR(
            src_human="smplx",
            tgt_robot="unitree_g1",
            verbose=False
        )
        print("✓ GMR initialized successfully")
    except Exception as e:
        print(f"✗ Failed to initialize GMR: {e}")
        exit(1)
    
    # Test with a few frames
    test_motion = create_simple_test_motion()
    num_test_frames = 10
    
    print(f"Running {num_test_frames} test frames...")
    for i in range(num_test_frames):
        try:
            # Slightly vary the motion
            varied_motion = {}
            for body_name, (pos, rot) in test_motion.items():
                varied_pos = pos + np.random.normal(0, 0.01, 3)
                varied_motion[body_name] = [varied_pos, rot]
            
            qpos = retarget.retarget(varied_motion)
            print(f"  Frame {i+1}: qpos shape {qpos.shape}")
        except Exception as e:
            print(f"✗ Error on frame {i+1}: {e}")
            break
    
    # Print profiling results
    print("\n" + "="*50)
    print("PROFILING TEST RESULTS")
    print("="*50)
    retarget.print_profiling_stats()
    
    print("\n✓ Profiling test completed!")