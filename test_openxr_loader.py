#!/usr/bin/env python3

import sys
import os
sys.path.insert(0, os.path.abspath('.'))

from general_motion_retargeting.utils.openxr import load_openxr_file

def test_openxr_loader():
    # Test with the sample data
    sample_file = "References/recordings/sample_body_pose_data.jsonl"
    
    if not os.path.exists(sample_file):
        print(f"Sample file not found: {sample_file}")
        return
    
    print(f"Loading OpenXR data from: {sample_file}")
    
    try:
        frames, human_height = load_openxr_file(sample_file)
        
        print(f"Successfully loaded {len(frames)} frames")
        print(f"Estimated human height: {human_height:.2f} meters")
        
        if frames:
            first_frame = frames[0]
            print(f"First frame contains {len(first_frame)} bones:")
            for bone_name, (pos, rot) in first_frame.items():
                print(f"  {bone_name}: pos={pos}, rot={rot}")
                
        print("\nTesting completed successfully!")
        
    except Exception as e:
        print(f"Error loading OpenXR data: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_openxr_loader()