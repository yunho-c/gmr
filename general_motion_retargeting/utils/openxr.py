import json
import numpy as np
from scipy.spatial.transform import Rotation as R
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))
from References.openxr_skeletons import FullBodyBoneId

OPENXR_BONE_NAMES = {
    0: "FullBody_Root",
    1: "FullBody_Hips", 
    2: "FullBody_SpineLower",
    3: "FullBody_SpineMiddle",
    4: "FullBody_SpineUpper",
    5: "FullBody_Chest",
    6: "FullBody_Neck",
    7: "FullBody_Head",
    8: "FullBody_LeftShoulder",
    9: "FullBody_LeftScapula",
    10: "FullBody_LeftArmUpper",
    11: "FullBody_LeftArmLower", 
    12: "FullBody_LeftHandWristTwist",
    13: "FullBody_RightShoulder",
    14: "FullBody_RightScapula",
    15: "FullBody_RightArmUpper",
    16: "FullBody_RightArmLower",
    17: "FullBody_RightHandWristTwist",
    18: "FullBody_LeftHandPalm",
    19: "FullBody_LeftHandWrist",
    20: "FullBody_LeftHandThumbMetacarpal",
    21: "FullBody_LeftHandThumbProximal",
    22: "FullBody_LeftHandThumbDistal",
    23: "FullBody_LeftHandThumbTip",
    24: "FullBody_LeftHandIndexMetacarpal",
    25: "FullBody_LeftHandIndexProximal",
    26: "FullBody_LeftHandIndexIntermediate",
    27: "FullBody_LeftHandIndexDistal",
    28: "FullBody_LeftHandIndexTip",
    29: "FullBody_LeftHandMiddleMetacarpal",
    30: "FullBody_LeftHandMiddleProximal",
    31: "FullBody_LeftHandMiddleIntermediate",
    32: "FullBody_LeftHandMiddleDistal",
    33: "FullBody_LeftHandMiddleTip",
    34: "FullBody_LeftHandRingMetacarpal",
    35: "FullBody_LeftHandRingProximal",
    36: "FullBody_LeftHandRingIntermediate",
    37: "FullBody_LeftHandRingDistal",
    38: "FullBody_LeftHandRingTip",
    39: "FullBody_LeftHandLittleMetacarpal",
    40: "FullBody_LeftHandLittleProximal",
    41: "FullBody_LeftHandLittleIntermediate",
    42: "FullBody_LeftHandLittleDistal",
    43: "FullBody_LeftHandLittleTip",
    44: "FullBody_RightHandPalm",
    45: "FullBody_RightHandWrist",
    46: "FullBody_RightHandThumbMetacarpal",
    47: "FullBody_RightHandThumbProximal",
    48: "FullBody_RightHandThumbDistal",
    49: "FullBody_RightHandThumbTip",
    50: "FullBody_RightHandIndexMetacarpal",
    51: "FullBody_RightHandIndexProximal",
    52: "FullBody_RightHandIndexIntermediate",
    53: "FullBody_RightHandIndexDistal",
    54: "FullBody_RightHandIndexTip",
    55: "FullBody_RightHandMiddleMetacarpal",
    56: "FullBody_RightHandMiddleProximal",
    57: "FullBody_RightHandMiddleIntermediate",
    58: "FullBody_RightHandMiddleDistal",
    59: "FullBody_RightHandMiddleTip",
    60: "FullBody_RightHandRingMetacarpal",
    61: "FullBody_RightHandRingProximal",
    62: "FullBody_RightHandRingIntermediate",
    63: "FullBody_RightHandRingDistal",
    64: "FullBody_RightHandRingTip",
    65: "FullBody_RightHandLittleMetacarpal",
    66: "FullBody_RightHandLittleProximal",
    67: "FullBody_RightHandLittleIntermediate",
    68: "FullBody_RightHandLittleDistal",
    69: "FullBody_RightHandLittleTip",
    70: "FullBody_LeftUpperLeg",
    71: "FullBody_LeftLowerLeg",
    72: "FullBody_LeftFootAnkleTwist",
    73: "FullBody_LeftFootAnkle",
    74: "FullBody_LeftFootSubtalar",
    75: "FullBody_LeftFootTransverse",
    76: "FullBody_LeftFootBall",
    77: "FullBody_RightUpperLeg",
    78: "FullBody_RightLowerLeg",
    79: "FullBody_RightFootAnkleTwist",
    80: "FullBody_RightFootAnkle",
    81: "FullBody_RightFootSubtalar",
    82: "FullBody_RightFootTransverse",
    83: "FullBody_RightFootBall",
}

OPENXR_TO_STANDARD_MAPPING = {
    "FullBody_Root": "pelvis",
    "FullBody_Hips": "pelvis",
    "FullBody_SpineLower": "spine1", 
    "FullBody_SpineMiddle": "spine2",
    "FullBody_SpineUpper": "spine3",
    "FullBody_Chest": "spine3",
    "FullBody_Neck": "neck",
    "FullBody_Head": "head",
    "FullBody_LeftShoulder": "left_shoulder",
    "FullBody_LeftArmUpper": "left_elbow",
    "FullBody_LeftArmLower": "left_elbow",
    "FullBody_LeftHandWrist": "left_wrist",
    "FullBody_RightShoulder": "right_shoulder", 
    "FullBody_RightArmUpper": "right_elbow",
    "FullBody_RightArmLower": "right_elbow",
    "FullBody_RightHandWrist": "right_wrist",
    "FullBody_LeftUpperLeg": "left_hip",
    "FullBody_LeftLowerLeg": "left_knee",
    "FullBody_LeftFootAnkle": "left_foot",
    "FullBody_LeftFootBall": "left_foot",
    "FullBody_RightUpperLeg": "right_hip",
    "FullBody_RightLowerLeg": "right_knee", 
    "FullBody_RightFootAnkle": "right_foot",
    "FullBody_RightFootBall": "right_foot",
}


def load_openxr_file(openxr_file):
    """
    Load OpenXR full body pose data from JSONL or JSON file.
    
    Args:
        openxr_file: Path to JSONL or JSON file containing OpenXR skeletal data
    
    Returns:
        frames: List of frame dictionaries with format:
            {"BoneName": (position, orientation), ...}
        human_height: Estimated human height in meters
    """
    frames = []
    
    try:
        with open(openxr_file, 'r') as f:
            content = f.read().strip()
            
            # Try loading as single JSON object first
            if content.startswith('{') and not content.count('\n'):
                frame_data = json.loads(content)
                frames.append(_parse_openxr_frame(frame_data))
            else:
                # Handle JSONL format (line-by-line JSON)
                for line in content.split('\n'):
                    line = line.strip()
                    if not line:
                        continue
                        
                    # Handle lines with path prefix before JSON
                    json_start = line.find('{')
                    if json_start != -1:
                        line = line[json_start:]
                    else:
                        continue  # Skip non-JSON lines
                        
                    try:
                        frame_data = json.loads(line)
                        frames.append(_parse_openxr_frame(frame_data))
                    except json.JSONDecodeError as e:
                        print(f"Warning: Skipping malformed JSON line: {e}")
                        continue
                        
    except FileNotFoundError:
        raise FileNotFoundError(f"OpenXR file not found: {openxr_file}")
    except Exception as e:
        raise RuntimeError(f"Error loading OpenXR file {openxr_file}: {e}")
    
    if not frames:
        raise ValueError(f"No valid frames found in OpenXR file: {openxr_file}")
    
    # Estimate human height from head to foot
    human_height = _estimate_human_height(frames[0])
        
    return frames, human_height


def _parse_openxr_frame(frame_data):
    """Parse a single OpenXR frame from JSON data."""
    result = {}
    
    if "bones" not in frame_data:
        print("Warning: Frame missing 'bones' data")
        return result
        
    for bone_data in frame_data["bones"]:
        try:
            bone_id = bone_data["id"]
            pos_data = bone_data["pos"]
            rot_data = bone_data["rot"]
            
            position = np.array([pos_data["x"], pos_data["y"], pos_data["z"]], dtype=np.float32)
            
            # Convert XYZW quaternion to scalar-first format (WXYZ)
            orientation = np.array([rot_data["w"], rot_data["x"], rot_data["y"], rot_data["z"]], dtype=np.float32)
            
            # Normalize quaternion to handle any numerical errors
            orientation = orientation / np.linalg.norm(orientation)
            
            # Get the OpenXR bone name
            if bone_id in OPENXR_BONE_NAMES:
                openxr_name = OPENXR_BONE_NAMES[bone_id]
                
                # Map to standard bone names if available
                if openxr_name in OPENXR_TO_STANDARD_MAPPING:
                    bone_name = OPENXR_TO_STANDARD_MAPPING[openxr_name] 
                    result[bone_name] = (position, orientation)
                else:
                    # Keep original OpenXR name for unmapped bones (hands, etc.)
                    result[openxr_name] = (position, orientation)
                    
        except KeyError as e:
            print(f"Warning: Missing required bone data field {e} for bone ID {bone_data.get('id', 'unknown')}")
            continue
        except Exception as e:
            print(f"Warning: Error parsing bone ID {bone_data.get('id', 'unknown')}: {e}")
            continue
            
    return result


def _estimate_human_height(frame):
    """Estimate human height from a single frame."""
    # Try multiple combinations for height estimation
    height_candidates = []
    
    # Method 1: Head to foot
    if "head" in frame and "left_foot" in frame:
        head_pos = frame["head"][0]
        left_foot_pos = frame["left_foot"][0] 
        right_foot_pos = frame.get("right_foot", (left_foot_pos,))[0]
        
        # Use minimum foot height for ground reference
        foot_height = min(left_foot_pos[1], right_foot_pos[1])  # Y-axis is typically up in OpenXR
        height_candidates.append(abs(head_pos[1] - foot_height))
    
    # Method 2: Try Z-axis if Y doesn't work
    if "head" in frame and "left_foot" in frame:
        head_pos = frame["head"][0]
        left_foot_pos = frame["left_foot"][0]
        right_foot_pos = frame.get("right_foot", (left_foot_pos,))[0]
        
        foot_height = min(left_foot_pos[2], right_foot_pos[2])  # Z-axis alternative
        height_candidates.append(abs(head_pos[2] - foot_height))
        
    # Method 3: Pelvis to head (partial height)
    if "head" in frame and "pelvis" in frame:
        head_pos = frame["head"][0]
        pelvis_pos = frame["pelvis"][0]
        partial_height = np.linalg.norm(head_pos - pelvis_pos)
        # Estimate full height (pelvis to head is roughly 60% of total height)
        height_candidates.append(partial_height / 0.6)
    
    # Return most reasonable height (filter out extreme values)
    if height_candidates:
        # Filter heights between 1.4m and 2.2m (reasonable human range)
        valid_heights = [h for h in height_candidates if 1.4 <= h <= 2.2]
        if valid_heights:
            return np.median(valid_heights)
    
    return 1.75  # Default height in meters


def get_openxr_data_offline_fast(openxr_frames, tgt_fps=30):
    """
    Process OpenXR frames with FPS alignment similar to SMPL-X processing.
    
    Args:
        openxr_frames: List of frame dictionaries from load_openxr_file
        tgt_fps: Target frame rate for output
        
    Returns:
        processed_frames: List of processed frame dictionaries
        aligned_fps: Actual output frame rate
    """
    src_fps = 90  # OpenXR typically runs at 90Hz
    frame_skip = max(1, int(src_fps / tgt_fps))
    num_frames = len(openxr_frames)
    
    if tgt_fps < src_fps and num_frames > frame_skip:
        # Downsample by taking every N-th frame
        processed_frames = openxr_frames[::frame_skip]
        aligned_fps = len(processed_frames) / num_frames * src_fps
    else:
        # Keep all frames
        processed_frames = openxr_frames
        aligned_fps = src_fps
        
    return processed_frames, aligned_fps


def get_openxr_data(openxr_frames, curr_frame):
    """
    Get a single OpenXR frame data in the standard format.
    
    Args:
        openxr_frames: List of frame dictionaries from load_openxr_file
        curr_frame: Frame index to retrieve
        
    Returns:
        Frame dictionary: {"BoneName": (position, orientation), ...}
    """
    if curr_frame >= len(openxr_frames):
        raise IndexError(f"Frame {curr_frame} not available, only {len(openxr_frames)} frames loaded")
        
    return openxr_frames[curr_frame]


def convert_openxr_frame_to_standard(openxr_frame):
    """
    Convert a single OpenXR frame to the standard format expected by GMR.
    
    Args:
        openxr_frame: Dictionary with OpenXR bone data
        
    Returns:
        Standard format dictionary: {"BoneName": (position, orientation), ...}
    """
    # This function is already handled within load_openxr_file
    # but kept for compatibility if needed separately
    return openxr_frame