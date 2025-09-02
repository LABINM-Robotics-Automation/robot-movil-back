import rosbag
import cv2
import numpy as np
from cv_bridge import CvBridge
import os
from pathlib import Path


def calculate_fps(bag_file, topic):
    """Calculate FPS from bag file timestamps"""
    timestamps = []
    bag = rosbag.Bag(bag_file, "r")
    for _, _, t in bag.read_messages(topics=[topic]):
        timestamps.append(t.to_sec())
    bag.close()

    if len(timestamps) < 2:
        return 10  # fallback
    duration = timestamps[-1] - timestamps[0]
    return len(timestamps) / duration if duration > 0 else 10


def convert_bag_to_mp4(bag_file_path, output_video_path, topic="/zed2i/zed_node/rgb_raw/image_raw_color", fps=None):
    """
    Convert ROS bag file to MP4 video
    
    Args:
        bag_file_path (str): Path to input bag file
        output_video_path (str): Path for output MP4 file
        topic (str): ROS topic to extract images from
        fps (float): Frames per second for output video. If None, auto-calculated
    
    Returns:
        dict: Result with success status and message
    """
    try:
        bridge = CvBridge()
        
        # Check if bag file exists
        if not os.path.exists(bag_file_path):
            return {"success": False, "message": f"Bag file not found: {bag_file_path}"}
        
        # Create output directory if it doesn't exist
        output_dir = os.path.dirname(output_video_path)
        Path(output_dir).mkdir(parents=True, exist_ok=True)
        
        # Calculate FPS if not provided
        if fps is None:
            fps = calculate_fps(bag_file_path, topic)
        
        bag = rosbag.Bag(bag_file_path, "r")
        
        # Get frame size from first message
        width = height = None
        for _, msg, _ in bag.read_messages(topics=[topic]):
            if msg._type == "sensor_msgs/CompressedImage":
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            elif msg._type == "sensor_msgs/Image":
                cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            else:
                continue
                
            if cv_image is not None:
                height, width, _ = cv_image.shape
                break
        
        if width is None or height is None:
            bag.close()
            return {"success": False, "message": "No valid frames found in bag file"}
        
        # Count total frames for progress tracking
        total_frames = sum(1 for _ in bag.read_messages(topics=[topic]))
        bag.close()
        
        if total_frames == 0:
            return {"success": False, "message": f"No messages found for topic: {topic}"}
        
        # Initialize video writer
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        out = cv2.VideoWriter(output_video_path, fourcc, fps, (width, height))
        
        # Process frames
        bag = rosbag.Bag(bag_file_path, "r")
        frame_count = 0
        
        for _, msg, _ in bag.read_messages(topics=[topic]):
            if msg._type == "sensor_msgs/CompressedImage":
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            elif msg._type == "sensor_msgs/Image":
                cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            else:
                continue
            
            if cv_image is not None:
                out.write(cv_image)
                frame_count += 1
        
        bag.close()
        out.release()
        
        return {
            "success": True, 
            "message": f"Successfully converted {frame_count} frames to {output_video_path}",
            "frames_processed": frame_count,
            "fps": fps,
            "resolution": f"{width}x{height}"
        }
        
    except Exception as e:
        return {"success": False, "message": f"Error during conversion: {str(e)}"}