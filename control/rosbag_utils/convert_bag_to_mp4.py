import sys
import os
import rosbag
import cv2
import numpy as np
from cv_bridge import CvBridge

def calculate_fps(bag_file, topic):
    timestamps = []
    bag = rosbag.Bag(bag_file, "r")
    for _, _, t in bag.read_messages(topics=[topic]):
        timestamps.append(t.to_sec())
    bag.close()
    if len(timestamps) < 2:
        return 10
    duration = timestamps[-1] - timestamps[0]
    return len(timestamps) / duration if duration > 0 else 10

def main():
    if len(sys.argv) != 5 or sys.argv[1] != '--bag-file' or sys.argv[3] != '--output-file':
        print("Usage: python convert_bag_to_mp4.py --bag-file <bag_path> --output-file <mp4_path>")
        sys.exit(1)

    bag_file = sys.argv[2]
    output_file = sys.argv[4]
    topic = "/zed2i/zed_node/rgb_raw/image_raw_color"

    try:
        bridge = CvBridge()
        fps = calculate_fps(bag_file, topic)

        bag = rosbag.Bag(bag_file, "r")

        # Get frame size
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
            print("ERROR: No valid frames found")
            sys.exit(1)

        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        out = cv2.VideoWriter(output_file, fourcc, fps, (width, height))

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

        print(f"SUCCESS: Converted {frame_count} frames to {output_file}")
        sys.exit(0)

    except Exception as e:
        print(f"ERROR: {str(e)}")
        sys.exit(1)

if __name__ == "__main__":
    main()
