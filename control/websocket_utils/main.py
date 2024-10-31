import numpy as np
import roslibpy
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import base64

# Create a CvBridge object for handling the conversion between ROS and OpenCV
bridge = CvBridge()

def image_callback(message):
    print("Received compressed image from ROS")
    # message_data = message['data'].encode()

    message_data = base64.b64decode(message['data'])


    np_arr = np.fromstring(message_data, np.uint8)
    print(np_arr)

    # Convert the CompressedImage message to an OpenCV image
    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    print('cv_image:',cv_image)

    try:

        # Display the image using OpenCV
        cv2.imshow('Compressed Image', cv_image)
        cv2.waitKey(1)

    except CvBridgeError as e:
        print(f"Error converting image: {e}")
    except Exception as e:
        print(f"Error displaying image: {e}")

# Connect to the ROS WebSocket server
client = roslibpy.Ros(host='localhost', port=9090)
client.run()

# Subscribe to the ROS image topic
image_topic = roslibpy.Topic(client, '/image_topic/compressed', 'sensor_msgs/CompressedImage')
image_topic.subscribe(image_callback)

# Keep the connection alive
try:
    while True:
        pass  # Infinite loop to keep the script alive
except KeyboardInterrupt:
    print('Shutting down...')
    image_topic.unsubscribe()
    client.terminate()

