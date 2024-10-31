# import rospy
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
from django.http import HttpResponse
from django.views import View
import cv2
import numpy as np

# bridge = CvBridge()
#
# class ZedImageView(View):
#     def get(self, request, *args, **kwargs):
#         # Initialize the ROS node (if not already initialized)
#         # if not rospy.get_node_uri():
#         #     rospy.init_node('zed_image_view', anonymous=True)
#         #
#         # Capture a frame from the ZED camera topic
#         img_msg = rospy.wait_for_message("/zed2i/zed_node/rgb/image_rect_color", Image)
#         img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
#
#         # Convert the image to JPEG format
#         success, encoded_image = cv2.imencode('.png', img)
#        
#         if success:
#             # Convert to bytes and send as HTTP response
#             response = HttpResponse(encoded_image.tobytes(), content_type='image/jpeg')
#             return response
#         else:
#             return HttpResponse("Error: Unable to encode image", status=500)
#
#         png_image_bytes = png_image.tobytes()
#
#         # Serve the image as an HTTP response
#         return HttpResponse(png_image_bytes, content_type="image/png")
#
