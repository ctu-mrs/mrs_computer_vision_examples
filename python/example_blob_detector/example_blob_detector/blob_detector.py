#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

import cv2
from cv_bridge import CvBridge

from PIL import Image as PilImage
from PIL import ImageFilter, ImageDraw, ImageFont

class BlobDetector(Node):

    def __init__(self):

        super().__init__('blob_detector')

        self.cbkgr_ss = MutuallyExclusiveCallbackGroup() # for service servers

        ## | --------------------- load parameters -------------------- |

        self.declare_parameter("min_area", 0)
        self.declare_parameter("max_area", 0)

        self._min_area_ = self.get_parameter("min_area").value
        self._max_area_ = self.get_parameter("max_area").value

        ## | ----------------------- subscribers ---------------------- |

        self.sub_image_ = self.create_subscription(Image, "~/image_raw_in", self.callbackImage, qos_profile_sensor_data, callback_group=self.cbkgr_ss)
        self.sub_cam_info_ = self.create_subscription(CameraInfo, "~/camera_info_in", self.callbackCameraInfo, qos_profile_sensor_data, callback_group=self.cbkgr_ss)

        self.got_camera_info_ = False

        ## | ----------------------- publishers ----------------------- |

        self.pub_image_ = self.create_publisher(Image, "~/image_raw_out", qos_profile_sensor_data)
        self.pub_cam_info_ = self.create_publisher(CameraInfo, "~/camera_info_out", qos_profile_sensor_data)

        ## | ---------------------- opencv bridge --------------------- |

        self.bridge = CvBridge()

        ## | --------------------- finish the init -------------------- |

        self.is_initialized_ = True

        self.get_logger().info('initialized')

    # #} end of __init__()

    ## | ------------------------ callbacks ----------------------- |

    # #{ callbackImage()

    def callbackImage(self, msg):

        if not self.is_initialized_:
            return

        self.get_logger().info('getting images', once=True)

        if not self.got_camera_info_:
            return

        original_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        ## | -------- do some image modifications using pillow -------- |

        # convert opencv image to pillow image
        cv_rgb = cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB)
        pil_img = PilImage.fromarray(cv_rgb)

        blurred = pil_img.filter(ImageFilter.GaussianBlur(3))
        draw = ImageDraw.Draw(blurred)
        draw.ellipse((20, 20, 100, 200), fill="blue", width=5)
        draw.ellipse((300, 300, 500, 400), fill="green", width=5)

        # convert back from pillow to opencv
        cv_back = cv2.cvtColor(np.array(blurred), cv2.COLOR_RGB2BGR)

        ## | -------------- prepare opencv blob detector -------------- |

        # Adjust detection parameters
        params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        params.minThreshold = 0;
        params.maxThreshold = 100;

        # Filter by Area.
        params.filterByArea = True
        params.minArea = self._min_area_
        params.maxArea = self._max_area_

        # Filter by Circularity
        params.filterByCircularity = False
        params.minCircularity = 0.1

        # Filter by Convexity
        params.filterByConvexity = False
        params.minConvexity = 0.5

        # Filter by Inertia
        params.filterByInertia = False
        params.minInertiaRatio = 0.5

        # Set up the detector with default parameters.
        detector = cv2.SimpleBlobDetector_create(params)

        # Detect blobs.
        keypoints = detector.detect(cv_back)

        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(cv_back, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # publish the new image with the camera info
        self.pub_cam_info_.publish(self.camera_info_)

        image_message = self.bridge.cv2_to_imgmsg(im_with_keypoints, encoding="bgr8")
        image_message.header = msg.header

        self.pub_image_.publish(image_message)

    # #} end of callbackImage()

    # #{ callbackCameraInfo()

    def callbackCameraInfo(self, msg):

        if not self.is_initialized_:
            return

        self.get_logger().info('getting camera info', once=True)

        self.camera_info_ = msg

        self.got_camera_info_ = True

    # #} end of callbackImage()

def main(args=None):

    rclpy.init(args=args)

    node = BlobDetector()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
