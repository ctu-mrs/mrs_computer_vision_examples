#!/usr/bin/python3

import rospy
import numpy as np

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from cv_bridge import CvBridge, CvBridgeError
import cv2

class Node:

    # #{ __init__(self)

    def __init__(self):

        rospy.init_node("blob_detector", anonymous=True)

        ## | --------------- initialize member variable --------------- |

        self.camera_info = False

        ## | --------------------- load parameters -------------------- |

        # self.frame_id = rospy.get_param("~frame_id")

        ## | ----------------------- subscribers ---------------------- |

        self.sub_image = rospy.Subscriber("~image_raw_in", Image, self.callbackImage)
        self.sub_image_info = rospy.Subscriber("~camera_info_in", CameraInfo, self.callbackCameraInfo)

        ## | ----------------------- publishers ----------------------- |

        self.publisher_image = rospy.Publisher("~image_raw_out", Image, queue_size=1)
        self.publisher_cam_info = rospy.Publisher("~camera_info_out", CameraInfo, queue_size=1)

        ## | ---------------------- OpenCV Bridge --------------------- |

        self.bridge = CvBridge()

        ## | -------------------- spin till the end ------------------- |

        self.is_initialized = True

        rospy.loginfo('[BlobDetector]: initialized')

        rospy.spin()

    # #} end of __init__()

    ## | ------------------------ callbacks ----------------------- |

    # #{ callbackImage()

    def callbackImage(self, msg):

        if not self.is_initialized:
            return

        if not isinstance(self.camera_info, CameraInfo):
            rospy.logwarn_throttle(1.0, '[BlobDetector]: waiting for camera info')
            return

        rospy.loginfo_once('[BlobDetector]: getting images')

        original_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Adjust detection parameters
        params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        params.minThreshold = 0;
        params.maxThreshold = 100;

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 400
        params.maxArea = 20000

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
        keypoints = detector.detect(original_image)

        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(original_image, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # publish the new image with the camera info
        self.publisher_cam_info.publish(self.camera_info)

        image_message = self.bridge.cv2_to_imgmsg(im_with_keypoints, encoding="bgr8")
        image_message.header = msg.header

        self.publisher_image.publish(image_message)

    # #} end of callbackImage()

    # #{ callbackCameraInfo()

    def callbackCameraInfo(self, msg):

        if not self.is_initialized:
            return

        rospy.loginfo_once('[BlobDetector]: getting camera info')
        self.camera_info = msg

    # #} end of callbackCameraInfo()

if __name__ == '__main__':
    try:
        node = Node()
    except rospy.ROSInterruptException:
        pass
