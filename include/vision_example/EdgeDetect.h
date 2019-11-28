#pragma once
#ifndef VISION_EXAMPLE_EDGE_DETECT_H
#define VISION_EXAMPLE_EDGE_DETECT_H

/* includes //{ */

/* each ROS nodelet must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

/* TF2 related ROS includes */
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>

/* camera image messages */
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

/* long unsigned integer message */
#include <std_msgs/UInt64.h>

/* some STL includes */
#include <stdlib.h>
#include <stdio.h>
#include <mutex>

/* some OpenCV includes */
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

/* ROS includes for working with OpenCV and images */
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

/* custom helper functions from our library */
#include <mrs_lib/ParamLoader.h>

//}

namespace vision_example
{

/* class EdgeDetect //{ */
class EdgeDetect : public nodelet::Nodelet {

public:
  /* onInit() is called when nodelet is launched (similar to main() in regular node) */
  virtual void onInit();

private:
  /* flags */
  bool is_initialized_ = false;

  /* ros parameters */
  bool _gui_ = true;

  std::string _uav_name_;

  // | ---------------------- msg callbacks --------------------- |

  void                        callbackImage(const sensor_msgs::ImageConstPtr& msg);
  image_transport::Subscriber sub_image_;

  void                               callbackCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg);
  ros::Subscriber                    sub_camera_info_;
  image_geometry::PinholeCameraModel camera_model_;

  // | --------------------- timer callbacks -------------------- |

  void       callbackTimerCheckSubscribers(const ros::TimerEvent& te);
  ros::Timer timer_check_subscribers_;
  int        _rate_timer_check_subscribers_;

  // | -------------------- image processing -------------------- |

  static cv::Mat detectEdgesCanny(cv::InputArray image, int low_threshold);
  static void    showEdgeImage(cv::InputArray image, cv::InputArray detected_edges);
  cv::Mat        projectWorldPointToImage(cv::InputArray image, const ros::Time& image_stamp, const double x, const double y, const double z);

  // | --------- variables, related to message checking --------- |

  std::mutex mutex_counters_;           // to prevent data races when accessing the following variables from multiple threads
  ros::Time  time_last_image_;          // time stamp of the last received image message
  ros::Time  time_last_camera_info_;    // time stamp of the last received camera info message
  uint64_t   image_counter_   = 0;      // counts the number of images received
  bool       got_image_       = false;  // indicates whether at least one image message was received
  bool       got_camera_info_ = false;  // indicates whether at least one camera info message was received

  // | --------------- variables for edge detector -------------- |

  int       low_threshold_;
  int const max_low_threshold_ = 100;

  // | ------------- variables for point projection ------------- |
  std::string                                 world_frame_id_;
  double                                      world_point_x_;
  double                                      world_point_y_;
  double                                      world_point_z_;
  tf2_ros::Buffer                             tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;

  // | ----------------------- publishers ----------------------- |

  ros::Publisher             pub_test_;
  image_transport::Publisher pub_edges_;
  image_transport::Publisher pub_projection_;
  int                        _rate_timer_publish_;

  // | --------------------- other functions -------------------- |

  void publishOpenCVImage(cv::InputArray detected_edges, const std_msgs::Header& header, const std::string& encoding, const image_transport::Publisher& pub);
  void publishImageNumber(uint64_t count);

  bool transformPointFromWorld(const geometry_msgs::Point& point, const std::string& to_frame, const ros::Time& stamp, geometry_msgs::Point& point_out);
  bool getTransform(const std::string& from_frame, const std::string& to_frame, const ros::Time& stamp, geometry_msgs::TransformStamped& transform_out);
};
//}

}  // namespace vision_example
#endif
