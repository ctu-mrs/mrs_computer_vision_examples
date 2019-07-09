#pragma once
#ifndef VISION_EXAMPLE_EDGE_DETECT_H
#define VISION_EXAMPLE_EDGE_DETECT_H

/* includes //{ */

/* each ros package must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

/* camera image messages */
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

/* long unsigned integer message */
#include <std_msgs/UInt64.h>

/* some std includes */
#include <stdlib.h>
#include <stdio.h>
#include <mutex>

/* some opencv includes */
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

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
  bool is_initialized_  = false;
  bool got_image_       = false;
  bool got_camera_info_ = false;

  /* ros parameters */
  bool _gui_ = true;

  // | ---------------------- msg callbacks --------------------- |

  void                        callbackImage(const sensor_msgs::ImageConstPtr &msg);
  image_transport::Subscriber sub_image_;
  cv::Mat                     image_;
  std::mutex                  mutex_image_;
  cv_bridge::CvImage          cv_ptr_;
  ros::Time                   time_last_image_;


  void            callbackCameraInfo(const sensor_msgs::CameraInfoConstPtr &msg);
  ros::Subscriber sub_camera_info_;
  ros::Time       time_last_camera_info_;

  // | --------------------- timer callbacks -------------------- |

  void       callbackTimerCheckSubscribers(const ros::TimerEvent &te);
  ros::Timer timer_check_subscribers_;
  int        _rate_timer_check_subscribers_;

  // | -------------------- image processing -------------------- |
  //
  void doSimpleImageProcessing(int, void *);

  /* increment as images comes in */
  uint64_t image_counter_ = 0;

  /* for edge detector */
  int       low_threshold_;
  int const max_low_threshold_ = 100;

  // | ----------------------- publishers ----------------------- |
  ros::Publisher pub_test_;
  int            _rate_timer_publish_;

  // | --------------------- other functions -------------------- |
  void publishImageNumber(uint64_t count);
};
//}

}  // namespace vision_example
#endif
