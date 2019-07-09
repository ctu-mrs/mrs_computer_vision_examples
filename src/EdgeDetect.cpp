#include "EdgeDetect.h"

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>

namespace vision_example
{

/* onInit() //{ */

void EdgeDetect::onInit() {

  // | ---------------- set my booleans to false ---------------- |
  // but remember, always set them to their default value in the header file
  // because, when you add new one later, you might forger to come back here
  got_image_       = false;
  got_camera_info_ = false;

  /* obtain node handle */
  ros::NodeHandle nh("~");

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  // | ------------------- load ros parameters ------------------ |
  /* (mrs_lib implementation checks whether the parameter was loaded or not) */
  mrs_lib::ParamLoader param_loader(nh, "EdgeDetect");
  param_loader.load_param("gui", _gui_);
  param_loader.load_param("rate/publish", _rate_timer_publish_);
  param_loader.load_param("rate/check_subscribers", _rate_timer_check_subscribers_);

  /* create windows */
  if (_gui_) {

    cv::namedWindow("original", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("edges", CV_WINDOW_AUTOSIZE);

    /* Create a Trackbar for user to enter threshold */
    cv::createTrackbar("Min Threshold:", "edges", &low_threshold_, max_low_threshold_);
  }

  /* initialize the image transport, needs node handle */
  image_transport::ImageTransport it(nh);

  // | ----------------- initialize subscribers ----------------- |
  sub_image_       = it.subscribe("image_in", 1, &EdgeDetect::callbackImage, this);
  sub_camera_info_ = nh.subscribe("camera_info_in", 1, &EdgeDetect::callbackCameraInfo, this, ros::TransportHints().tcpNoDelay());

  // | ------------------ initialize publishers ----------------- |
  pub_test_ = nh.advertise<std_msgs::UInt64>("test_publisher", 1);

  // | -------------------- initialize timers ------------------- |
  timer_check_subscribers_ = nh.createTimer(ros::Rate(_rate_timer_check_subscribers_), &EdgeDetect::callbackTimerCheckSubscribers, this);

  ROS_INFO_ONCE("[EdgeDetect]: initialized");

  is_initialized_ = true;
}

//}

// | ---------------------- msg callbacks --------------------- |

/* callbackCameraInfo() //{ */

void EdgeDetect::callbackCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg) {

  if (!is_initialized_)
    return;

  got_camera_info_ = true;
  time_last_camera_info_ = ros::Time::now();

  sensor_msgs::CameraInfo info = *msg;

  ROS_INFO_STREAM_THROTTLE(1, "[EdgeDetect]: header " << info.header);
}

//}

/* callbackImage() //{ */

void EdgeDetect::callbackImage(const sensor_msgs::ImageConstPtr& msg) {

  if (!is_initialized_)
    return;

  got_image_ = true;
  image_counter_++;
  time_last_image_ = ros::Time::now();

  {
    std::scoped_lock lock(mutex_image_);

    image_ = cv_bridge::toCvShare(msg, "bgr8")->image;
  }

  /* show the image in gui */
  if (_gui_) {

    {
      std::scoped_lock lock(mutex_image_);
      cv::imshow("original", image_);
    }

    /* needed to correctly show the image */
    cv::waitKey(1);
  }

  /* output a text about it */
  ROS_INFO_THROTTLE(1, "[EdgeDetect]: Total of %u images received so far", (unsigned int)image_counter_);

  /* find edges in the image */
  EdgeDetect::doSimpleImageProcessing(0, 0);

  /* publish image count */
  EdgeDetect::publishImageNumber(image_counter_);
}

//}

// | --------------------- timer callbacks -------------------- |

/* callbackTimerCheckSubscribers() //{ */

void EdgeDetect::callbackTimerCheckSubscribers([[maybe_unused]] const ros::TimerEvent& te) {

  if (!is_initialized_)
    return;

  ros::Time time_now = ros::Time::now();

  /* check whether camera image msgs are coming */
  if (!got_image_) {
    ROS_WARN_THROTTLE(1.0, "Not received camera image since node launch.");
  } else {
    if ((time_now - time_last_image_).toSec() > 1.0) {
      ROS_WARN_THROTTLE(1.0, "Not received camera image msg for %f sec.", (time_now - time_last_image_).toSec());
    }
  }

  /* check whether camera info msgs are coming */
  if (!got_camera_info_) {
    ROS_WARN_THROTTLE(1.0, "Not received camera info msg since node launch.");
  } else {
    if ((time_now - time_last_image_).toSec() > 1.0) {
      ROS_WARN_THROTTLE(1.0, "Not received camera info msg for %f sec.", (time_now - time_last_camera_info_).toSec());
    }
  }
}

//}

// | -------------------- other functions ------------------- |

/* publishImageNumber() //{ */

void EdgeDetect::publishImageNumber(uint64_t count) {

  std_msgs::UInt64 message;

  /* set the value */
  message.data = count;

  /* publish the message */
  try {
    pub_test_.publish(message);
  }
  catch (...) {
    ROS_ERROR("Exception caught during publishing topic %s.", pub_test_.getTopic().c_str());
  }
}

//}

/* doSimpleImageProcessing() //{ */

void EdgeDetect::doSimpleImageProcessing(int, void*) {

  // !!!!
  // BASED ON EXAMPLE http://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/canny_detector/canny_detector.html
  cv::Mat src_gray, dst, detected_edges;

  // initialize some variables
  int ratio       = 3;
  int kernel_size = 3;


  {
    std::scoped_lock lock(mutex_image_);

    // Create a matrix of the same type and size as src (for dst)
    dst.create(image_.size(), image_.type());

    // Convert the image to grayscale
    cv::cvtColor(image_, src_gray, CV_BGR2GRAY);
  }

  // Reduce noise with a kernel 3x3
  cv::blur(src_gray, detected_edges, cv::Size(3, 3));

  // Canny detector
  cv::Canny(detected_edges, detected_edges, low_threshold_, low_threshold_ * ratio, kernel_size);

  // Using Canny's output as a mask, we display our result
  dst = cv::Scalar::all(0);

  {
    std::scoped_lock lock(mutex_image_);

    // copy the result
    image_.copyTo(dst, detected_edges);
  }

  // show the image
  cv::imshow("edges", dst);
}

//}


}  // namespace vision_example

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(vision_example::EdgeDetect, nodelet::Nodelet);
