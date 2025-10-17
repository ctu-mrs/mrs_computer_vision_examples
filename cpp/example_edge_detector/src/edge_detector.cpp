/* includes //{ */

#include <rclcpp/rclcpp.hpp>

/* TF2 related ROS includes */
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

/* camera image messages */
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

/* long unsigned integer message */
#include <std_msgs/msg/u_int16.hpp>

/* some STL includes */
#include <stdlib.h>
#include <mutex>

/* some OpenCV includes */
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/* ROS includes for working with OpenCV and images */
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_geometry/pinhole_camera_model.hpp>
#include <image_transport/camera_subscriber.hpp>

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/node.h>
#include <mrs_lib/timer_handler.h>
#include <mrs_lib/dynparam_mgr.h>
#include <mrs_lib/mutex.h>

//}

/* typedefs //{ */

#if USE_ROS_TIMER == 1
typedef mrs_lib::ROSTimer TimerType;
#else
typedef mrs_lib::ThreadTimer TimerType;
#endif

//}

namespace example_edge_detector
{

/* DynParams_t //{ */

struct DynParams_t
{
  int low_threshold;
};

//}

/* class EdgeDetector //{ */

class EdgeDetector : public mrs_lib::Node {
public:
  EdgeDetector(rclcpp::NodeOptions options);

private:
  void initialize();

  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  /* flags */
  std::atomic<bool> is_initialized_ = false;

  std::string _uav_name_;

  // | --------------------- MRS transformer -------------------- |

  std::unique_ptr<mrs_lib::Transformer> transformer_;

  // | ---------------------- msg callbacks --------------------- |

  void callbackCamera(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg);

  image_transport::CameraSubscriber sub_camera_;

  image_geometry::PinholeCameraModel camera_model_;

  // | --------------------- timer callbacks -------------------- |

  void timerCheckSubscribers(void);

  std::shared_ptr<TimerType> timer_check_subscribers_;

  double _rate_timer_check_subscribers_;

  // | -------------------- image processing -------------------- |

  static cv::Mat detectEdgesCanny(cv::InputArray image, int low_threshold);
  cv::Mat        projectWorldPointToImage(cv::InputArray image, const rclcpp::Time& image_stamp, const double x, const double y, const double z);

  // | ------------- variables for point projection ------------- |

  std::string world_frame_id_;
  double      world_point_x_;
  double      world_point_y_;
  double      world_point_z_;

  // | ----------------------- publishers ----------------------- |

  image_transport::Publisher pub_edges_;
  image_transport::Publisher pub_projection_;

  // | ------------------- dynamic reconfigure ------------------ |

  std::shared_ptr<mrs_lib::DynparamMgr> dynparam_mgr_;
  std::mutex                            mutex_drs_params_;
  DynParams_t                           drs_params_;

  // | --------------------- other functions -------------------- |

  void publishOpenCVImage(cv::InputArray detected_edges, const std_msgs::msg::Header& header, const std::string& encoding,
                          const image_transport::Publisher& pub);
};

//}

/* EdgeDetector() //{ */

EdgeDetector::EdgeDetector(rclcpp::NodeOptions options) : Node("example_edge_detector", options) {
  initialize();
}

//}

/* initialize() //{ */

void EdgeDetector::initialize() {

  node_  = this->this_node_ptr();
  clock_ = node_->get_clock();

  // | ------------------- load ros parameters ------------------ |
  /* (mrs_lib implementation checks whether the parameter was loaded or not) */

  mrs_lib::ParamLoader param_loader(node_);

  param_loader.addYamlFileFromParam("config");

  dynparam_mgr_ = std::make_shared<mrs_lib::DynparamMgr>(node_, mutex_drs_params_);

  // Dynparam_mgr has its own param loader.
  // This will replicate the params from our main param loader into the dynparam_mgr, so
  // that dynparam_
  dynparam_mgr_->get_param_provider().copyYamls(param_loader.getParamProvider());

  dynparam_mgr_->register_param("reconfigurable/canny_threshold", &drs_params_.low_threshold, mrs_lib::DynparamMgr::range_t<int>(0, 100));

  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("rate/check_subscribers", _rate_timer_check_subscribers_);
  param_loader.loadParam("world_frame_id", world_frame_id_);
  param_loader.loadParam("world_point/x", world_point_x_);
  param_loader.loadParam("world_point/y", world_point_y_);
  param_loader.loadParam("world_point/z", world_point_z_);

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "failed to load non-optional parameters!");
    rclcpp::shutdown();
    exit(1);
  }

  // | --------------------- tf transformer --------------------- |

  transformer_ = std::make_unique<mrs_lib::Transformer>(node_);
  transformer_->setDefaultPrefix(_uav_name_);
  transformer_->retryLookupNewest(true);

  /* initialize the image transport, needs node handle */
  image_transport::ImageTransport it(node_);

  // | ----------------- initialize subscribers ----------------- |

  sub_camera_ = it.subscribeCamera("~/image_in/image", 10, std::bind(&EdgeDetector::callbackCamera, this, std::placeholders::_1, std::placeholders::_2));

  // | ------------------ initialize publishers ----------------- |

  pub_edges_      = it.advertise("~/detected_edges", 1);
  pub_projection_ = it.advertise("~/projected_point", 1);

  // | -------------------- initialize timers ------------------- |

  mrs_lib::TimerHandlerOptions timer_opts_start;

  timer_opts_start.node      = this_node_ptr();
  timer_opts_start.autostart = true;

  {
    std::function<void()> callback_fcn = std::bind(&EdgeDetector::timerCheckSubscribers, this);

    timer_check_subscribers_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(_rate_timer_check_subscribers_, clock_), callback_fcn);
  }

  // | -------------------- finisht the init -------------------- |

  RCLCPP_INFO_ONCE(node_->get_logger(), "initialized");

  is_initialized_ = true;
}

//}

// | ---------------------- msg callbacks --------------------- |

/* callbackCamera() method //{ */

void EdgeDetector::callbackCamera(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg) {

  if (!is_initialized_) {
    return;
  }

  auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

  const std::string color_encoding     = "bgr8";
  const std::string grayscale_encoding = "mono8";

  // update the camera model using the latest camera info message
  camera_model_.fromCameraInfo(info_msg);

  // toCvShare avoids copying the image data and instead copies only the (smart) constpointer
  // to the data. Then, the data cannot be changed (it is potentially shared between multiple nodes) and
  // it is automatically freed when all pointers to it are released. If you want to modify the image data,
  // use toCvCopy (see https://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages),
  // or copy the image data using cv::Mat::copyTo() method.
  // Adittionally, toCvShare and toCvCopy will convert the input image to the specified encoding
  // if it differs from the one in the message. Try to be consistent in what encodings you use throughout the code.
  const cv_bridge::CvImageConstPtr bridge_image_ptr = cv_bridge::toCvShare(image_msg, color_encoding);
  const std_msgs::msg::Header      msg_header       = image_msg->header;

  // | ---------- Detect edges in the image using Canny --------- |

  /* find edges in the image */
  const auto detected_edges = EdgeDetector::detectEdgesCanny(bridge_image_ptr->image, drs_params.low_threshold);

  /* publish the image with the detected edges */
  EdgeDetector::publishOpenCVImage(detected_edges, msg_header, grayscale_encoding, pub_edges_);

  // | ----------- Project a world point to the image ----------- |

  /* find edges in the image */
  const auto projection_image = EdgeDetector::projectWorldPointToImage(bridge_image_ptr->image, msg_header.stamp, 0, 0, 0);

  /* publish the image with the detected edges */
  EdgeDetector::publishOpenCVImage(projection_image, msg_header, color_encoding, pub_projection_);
}

//}

// | --------------------- timer callbacks -------------------- |

/* timerCheckSubscribers() method //{ */

void EdgeDetector::timerCheckSubscribers(void) {

  if (!is_initialized_) {
    return;
  }

  int num_pubs = sub_camera_.getNumPublishers();

  RCLCPP_INFO(node_->get_logger(), "the image subscriber report %d publishers", num_pubs);
}

//}

/* publishOpenCVImage() method //{ */

void EdgeDetector::publishOpenCVImage(cv::InputArray image, const std_msgs::msg::Header& header, const std::string& encoding,
                                      const image_transport::Publisher& pub) {

  // Prepare a cv_bridge image to be converted to the ROS message
  cv_bridge::CvImage bridge_image_out;

  // Set the desired message header (time stamp and frame id)
  bridge_image_out.header = header;

  // Copy the cv::Mat, pointing to the image
  bridge_image_out.image = image.getMat();

  // Fill out the message encoding - this tells ROS how to interpret the raw image data
  // (see https://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages)
  bridge_image_out.encoding = encoding;

  // Now convert the cv_bridge image to a ROS message
  sensor_msgs::msg::Image::SharedPtr out_msg = bridge_image_out.toImageMsg();

  // ... and publish the message
  pub.publish(out_msg);
}

//}

/* detectEdgesCanny() method //{ */

cv::Mat EdgeDetector::detectEdgesCanny(cv::InputArray image, int low_threshold) {

  // BASED ON EXAMPLE https://docs.opencv.org/3.2.0/da/d5c/tutorial_canny_detector.html
  cv::Mat src_gray, detected_edges;

  // initialize some variables
  const int ratio       = 3;
  const int kernel_size = 3;

  // Convert the image to grayscale
  cv::cvtColor(image, src_gray, CV_BGR2GRAY);

  // Reduce noise with a kernel 3x3
  cv::blur(src_gray, detected_edges, cv::Size(3, 3));

  // Canny detector
  cv::Canny(detected_edges, detected_edges, low_threshold, low_threshold * ratio, kernel_size);

  // Return the detected edges (not colored)
  return detected_edges;
}

//}

/* projectWorldPointToImage() method //{ */

cv::Mat EdgeDetector::projectWorldPointToImage(cv::InputArray image, const rclcpp::Time& image_stamp, const double x, const double y, const double z) {

  // cv::InputArray indicates that the variable should not be modified, but we want
  // to draw into the image. Therefore we need to copy it.
  cv::Mat projected_point;
  image.copyTo(projected_point);

  // | --------- transform the point to the camera frame -------- |

  geometry_msgs::msg::PoseStamped pt3d_world;

  pt3d_world.header.frame_id = _uav_name_ + "/" + world_frame_id_;
  pt3d_world.header.stamp    = image_stamp;
  pt3d_world.pose.position.x = x;
  pt3d_world.pose.position.y = y;
  pt3d_world.pose.position.z = z;

  std::string camera_frame = camera_model_.tfFrame();

  auto ret = transformer_->transformSingle(pt3d_world, camera_frame);

  geometry_msgs::msg::PoseStamped pt3d_cam;

  if (ret) {
    pt3d_cam = ret.value();
  } else {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "failed to tranform point from world to camera frame, cannot backproject point to image");
    return projected_point;
  }

  // fill in the header from the transformed point
  pt3d_world.header = pt3d_cam.header;

  // | ----------- backproject the point from 3D to 2D ---------- |

  const cv::Point3d pt3d(pt3d_cam.pose.position.x, pt3d_cam.pose.position.y, pt3d_cam.pose.position.z);
  const cv::Point2d pt2d = camera_model_.project3dToPixel(pt3d);  // this is now in rectified image coordinates

  // | ----------- unrectify the 2D point coordinates ----------- |

  // This is done to correct for camera distortion. IT has no effect in simulation, where the camera model
  // is an ideal pinhole camera, but usually has a BIG effect when using real cameras, so don't forget this part!
  const cv::Point2d pt2d_unrec = camera_model_.unrectifyPoint(pt2d);  // this is now in unrectified image coordinates

  // | --------------- draw the point to the image -------------- |

  // The point will be drawn as a filled circle with the coordinates as text in the image
  const int        pt_radius = 5;      // pixels
  const cv::Scalar color(255, 0, 0);   // red or blue color, depending on the pixel ordering (BGR or RGB)
  const int        pt_thickness = -1;  // pixels, -1 means filled
  cv::circle(projected_point, pt2d_unrec, pt_radius, color, pt_thickness);

  // Draw the text with the coordinates to the image
  const std::string coord_txt = "[" + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) + "]";
  const cv::Point2d txt_pos(pt2d_unrec.x + 5, pt2d_unrec.y + 5);  // offset the text a bit to avoid overlap with the circle
  const int         txt_font       = cv::FONT_HERSHEY_PLAIN;      // some default OpenCV font
  const double      txt_font_scale = 1.0;
  cv::putText(projected_point, coord_txt, txt_pos, txt_font, txt_font_scale, color);

  return projected_point;
}

//}

}  // namespace example_edge_detector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(example_edge_detector::EdgeDetector)
