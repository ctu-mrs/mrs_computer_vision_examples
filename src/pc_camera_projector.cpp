/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <stdlib.h>
#include <stdio.h>
#include <mutex>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/subscribe_handler.h>

#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

//}

/* defines //{ */

#ifdef COLOR_OCTOMAP_SERVER
using PCLPoint      = pcl::PointXYZRGB;
using PCLPointCloud = pcl::PointCloud<PCLPoint>;
#else
using PCLPoint      = pcl::PointXYZ;
using PCLPointCloud = pcl::PointCloud<PCLPoint>;
#endif

//}

namespace pc_camera_projector
{

/* class PcCameraProjector //{ */

class PcCameraProjector : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;

  bool is_initialized_ = false;

  // | --------------------- MRS transformer -------------------- |

  mrs_lib::Transformer transformer_;

  // | ---------------------- msg callbacks --------------------- |

  void                        callbackImage(const sensor_msgs::ImageConstPtr& msg);
  image_transport::Subscriber sub_image_;

  void            callbackCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg);
  ros::Subscriber sub_camera_info_;

  image_geometry::PinholeCameraModel camera_model_;
  std::mutex                         mutex_camera_model_;

  void                                                callback3dLidarCloud2(mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2>& wrp);
  mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2> sh_3dlaser_pc2_;

  // | ------------------ remembering the image ----------------- |

  cv_bridge::CvImageConstPtr image_;
  std_msgs::Header           image_header_;
  std::mutex                 mutex_image_;

  bool got_image_       = false;
  bool got_camera_info_ = false;

  // | ------------------------- params ------------------------- |

  int    _point_size_;
  double _distance_color_limit_;

  // | -------------------- image processing -------------------- |

  void projectPoint(cv::Mat& image, const double x, const double y, const double z);

  // | ----------------------- publishers ----------------------- |

  image_transport::Publisher pub_image_;
};

//}

/* onInit() method //{ */
void PcCameraProjector::onInit() {

  nh_ = ros::NodeHandle("~");

  ros::Time::waitForValid();

  // | ----------------------- load params ---------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "PcCameraProjector");

  param_loader.loadParam("point_size", _point_size_);
  param_loader.loadParam("distance_color_limit", _distance_color_limit_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[WaypointFlier]: failed to load non-optional parameters!");
    ros::shutdown();
  }

  // | --------------------- tf transformer --------------------- |

  transformer_ = mrs_lib::Transformer("PcCameraProjector");

  image_transport::ImageTransport it(nh_);

  // | ----------------- initialize subscribers ----------------- |

  sub_image_       = it.subscribe("image_in", 1, &PcCameraProjector::callbackImage, this);
  sub_camera_info_ = nh_.subscribe("camera_info_in", 1, &PcCameraProjector::callbackCameraInfo, this, ros::TransportHints().tcpNoDelay());

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "PcCameraProjector";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 1;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_3dlaser_pc2_ = mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2>(shopts, "point_cloud_in", &PcCameraProjector::callback3dLidarCloud2, this);

  // | ------------------ initialize publishers ----------------- |

  pub_image_ = it.advertise("image", 1);

  ROS_INFO_ONCE("[PcCameraProjector]: initialized");

  is_initialized_ = true;
}
//}

// | ------------------------ callbacks ----------------------- |

/* callbackCameraInfo() //{ */

void PcCameraProjector::callbackCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[PcCameraProjector]: getting camera info");

  {
    std::scoped_lock lock(mutex_camera_model_);

    camera_model_.fromCameraInfo(*msg);
  }

  got_camera_info_ = true;
}

//}

/* callbackImage() //{ */

void PcCameraProjector::callbackImage(const sensor_msgs::ImageConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[PcCameraProjector]: getting image");

  {
    std::scoped_lock lock(mutex_image_);

    image_        = cv_bridge::toCvShare(msg, msg->encoding);
    image_header_ = msg->header;
  }

  got_image_ = true;
}

//}

/* callback3dLidarCloud2() //{ */

void PcCameraProjector::callback3dLidarCloud2(mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2>& wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[PcCameraProjector]: getting point cloud");

  if (!(got_image_ && got_camera_info_)) {
    return;
  }

  sensor_msgs::PointCloud2ConstPtr cloud = wrp.getMsg();

  ros::Time time_start = ros::Time::now();

  PCLPointCloud::Ptr pc = boost::make_shared<PCLPointCloud>();
  pcl::fromROSMsg(*cloud, *pc);

  auto res = transformer_.getTransform(cloud->header.frame_id, image_header_.frame_id, image_header_.stamp);

  if (!res) {
    ROS_WARN_THROTTLE(1.0, "[OctomapServer]: insertCloudScanCallback(): could not find tf from %s to %s", cloud->header.frame_id.c_str(),
                      image_header_.frame_id.c_str());
    return;
  }

  Eigen::Matrix4f                 sensorToWorld;
  geometry_msgs::TransformStamped sensorToWorldTf = res.value().getTransform();
  pcl_ros::transformAsMatrix(sensorToWorldTf.transform, sensorToWorld);

  pcl::transformPointCloud(*pc, *pc, sensorToWorld);

  cv::Mat new_image;
  image_->image.copyTo(new_image);

  // all measured points: make it free on ray, occupied on endpoint:
  for (PCLPointCloud::const_iterator it = pc->begin(); it != pc->end(); ++it) {

    if (!(std::isfinite(it->x) && std::isfinite(it->y) && std::isfinite(it->z))) {
      continue;
    }

    projectPoint(new_image, it->x, it->y, it->z);
  }

  // | -------------------- publish the image ------------------- |

  {
    cv_bridge::CvImage image_out;

    image_out.header   = image_->header;
    image_out.image    = new_image;
    image_out.encoding = image_->encoding;

    sensor_msgs::ImageConstPtr out_msg = image_out.toImageMsg();

    pub_image_.publish(out_msg);
  }
}

//}

// | ------------------------ routines ------------------------ |

/* projectPoint() //{ */

void PcCameraProjector::projectPoint(cv::Mat& image, const double x, const double y, const double z) {

  // | ----------- backproject the point from 3D to 2D ---------- |

  if (z <= 0) {
    return;
  }

  const cv::Point3d pt3d(x, y, z);
  const cv::Point2d pt2d = camera_model_.project3dToPixel(pt3d);  // this is now in rectified image coordinates

  // | ----------- unrectify the 2D point coordinates ----------- |

  const cv::Point2d pt2d_unrec = camera_model_.unrectifyPoint(pt2d);  // this is now in unrectified image coordinates

  // | --------------- draw the point to the image -------------- |

  double dist = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));

  int red  = floor(255 - std::min(dist, _distance_color_limit_) * (255 / _distance_color_limit_));
  int blue = floor(std::min(dist, _distance_color_limit_) * (255 / _distance_color_limit_));

  const cv::Scalar color(red, 0, blue);
  const int        pt_thickness = -1;

  cv::circle(image, pt2d_unrec, _point_size_, color, pt_thickness);
}

//}

}  // namespace pc_camera_projector

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pc_camera_projector::PcCameraProjector, nodelet::Nodelet);
