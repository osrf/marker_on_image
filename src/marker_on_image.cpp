#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/MarkerArray.h>

/// Holds information necessary to print a marker on an image
struct MarkerInfo
{
  /// Marker message
  visualization_msgs::Marker msg;

  /// Expiration time
  ros::Time expiration;
};

/// Subscribes to images and markers and publishes a new image with markers overlayed
class MarkerOnImage
{
public:

  /// Constructor
  MarkerOnImage() : it_(nh_)
  {
    // Subscribe to images
    image_sub_ = it_.subscribeCamera("/camera/rgb/image_raw", 1,
      &MarkerOnImage::onImage, this);

    // Subscribe to markers
    marker_sub_ = nh_.subscribe("/einstein/safe_mobility_cart", 1,
      &MarkerOnImage::onMarker, this);

    // Publish images with overlayes markers
    overlay_pub_ = it_.advertise("/camera/rgb/image_markers", 1);
  }

  /// Destructor
  ~MarkerOnImage()
  {
  }

private:

  /// Callback when an image is received
  /// \param[in] msg New image message
  /// \param[in] msg Camera info message
  void onImage(const sensor_msgs::ImageConstPtr & msg,
               const sensor_msgs::CameraInfoConstPtr & cam_msg)
  {
    // Process latest marker messages
    processPendingMarkers();

    // Convert to CV image
    auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    // Get camera model
    cam_model_.fromCameraInfo(cam_msg);

    // Current image time
    auto current_time = msg->header.stamp;

    // Iterate over current markers
    auto it = markers_.begin();
    while (it != markers_.end())
    {
      // Set expiration right before displaying, otherwise it may expire before ever showing
      if (it->msg.lifetime.toSec() > 0.0001f && it->expiration.toSec() < 0.0001f)
      {
        // marker_msg.header.stamp is zero, using now()
        it->expiration = current_time + it->msg.lifetime;
      }

      // Remove if expired
      if (it->expiration <= current_time)
      {
        markers_.erase(it++);
        continue;
      }

      // Remove if not supported
      if (it->msg.type != visualization_msgs::Marker::SPHERE)
      {
        markers_.erase(it++);
        continue;
      }

      // Get marker frame with respect to camera
      auto frame_id = it->msg.header.frame_id;

      tf::StampedTransform cam_to_frame_tf;
      try
      {
        ros::Duration timeout(1.0 / 30);
        tf_listener_.waitForTransform(cam_model_.tfFrame(), frame_id,
                                      current_time, timeout);
        tf_listener_.lookupTransform(cam_model_.tfFrame(), frame_id,
                                     current_time, cam_to_frame_tf);
      }
      catch (tf::TransformException & ex)
      {
        ROS_WARN("TF exception:\n%s", ex.what());
        return;
      }

      Eigen::Affine3d cam_to_frame_eigen;
      tf::transformTFToEigen(cam_to_frame_tf, cam_to_frame_eigen);

      // Marker offset
      auto frame_to_marker = it->msg.pose;

      Eigen::Affine3d frame_to_marker_eigen;
      tf::poseMsgToEigen(frame_to_marker, frame_to_marker_eigen);

      // DEBUG: skip spheres too close to odom frame
//      auto x = frame_to_marker.position.x;
//      auto y = frame_to_marker.position.y;
//      if (x*x + y*y < 0.5)
//      {
//        ++it;
//        continue;
//      }
//
//      // DEBUG: skip spheres behind odom frame
//      if (x < 0 && y < 0)
//      {
//        ++it;
//        continue;
//      }

      Eigen::Affine3d cam_to_marker_eigen = cam_to_frame_eigen * frame_to_marker_eigen;

      // Transform to image
      cv::Point3d pos_3d(cam_to_marker_eigen.translation().x(),
                         cam_to_marker_eigen.translation().y(),
                         cam_to_marker_eigen.translation().z());

      auto pos_2d = cam_model_.project3dToPixel(pos_3d);

      // Draw according to type
      switch (it->msg.type)
      {
        case visualization_msgs::Marker::SPHERE:
          drawSphere(cv_ptr, it->msg, pos_2d);
          break;

        default:
          ROS_WARN("Marker type not supported: %i\n", it->msg.type);
      }

      ++it;
    }

    // Publish image with overlayed markers
    overlay_pub_.publish(cv_ptr->toImageMsg());
  }

  /// Callback when a marker is received
  /// \param[in] msg New marker message
  void onMarker(const visualization_msgs::MarkerArrayPtr & msg)
  {
    // Append to pending markers
    pending_markers_.markers.insert(pending_markers_.markers.end(),
        msg->markers.begin(), msg->markers.end());
  }

  /// Process all pending marker messages
  void processPendingMarkers()
  {
    for (auto marker : pending_markers_.markers)
    {
      switch (marker.action)
      {
        case visualization_msgs::Marker::ADD:
          processAdd(marker);
          break;

        case visualization_msgs::Marker::DELETE:
          ROS_WARN("Marker delete currently not supported");
          break;

        case visualization_msgs::Marker::DELETEALL:
          ROS_WARN("Marker delete all currently not supported");
          break;

        default:
          ROS_ERROR("Unknown marker action: %d\n", marker.action);
      }
    }
    pending_markers_.markers.clear();
  }

  /// Process marker addition
  /// \param[in] marker_msg Message with new marker
  void processAdd(const visualization_msgs::Marker marker_msg)
  {
    // If already in list, remove so it is updated
    auto it = markers_.begin();
    while (it != markers_.end())
    {
      if (it->msg.id == marker_msg.id && it->msg.ns == marker_msg.ns)
      {
        markers_.erase(it++);
        break;
      }
      ++it;
    }

    // Marker info
    MarkerInfo info;
    info.msg = marker_msg;

    if (marker_msg.frame_locked)
    {
      ROS_WARN("Frame locked markers currently not supported");
    }

    markers_.push_back(info);
  }

  /// Draw a sphere marker onto an image
  /// \param[in] image OpenCV image pointer
  /// \param[in] marker Message describing marker
  /// \param[in] pos 2D position of marker on image
  void drawSphere(const cv_bridge::CvImagePtr image, visualization_msgs::Marker marker,
      cv::Point2d pos)
  {
    // Draw circle onto image
    // TODO(chapulina): scale according to distance
    double radius = 10;
    auto color = CV_RGB(marker.color.r*255, marker.color.g*255, marker.color.b*255);
    cv::circle(image->image, pos, radius, color, -1);
  }

  /// ROS Node handle
  ros::NodeHandle nh_;

  /// Image transport
  image_transport::ImageTransport it_;

  /// Image subscriber
  image_transport::CameraSubscriber image_sub_;

  /// Marker subscriber
  ros::Subscriber marker_sub_;

  /// Overlayed image publisher
  image_transport::Publisher overlay_pub_;

  /// Markers to be displayed
  std::list<MarkerInfo> markers_;

  /// Pending marker messages which haven't been processed yet
  visualization_msgs::MarkerArray pending_markers_;

  /// Camera model
  image_geometry::PinholeCameraModel cam_model_;

  /// Listen to TF transforms (for camera transform)
  tf::TransformListener tf_listener_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "marker_on_image");
  MarkerOnImage mOi;
  ros::spin();
  return 0;
}

