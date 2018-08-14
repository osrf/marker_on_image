#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
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
  MarkerOnImage() : nh_("~"), it_(nh_), tf_listener_(tf_buffer_)
  {
    // Subscribe to images
    std::string in_image_topic;
    nh_.param<std::string>("in_image_topic", in_image_topic,
        "camera/rgb/image_raw");

    image_sub_ = it_.subscribeCamera(in_image_topic, 1,
      &MarkerOnImage::onImage, this);

    // Subscribe to markers
    std::string in_marker_topic;
    nh_.param<std::string>("in_marker_topic", in_marker_topic, "markers");

    marker_sub_ = nh_.subscribe(in_marker_topic, 1,
      &MarkerOnImage::onMarker, this);

    // Publish images with overlayes markers
    std::string out_image_topic;
    nh_.param<std::string>("out_image_topic", out_image_topic,
        "camera/rgb/image_markers");

    overlay_pub_ = it_.advertise(out_image_topic, 1);

    ROS_INFO("marker_on_image node combining images from [%s] and markers"
        " from [%s] into images on [%s]",
        in_image_topic.c_str(),
        in_marker_topic.c_str(),
        out_image_topic.c_str());
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

      geometry_msgs::TransformStamped cam_to_frame_msg;
      try
      {
        ros::Duration timeout(1.0 / 30);
        cam_to_frame_msg = tf_buffer_.lookupTransform(cam_model_.tfFrame(),
            frame_id, ros::Time(0));
      }
      catch (tf2::TransformException & ex)
      {
        ROS_WARN("TF exception:\n%s", ex.what());
        markers_.erase(it++);
        continue;
      }

      auto cam_to_frame_eigen = tf2::transformToEigen(cam_to_frame_msg);

      // Marker offset
      auto frame_to_marker = it->msg.pose;

      Eigen::Affine3d frame_to_marker_eigen;
      tf2::fromMsg(frame_to_marker, frame_to_marker_eigen);

      auto cam_to_marker_eigen = cam_to_frame_eigen * frame_to_marker_eigen;

      // Skip if behind camera
      if (cam_to_marker_eigen.translation().z() <= 0)
      {
        markers_.erase(it++);
        continue;
      }

      // Transform to image
      cv::Point3d pos_3d(cam_to_marker_eigen.translation().x(),
                         cam_to_marker_eigen.translation().y(),
                         cam_to_marker_eigen.translation().z());

      auto pos_2d = cam_model_.project3dToPixel(pos_3d);

      // Draw according to type
      switch (it->msg.type)
      {
        case visualization_msgs::Marker::SPHERE:
          drawSphere(cv_ptr, it->msg, pos_2d,
              cam_to_marker_eigen.translation().z());
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
  /// \param[in] dist Marker distance to camera on Z axis
  void drawSphere(const cv_bridge::CvImagePtr image,
      const visualization_msgs::Marker marker,
      const cv::Point2d pos, const double dist)
  {
    if (dist <= 0)
      return;

    // 3D radius
    if (marker.scale.x != marker.scale.y ||
        marker.scale.x != marker.scale.z)
    {
      ROS_WARN("Sphere marker unevenly scaled, taking X: %f\n",
          marker.scale.x);
    }
    auto radius_3d = marker.scale.x * 0.5;


    // Camera focal lengths
    if (cam_model_.fx() != cam_model_.fy())
    {
      ROS_WARN("Camera has different focal lengths, taking Fx: %f\n",
          cam_model_.fx());
    }
    auto fx = cam_model_.fx();

    // Radius in px
    double radius = radius_3d * fx / dist;

    // Color from msg
    auto color = CV_RGB(marker.color.r*255,
                        marker.color.g*255,
                        marker.color.b*255);

    // Draw circle onto image
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

  /// TF buffer
  tf2_ros::Buffer tf_buffer_;

  /// Listen to TF transforms (for camera transform)
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "marker_on_image");
  MarkerOnImage mOi;
  ros::spin();
  return 0;
}

