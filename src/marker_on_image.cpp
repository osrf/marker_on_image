/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

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

/// Holds a 3D pose and its 2D projection.
struct PoseProjection
{
  /// 3D pose
  Eigen::Affine3d pos_3d;

  /// 2D position
  cv::Point pos_2d;
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

    // Publish images with overlayed markers
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
  /// \param[in] cam_msg Camera info message
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
      if (it->msg.type != visualization_msgs::Marker::SPHERE &&
          it->msg.type != visualization_msgs::Marker::CUBE)
      {
        ROS_WARN("Marker type not supported: %i\n", it->msg.type);
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

      // Draw according to type
      switch (it->msg.type)
      {
        case visualization_msgs::Marker::SPHERE:
          drawSphere(cv_ptr, it->msg, cam_to_marker_eigen);
          break;

        case visualization_msgs::Marker::CUBE:
          drawCube(cv_ptr, it->msg, cam_to_marker_eigen);
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
  /// \param[in] center 3D transform from camera to marker's origin
  void drawSphere(const cv_bridge::CvImagePtr image,
      const visualization_msgs::Marker marker,
      const Eigen::Affine3d center)
  {
    // Skip if behind camera
    if (center.translation().z() <= 0)
    {
      return;
    }

    // Transform sphere's center point to image
    cv::Point3d pos_3d(center.translation().x(),
                       center.translation().y(),
                       center.translation().z());

    auto pos_2d = cam_model_.project3dToPixel(pos_3d);

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
    double radius = radius_3d * fx / pos_3d.z;

    // Color from msg
    auto color = CV_RGB(marker.color.r*255,
                        marker.color.g*255,
                        marker.color.b*255);

    // Draw circle onto image
    cv::circle(image->image, pos_2d, radius, color, -1);
  }

  /// Draw a cube marker onto an image
  /// \param[in] image OpenCV image pointer
  /// \param[in] marker Message describing marker
  /// \param[in] center 3D transform from camera to marker's origin
  void drawCube(const cv_bridge::CvImagePtr image,
      const visualization_msgs::Marker marker,
      const Eigen::Affine3d center)
  {
    if (center.translation().z() < 0)
    {
      // ROS_WARN("Center behind camera, skipping whole cube");
      return;
    }

    // Color from msg
    // \todo(louise) choose colors in a smarter way, could use BFLRDU for that
    // Also figure out how to add transparency
    std::vector<cv::Scalar> colors;
    colors.push_back(cv::Scalar(marker.color.b*255,
                                marker.color.g*255,
                                marker.color.r*255,
                                150));
    colors.push_back(cv::Scalar(marker.color.b*255 + 50,
                                marker.color.g*255 + 50,
                                marker.color.r*255 + 50,
                                150));
    colors.push_back(cv::Scalar(marker.color.b*255 - 50,
                                marker.color.g*255 - 50,
                                marker.color.r*255 - 50,
                                150));

    // Corner distances from center
    auto sX = marker.scale.x * 0.5;
    auto sY = marker.scale.y * 0.5;
    auto sZ = marker.scale.z * 0.5;

    // The one furthest from the camera, to choose which faces to draw
    auto furthest_corner = center;

    // 6 faces, each defined by 4 points
    std::map<std::string, std::vector<PoseProjection>> faces;
    for (auto x : {-sX, sX})
    {
      for (auto y : {-sY, sY})
      {
        for (auto z : {-sZ, sZ})
        {
          PoseProjection corner;

          // 3D
          Eigen::Affine3d offset;
          offset.translation().x() = x;
          offset.translation().y() = y;
          offset.translation().z() = z;

          corner.pos_3d = center * offset;

          // Keep the furthest corner
          if (corner.pos_3d.translation().z() > furthest_corner.translation().z())
            furthest_corner = corner.pos_3d;

          if (corner.pos_3d.translation().z() < 0)
          {
            // ROS_WARN("Corner behind camera, skipping whole cube");
            return;
          }

          // 2D
          cv::Point3d pos_3d(corner.pos_3d.translation().x(),
                             corner.pos_3d.translation().y(),
                             corner.pos_3d.translation().z());

          corner.pos_2d = cam_model_.project3dToPixel(pos_3d);

          // Add to faces
          if (x == -sX)
            faces["B"].push_back(corner);

          if (x == sX)
            faces["F"].push_back(corner);

          if (y == -sY)
            faces["L"].push_back(corner);

          if (y == sY)
            faces["R"].push_back(corner);

          if (z == -sZ)
            faces["D"].push_back(corner);

          if (z == sZ)
            faces["U"].push_back(corner);

          faces["debug"].push_back(corner);
        }
      }
    }

    // Sanity check: furthest corner should be different from center
    assert(furthest_corner.translation().z() != center.translation().z());

    // Remove faces which have furthest corner, because they are behind
    for (auto face = faces.begin(); face != faces.end();)
    {
      if (face->first == "debug")
      {
        ++face;
        continue;
      }

      // Sanity check: each face should have 4 corners
      assert(face->second.size() == 4);

      bool erased{false};

      for (auto corner : face->second)
      {
        if (corner.pos_3d.translation().x() == furthest_corner.translation().x() &&
            corner.pos_3d.translation().y() == furthest_corner.translation().y() &&
            corner.pos_3d.translation().z() == furthest_corner.translation().z())
        {
          faces.erase(face++);
          erased = true;
          continue;
        }
      }
      if (!erased)
        ++face;
    }

    // Sanity check: there should be only 3 faces + debug left
    assert(faces.size() == 4);

    // Draw a quadrilateral per face
    int count = 0;
    for (auto face : faces)
    {
      if (face.first == "debug")
      {
        continue;
      }

      cv::Point pts[4];

      for (auto i = 0; i < face.second.size(); ++i)
      {
        pts[i] = face.second[i].pos_2d;
      }

      // Points must be in order
      std::sort(pts, pts + 4, [](const cv::Point & lhs, const cv::Point & rhs)
      {
         if (lhs.y < rhs.y)
           return true;

         return lhs.x < rhs.x;
      });

      // Swap last 2 points
      std::swap(pts[2], pts[3]);

      // Draw polygon
      cv::fillConvexPoly(image->image, pts, 4, colors[count++]);
    }

    // Debug: print all corners and furthest corner in yellow
//    auto radius = 5.0;
//    for (auto corner : faces["debug"])
//    {
//      cv::circle(image->image, corner.pos_2d, radius, colors[0], -1);
//    }
//
//    {
//      cv::Point3d pos_3d(furthest_corner.translation().x(),
//                         furthest_corner.translation().y(),
//                         furthest_corner.translation().z());
//
//      auto pos_2d = cam_model_.project3dToPixel(pos_3d);
//      cv::circle(image->image, pos_2d, radius, CV_RGB(255, 255, 0), -1);
//    }
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

