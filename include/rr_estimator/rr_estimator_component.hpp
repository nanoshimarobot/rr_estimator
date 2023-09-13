#include <rclcpp/rclcpp.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/aruco.hpp>
#include <opencv4/opencv2/opencv.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace abu2023 {

template <class... Args>
    std::string format(const char *fmt, Args &&...args)
    {
        size_t str_len = std::snprintf(nullptr, 0, fmt, args...);
        std::string buf;
        buf.resize(str_len);
        std::snprintf(buf.data(), str_len + 1, fmt, args...);
        return buf;
    }

class RREstimator : public rclcpp::Node {
private:
  // pub / sub
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr detected_result_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr aruco_camera_info_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  // cv params
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
  cv::Ptr<cv::aruco::Dictionary> predefined_dict_;
  cv::Matx33d intrinsic_matrix_;
  cv::Mat distortion_coef_;

  image_geometry::PinholeCameraModel camera_model_;

  double marker_size_;
  std::vector<int64_t> reserved_marker_id_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

public:
  RREstimator(const rclcpp::NodeOptions &options) : RREstimator("", options) {}
  RREstimator(const std::string &name_space = "", const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("rr_estimator_node", name_space, options) {
    detector_params_ = cv::aruco::DetectorParameters::create();
    detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    predefined_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    declare_parameter("RREstimator.target_marker_size", 0.12);
    marker_size_ = get_parameter("RREstimator.target_marker_size").as_double();
    declare_parameter("RREstimator.target_marker_id", std::vector<int64_t>{0, 1, 2, 3});
    reserved_marker_id_ = get_parameter("RREstimator.target_marker_id").as_integer_array();

    detected_result_pub_ = this->create_publisher<sensor_msgs::msg::Image>("aruco/result_img", rclcpp::QoS(10));
    aruco_camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("aruco/camera_info", rclcpp::QoS(10));

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", rclcpp::SensorDataQoS().keep_last(1), [&](const sensor_msgs::msg::Image::SharedPtr msg) {
          cv_bridge::CvImage::Ptr input_img_ = std::make_shared<cv_bridge::CvImage>();
          input_img_ = cv_bridge::toCvCopy(*msg, "mono8");
          cv::Mat input_cv_img(input_img_->image);
          std::vector<int> detected_marker_id;
          std::vector<std::vector<cv::Point2f>> corners, rejected;
          cv::aruco::detectMarkers(input_cv_img, predefined_dict_, corners, detected_marker_id, detector_params_,
                                   rejected);

          if (detected_marker_id.size() > 0) {
            std::vector<geometry_msgs::msg::TransformStamped> detected_rr_pose_;
            std::vector<cv::Vec3d> rotation_vectors, translation_vectors;
            cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, intrinsic_matrix_, distortion_coef_,
                                                 rotation_vectors, translation_vectors);
            for (size_t i = 0; i < translation_vectors.size(); ++i) {
              geometry_msgs::msg::TransformStamped tf_msg =
                  create_tf_msg("map", format("id", detected_marker_id.at(i)), translation_vectors.at(i), rotation_vectors.at(i));
            }
            cv::aruco::drawDetectedMarkers(input_cv_img, corners, detected_marker_id);
            cv::aruco::drawAxis(input_cv_img, intrinsic_matrix_, distortion_coef_, rotation_vectors.at(0),
                                translation_vectors.at(0), marker_size_ * 0.5f);
            geometry_msgs::msg::TransformStamped tf_msg =
                create_tf_msg("map", "id0", translation_vectors.at(0), rotation_vectors.at(0));
            tf_broadcaster_->sendTransform(tf_msg);
          }

          input_img_->image = input_cv_img;
          detected_result_pub_->publish(*(input_img_->toImageMsg()));
        });

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", rclcpp::QoS(10), [&](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
          camera_model_.fromCameraInfo(*msg);
          camera_model_.distortionCoeffs().copyTo(distortion_coef_);
          intrinsic_matrix_ = camera_model_.intrinsicMatrix();
          aruco_camera_info_pub_->publish(*msg);
        });
  }

  // utils for cv2tf
  /**
   * @brief Make tf2::Vector3 from cv_vector3d
   *
   * @param vec [in] cv::Vec3d
   * @return tf2::Vector3
   */
  tf2::Vector3 cv_vector3d_to_tf_vector3(cv::Vec3d &vec) { return {vec[0], vec[1], vec[2]}; }

  /**
   * @brief Make tf2::Quaternion from cv::Vec3d
   *
   * @param rotation_vector [in] cv::Vec3d
   * @return tf2::Quaternion
   */
  tf2::Quaternion cv_vector3d_to_tf_quaternion(cv::Vec3d &rotation_vector) {
    cv::Mat rotation_matrix;
    auto ax = rotation_vector[0], ay = rotation_vector[1], az = rotation_vector[2];
    auto angle = sqrt(ax * ax + ay * ay + az * az);
    auto cosa = cos(angle * 0.5);
    auto sina = sin(angle * 0.5);
    auto qx = ax * sina / angle;
    auto qy = ay * sina / angle;
    auto qz = az * sina / angle;
    auto qw = cosa;
    tf2::Quaternion q;
    q.setValue(qx, qy, qz, qw);
    return q;
  }

  /**
   * @brief Make tf2::Transform from cv::Vec3d
   *
   * @param trans
   * @param rot
   * @return tf2::Transform
   */
  tf2::Transform create_transform(cv::Vec3d &trans, cv::Vec3d &rot) {
    tf2::Transform transform;
    transform.setOrigin(cv_vector3d_to_tf_vector3(trans));
    transform.setRotation(cv_vector3d_to_tf_quaternion(rot));
    return transform;
  }

  /**
   * @brief Create tf_msg,
   *
   * @param frame_id [in] std::string, typically set to map
   * @param child_frame [in] std::string, detected marker frame ?
   * @param translation [in] origin
   * @param rotation [in] rotation
   * @return geometry_msgs::msg::TransformStamped
   */
  geometry_msgs::msg::TransformStamped create_tf_msg(std::string frame_id, std::string child_frame,
                                                     cv::Vec3d &translation, cv::Vec3d &rotation) {
    geometry_msgs::msg::TransformStamped ret_msg;
    tf2::Transform transform = create_transform(translation, rotation);
    ret_msg.header.frame_id = frame_id;
    ret_msg.header.stamp = this->get_clock()->now();
    ret_msg.child_frame_id = child_frame;
    ret_msg.transform.translation.x = transform.getOrigin().getX();
    ret_msg.transform.translation.y = transform.getOrigin().getY();
    ret_msg.transform.translation.z = transform.getOrigin().getZ();
    ret_msg.transform.rotation.x = transform.getRotation().getX();
    ret_msg.transform.rotation.y = transform.getRotation().getY();
    ret_msg.transform.rotation.z = transform.getRotation().getZ();
    ret_msg.transform.rotation.w = transform.getRotation().getW();
    return ret_msg;
  }
};
} // namespace abu2023