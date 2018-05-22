#pragma once

/**\file charuco_detector.h
 * \brief Detector of ChArUco patterns
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// std includes
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

// external libs includes
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

// ros includes
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/static_transform_broadcaster.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


namespace charuco_detector {

	class ChArUcoDetector {
	public:
		ChArUcoDetector() {}
		~ChArUcoDetector() {}

		virtual void setupConfigurationFromParameterServer(ros::NodeHandlePtr &_node_handle, ros::NodeHandlePtr &_private_node_handle);
		virtual void startDetection();
		void imageCallback(const sensor_msgs::ImageConstPtr &_msg);
		void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &_msg);
		virtual bool detectChArUcoBoard(const cv::Mat &_image_grayscale, const cv::Mat &_camera_intrinsics, const cv::Mat &_camera_distortion_coefficients,
										cv::Vec3d &_camera_rotation_out, cv::Vec3d &_camera_translation_out,
										cv::InputOutputArray _image_with_detection_results, bool _show_rejected_markers);
		void fillPose(const cv::Vec3d &_camera_rotation, const cv::Vec3d &_camera_translation, geometry_msgs::PoseStamped &_pose_in_out);

	protected:
		cv::Ptr<cv::aruco::DetectorParameters> detector_parameters_;
		cv::Ptr<cv::aruco::Dictionary> dictionary_;
		cv::Ptr<cv::aruco::CharucoBoard> board_;
		double squares_sides_size_m_;
		double markers_sides_size_m_;
		int number_of_bits_for_markers_sides_;
		int number_of_markers_;
		int number_of_squares_in_x_;
		int number_of_squares_in_y_;
		int dictionary_id_;

		std::string charuco_tf_frame_;
		std::string image_topic_;
		std::string camera_info_topic_;
		std::string image_results_publish_topic_;
		std::string charuco_pose_publish_topic_;
		sensor_msgs::CameraInfo::ConstPtr camera_info_;
		cv::Mat camera_intrinsics_matrix;
		cv::Mat camera_distortion_coefficients_matrix;

		ros::NodeHandlePtr node_handle_;
		ros::NodeHandlePtr private_node_handle_;
		std::shared_ptr<image_transport::ImageTransport> image_transport_ptr_;
		image_transport::Subscriber image_subscriber_;
		std::shared_ptr<image_transport::ImageTransport> image_transport_results_ptr_;
		image_transport::Publisher image_results_publisher_;
		ros::Subscriber camera_info_subscriber_;
		ros::Publisher charuco_pose_publisher_;
		tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
	};

}
