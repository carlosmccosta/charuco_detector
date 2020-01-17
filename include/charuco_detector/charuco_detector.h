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
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


namespace charuco_detector {

	class ChArUcoDetector {
	public:
		ChArUcoDetector() {}
		~ChArUcoDetector() {}

		virtual void setupConfigurationFromParameterServer(ros::NodeHandlePtr &_node_handle, ros::NodeHandlePtr &_private_node_handle);
		virtual void startDetection();
		void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &_msg);
		void imageCallback(const sensor_msgs::ImageConstPtr &_msg);
		void applyMedianBlur(cv::Mat &image_in_out_);
		void applyDynamicRange(cv::Mat& image_in_out_);
		void applyBilateralFilter(cv::Mat &image_in_out_);
		void applyCLAHE(cv::Mat &image_in_out_);
		void applyAdaptiveThreshold(cv::Mat &image_in_out_);
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

		bool use_median_blur_;
		int median_blur_k_size_;

		bool use_dynamic_range_;

		bool use_bilateral_filter_;
		int bilateral_filter_pixel_neighborhood_;
		double bilateral_filter_sigma_color_;
		double bilateral_filter_sigma_space_;
		int bilateral_filter_border_type_;

		bool use_clahe_;
		double clahe_clip_limit_;
		int clahe_size_x_;
		int clahe_size_y_;

		bool use_adaptive_threshold_;
		double adaptive_threshold_max_value_;
		int adaptive_threshold_method_;
		int adaptive_threshold_type_;
		int adaptive_threshold_block_size_;
		double adaptive_threshold_constant_offset_from_mean_;

		bool use_static_tf_broadcaster_;
		double tf_broadcaster_republish_rate_;

		std::string sensor_frame_override_;
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
		tf2_ros::TransformBroadcaster tf_broadcaster_;
		geometry_msgs::TransformStamped transform_stamped_;
		bool transform_stamped_valid_;
	};

}
