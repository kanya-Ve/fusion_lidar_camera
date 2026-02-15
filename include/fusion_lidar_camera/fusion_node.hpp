#ifndef FUSION_LIDAR_CAMERA_FUSION_NODE_HPP
#define FUSION_LIDAR_CAMERA_FUSION_NODE_HPP

#include <deque>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "yolo_msgs/msg/detection.hpp"
#include "yolo_msgs/msg/detection_array.hpp"
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

//for RANSAC
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace fusion_lidar_camera
{

// Structure to hold timed messages for buffering
struct TimedMsg {
    std::chrono::steady_clock::time_point timestamp;
    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg;
};

class FusionLidarCamera : public rclcpp::Node
{
public:
    FusionLidarCamera();

private:
    // typedef message_filters::sync_policies::ApproximateTime<
    //     sensor_msgs::msg::PointCloud2,
    //     sensor_msgs::msg::Image,
    //     yolo_msgs::msg::DetectionArray> SyncPolicy;
    // typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;

    // Subscribers
    // message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_points_;
    // message_filters::Subscriber<sensor_msgs::msg::Image> sub_image_;
    // message_filters::Subscriber<yolo_msgs::msg::DetectionArray> sub_detections_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_points_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_image_;
    rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr subscriber_detections_;

    // // Synchronizer
    // std::shared_ptr<Synchronizer> sync_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_colored_points_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_projected_image_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_colored_filtered_points_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pub_detection_clouds_;

    // Debug timer
    rclcpp::TimerBase::SharedPtr debug_timer_;

    // Calibration parameters
    Eigen::Matrix4d T_lidar_camera_;
    Eigen::Matrix4d T_camera_lidar_;  // Inverse transformation (camera to LiDAR)
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    // ROI parameters
    bool use_roi_;
    int roi_x_, roi_y_, roi_width_, roi_height_;
    
    // PCD saving parameters
    bool save_pcd_;
    int pcd_save_interval_;
    int pcd_counter_ = 0;
    int save_counter_ = 0;

    // Private methods
    void initializeCalibrationParams();
    void debugCallback();
    void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
                  const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
                  const yolo_msgs::msg::DetectionArray::ConstSharedPtr& detections_msg);
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void detectionCallback(const yolo_msgs::msg::DetectionArray::ConstSharedPtr msg);
    
    // Point cloud processing functions
    std::pair<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>> 
    classifyPointsByDetections(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
        const yolo_msgs::msg::DetectionArray::ConstSharedPtr& detections_msg,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_cloud,
        cv::Mat& projected_image);
    
    void publishAndSaveResults(
        const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& detection_clouds,
        const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& colored_detection_clouds,
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_cloud,
        const cv::Mat& projected_image,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
        const yolo_msgs::msg::DetectionArray::ConstSharedPtr& detections_msg);

    // Utility functions
    double rgb2hue(int r, int g, int b);
    void scaleCalibrationParams(int image_width, int image_height);

    // Image scaling parameters
    bool is_calibration_scaled_;
    double current_scale_x_;
    double current_scale_y_;

    sensor_msgs::msg::PointCloud2::ConstSharedPtr _pointcloud;
    sensor_msgs::msg::Image::ConstSharedPtr _image;
    yolo_msgs::msg::DetectionArray::ConstSharedPtr _detections;

    std::deque<TimedMsg> _buffer_pointcloud;
    size_t _buffer_size = 30;  // Size of the buffer for point clouds
    std::deque<sensor_msgs::msg::Image::ConstSharedPtr> _buffer_image;
    

    //RANSAC
    void performRANSAC();

    void process();
};

}  // namespace fusion_lidar_camera

#endif  // FUSION_LIDAR_CAMERA_FUSION_NODE_HPP