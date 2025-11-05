#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <sstream>
#include <iomanip>

class SimpleLidarCameraFusion : public rclcpp::Node
{
public:
    SimpleLidarCameraFusion() : Node("simple_lidar_camera_fusion")
    {
        // Declare and get ROI parameters
        this->declare_parameter("use_roi", false);
        this->declare_parameter("roi_x", 0);
        this->declare_parameter("roi_y", 0);
        this->declare_parameter("roi_width", 1920);
        this->declare_parameter("roi_height", 1080);
        
        use_roi_ = this->get_parameter("use_roi").as_bool();
        roi_x_ = this->get_parameter("roi_x").as_int();
        roi_y_ = this->get_parameter("roi_y").as_int();
        roi_width_ = this->get_parameter("roi_width").as_int();
        roi_height_ = this->get_parameter("roi_height").as_int();
        
        // Initialize calibration parameters
        initializeCalibrationParams();

        // Subscribers (only lidar and image, no camera_info)
        sub_points_.subscribe(this, "/livox/lidar");
        sub_image_.subscribe(this, "/image_raw");

        // Synchronizer with relaxed settings
        sync_ = std::make_shared<Synchronizer>(SyncPolicy(100), sub_points_, sub_image_);
        sync_->setAgePenalty(100.0);
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(1.0));
        sync_->registerCallback(std::bind(&SimpleLidarCameraFusion::callback, this,
                                          std::placeholders::_1, std::placeholders::_2));

        // Publishers
        pub_colored_points_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/colored_pointcloud", 10);
        pub_projected_image_ = this->create_publisher<sensor_msgs::msg::Image>("/projected_image", 10);

        // Debug timer (uncomment if needed)
        // debug_timer_ = this->create_wall_timer(
        //     std::chrono::seconds(5),
        //     std::bind(&SimpleLidarCameraFusion::debugCallback, this));

        RCLCPP_INFO(this->get_logger(), "Simple LiDAR-Camera fusion node initialized");
        RCLCPP_INFO(this->get_logger(), "ROI enabled: %s", use_roi_ ? "true" : "false");
        if (use_roi_) {
            RCLCPP_INFO(this->get_logger(), "ROI region: x=%d, y=%d, width=%d, height=%d", 
                       roi_x_, roi_y_, roi_width_, roi_height_);
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for synchronized messages...");
        RCLCPP_INFO(this->get_logger(), "Output topics: /colored_pointcloud, /projected_image");
    }

private:
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2,
        sensor_msgs::msg::Image>
        SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;

    // Subscribers
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_points_;
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_image_;

    // Synchronizer
    std::shared_ptr<Synchronizer> sync_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_colored_points_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_projected_image_;

    // Debug timer
    rclcpp::TimerBase::SharedPtr debug_timer_;

    // Calibration parameters
    Eigen::Matrix4d T_lidar_camera_;
    Eigen::Matrix4d T_camera_lidar_; // Inverse transformation (camera to LiDAR)
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    
    // ROI parameters
    bool use_roi_;
    int roi_x_, roi_y_, roi_width_, roi_height_;

    void initializeCalibrationParams()
    {
        // Camera intrinsics [fx, fy, cx, cy]
        double fx = 851.31751;
        double fy = 911.92538;
        double cx = 961.52687;
        double cy = 629.60832;

        camera_matrix_ = (cv::Mat_<double>(3, 3) << fx, 0, cx,
                          0, fy, cy,
                          0, 0, 1);

        // Distortion coefficients
        dist_coeffs_ = (cv::Mat_<double>(5, 1) << -0.140261, 0.022702, 0.001249, -0.002815, 0.0);

        // Transformation from LiDAR to camera (translation + quaternion)
        double tx = 0.016611758862352718;
        double ty = 0.003872525514552308;
        double tz = -0.06873708340948441;
        double qx = 0.4276976468170657;
        double qy = 0.43199059079862145;
        double qz = 0.5577489752301957;
        double qw = 0.5651326684932215;

        // Create transformation matrix
        Eigen::Quaterniond q(qw, qx, qy, qz);
        Eigen::Matrix3d R = q.toRotationMatrix();
        Eigen::Vector3d t(tx, ty, tz);

        T_lidar_camera_ = Eigen::Matrix4d::Identity();
        T_lidar_camera_.block<3, 3>(0, 0) = R;
        T_lidar_camera_.block<3, 1>(0, 3) = t;

        // Calculate inverse transformation (camera to LiDAR)
        T_camera_lidar_ = T_lidar_camera_.inverse();

        // Print calibration info
        RCLCPP_INFO(this->get_logger(), "Calibration parameters initialized");
        RCLCPP_INFO(this->get_logger(), "Camera intrinsics: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", fx, fy, cx, cy);
        RCLCPP_INFO(this->get_logger(), "Translation: tx=%.4f, ty=%.4f, tz=%.4f", tx, ty, tz);
        RCLCPP_INFO(this->get_logger(), "Quaternion: qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f", qx, qy, qz, qw);

        // Print transformation matrices
        std::stringstream ss;
        ss << "T_lidar_camera_ (LiDAR to Camera, 4x4 transformation matrix):\n";
        for (int i = 0; i < 4; i++)
        {
            ss << "[";
            for (int j = 0; j < 4; j++)
            {
                ss << std::fixed << std::setprecision(6) << T_lidar_camera_(i, j);
                if (j < 3)
                    ss << ", ";
            }
            ss << "]\n";
        }
        ss << "\nT_camera_lidar_ (Camera to LiDAR, 4x4 transformation matrix):\n";
        for (int i = 0; i < 4; i++)
        {
            ss << "[";
            for (int j = 0; j < 4; j++)
            {
                ss << std::fixed << std::setprecision(6) << T_camera_lidar_(i, j);
                if (j < 3)
                    ss << ", ";
            }
            ss << "]\n";
        }
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    }

    void debugCallback()
    {
        RCLCPP_INFO(this->get_logger(), "Debug: Node is running, waiting for synchronized messages");
        RCLCPP_INFO(this->get_logger(), "Subscribed topics: /livox/lidar, /image_raw");
    }

    void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg,
                  const sensor_msgs::msg::Image::ConstSharedPtr &image_msg)
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "First synchronized message received! Fusion starting...");

        try
        {
            // Convert ROS image to OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat image = cv_ptr->image;
            cv::Mat projected_image = image.clone();

            // Convert ROS pointcloud to PCL
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*cloud_msg, *cloud);

            // Create colored point cloud
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

            int processed_points = 0;
            int valid_projections = 0;

            // Process each point
            for (const auto &point : cloud->points)
            {
                if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
                {
                    continue;
                }
                processed_points++;

                // Transform point from LiDAR to camera coordinate system
                Eigen::Vector4d lidar_point(point.x, point.y, point.z, 1.0);
                // Eigen::Vector4d camera_point = T_lidar_camera_ * lidar_point;
                Eigen::Vector4d camera_point = T_camera_lidar_ * lidar_point;

                // Skip points behind the camera
                if (camera_point(2) <= 0)
                {
                    continue;
                }

                // Project to image plane
                std::vector<cv::Point3f> object_points = {cv::Point3f(camera_point(0), camera_point(1), camera_point(2))};
                std::vector<cv::Point2f> image_points;

                cv::projectPoints(object_points, cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F),
                                  camera_matrix_, dist_coeffs_, image_points);

                int u = static_cast<int>(image_points[0].x);
                int v = static_cast<int>(image_points[0].y);

                if (processed_points % 1000 == 0)
                {
                    RCLCPP_INFO(this->get_logger(), "Lidar point: (%.2f, %.2f, %.2f) -> Camera point: (%.2f, %.2f, %.2f)",
                                point.x, point.y, point.z, camera_point(0), camera_point(1), camera_point(2));
                    RCLCPP_INFO(this->get_logger(), "Projected to image: (u=%d, v=%d)", u, v);
                }

                // Check if projection is within image bounds
                if (u >= 0 && u < image.cols && v >= 0 && v < image.rows) {
                    // Additional ROI check if enabled
                    bool within_roi = true;
                    if (use_roi_) {
                        within_roi = (u >= roi_x_ && u < (roi_x_ + roi_width_) && 
                                     v >= roi_y_ && v < (roi_y_ + roi_height_));
                    }
                    
                    if (within_roi) {
                        valid_projections++;

                        // Get color from image
                        cv::Vec3b bgr = image.at<cv::Vec3b>(v, u);

                        // Create colored point using PCL's standard PointXYZRGB
                        pcl::PointXYZRGB colored_point;
                        colored_point.x = point.x;
                        colored_point.y = point.y;
                        colored_point.z = point.z;

                        // Set RGB values (PCL uses RGB, not BGR)
                        colored_point.r = bgr[2];
                        colored_point.g = bgr[1];
                        colored_point.b = bgr[0];

                        colored_cloud->points.push_back(colored_point);

                        // Draw point on projected image
                        cv::circle(projected_image, cv::Point(u, v), 2, cv::Scalar(0, 255, 0), -1);
                    }
                }
            }

            // Set cloud properties
            colored_cloud->width = colored_cloud->points.size();
            colored_cloud->height = 1;
            colored_cloud->is_dense = false;
            
            // Draw ROI rectangle on projected image if enabled
            if (use_roi_) {
                cv::rectangle(projected_image, 
                             cv::Point(roi_x_, roi_y_), 
                             cv::Point(roi_x_ + roi_width_, roi_y_ + roi_height_),
                             cv::Scalar(255, 0, 0), 2);  // Blue rectangle
            }

            // Publish colored point cloud
            sensor_msgs::msg::PointCloud2 colored_cloud_msg;
            pcl::toROSMsg(*colored_cloud, colored_cloud_msg);
            colored_cloud_msg.header = cloud_msg->header;
            pub_colored_points_->publish(colored_cloud_msg);

            // Publish projected image
            cv_bridge::CvImage projected_cv_image;
            projected_cv_image.header = image_msg->header;
            projected_cv_image.encoding = sensor_msgs::image_encodings::BGR8;
            projected_cv_image.image = projected_image;

            pub_projected_image_->publish(*projected_cv_image.toImageMsg());

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Processed %d points, %d valid projections, %zu colored points",
                                 processed_points, valid_projections, colored_cloud->points.size());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error in fusion callback: %s", e.what());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleLidarCameraFusion>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
