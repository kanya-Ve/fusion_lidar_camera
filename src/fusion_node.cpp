#include "fusion_lidar_camera/fusion_node.hpp"
#include <sstream>
#include <iomanip>
#include <chrono>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <filesystem>

namespace fusion_lidar_camera
{

    FusionLidarCamera::FusionLidarCamera() : Node("fusion_lidar_camera")
    {
        // Declare parameters with default values
        this->declare_parameter<bool>("use_roi", false);
        this->declare_parameter<int>("roi_x", 0);
        this->declare_parameter<int>("roi_y", 0);
        this->declare_parameter<int>("roi_width", 1920);
        this->declare_parameter<int>("roi_height", 1080);
        this->declare_parameter<bool>("save_pcd", false);
        this->declare_parameter<int>("pcd_save_interval", 5); // Save every N detections

        // Get parameters
        use_roi_ = this->get_parameter("use_roi").as_bool();
        roi_x_ = this->get_parameter("roi_x").as_int();
        roi_y_ = this->get_parameter("roi_y").as_int();
        roi_width_ = this->get_parameter("roi_width").as_int();
        roi_height_ = this->get_parameter("roi_height").as_int();
        save_pcd_ = this->get_parameter("save_pcd").as_bool();
        pcd_save_interval_ = this->get_parameter("pcd_save_interval").as_int();

        save_pcd_ = false;
        pcd_save_interval_ = 50;

        // Initialize calibration parameters
        initializeCalibrationParams();

        subscriber_points_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar", 10, std::bind(&FusionLidarCamera::pointCloudCallback, this, std::placeholders::_1));
        // subscriber_image_ = this->create_subscription<sensor_msgs::msg::Image>(
        //     "/image_raw", 10, std::bind(&FusionLidarCamera::imageCallback, this, std::placeholders::_1));
        subscriber_image_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw/downsampled", 10, std::bind(&FusionLidarCamera::imageCallback, this, std::placeholders::_1));
        subscriber_detections_ = this->create_subscription<yolo_msgs::msg::DetectionArray>(
            "/yolo/detections", 10, std::bind(&FusionLidarCamera::detectionCallback, this, std::placeholders::_1));

        // // サブスクライバー
        // sub_points_.subscribe(this, "/livox/lidar");
        // sub_image_.subscribe(this, "/image_raw");
        // sub_detections_.subscribe(this, "/yolo/detections");

        // // 緩和設定でのシンクロナイザー
        // sync_ = std::make_shared<Synchronizer>(SyncPolicy(100), sub_points_, sub_image_, sub_detections_);
        // sync_->setAgePenalty(100.0);
        // // sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(1.0));
        // sync_->registerCallback(std::bind(&FusionLidarCamera::callback, this,
        //                                   std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        // Publishers
        pub_colored_points_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/colored_pointcloud", 10);
        pub_projected_image_ = this->create_publisher<sensor_msgs::msg::Image>("/projected_image", 10);
        pub_colored_filtered_points_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/colored_filtered_pointcloud", 10);
        // Initialize with some detection publishers (will be expanded dynamically if needed)
        for (int i = 0; i < 10; ++i) // Pre-create 10 detection publishers
        {
            std::string topic_name = "/detection_pointcloud_" + std::to_string(i);
            pub_detection_clouds_.push_back(
                this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name, 10));
        }

        RCLCPP_INFO(this->get_logger(), "Simple LiDAR-Camera fusion node initialized");

        if (use_roi_)
        {
            RCLCPP_INFO(this->get_logger(), "ROI enabled: x=%d, y=%d, width=%d, height=%d",
                        roi_x_, roi_y_, roi_width_, roi_height_);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "ROI disabled - using full image");
        }

        if (save_pcd_)
        {
            RCLCPP_INFO(this->get_logger(), "PCD saving enabled: interval=%d frames, output dir=/home/yakan/lidar_ws/pointcloud",
                        pcd_save_interval_);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "PCD saving disabled");
        }

        RCLCPP_INFO(this->get_logger(), "Waiting for synchronized messages...");
        RCLCPP_INFO(this->get_logger(), "Output topics: /colored_pointcloud, /projected_image");

        // Initialize scaling flags
        is_calibration_scaled_ = false;
        current_scale_x_ = 1.0;
        current_scale_y_ = 1.0;
    }

    void FusionLidarCamera::initializeCalibrationParams()
    {
        // Original camera intrinsics for 1920x1280 image [fx, fy, cx, cy]
        double fx_orig = 851.31751;
        double fy_orig = 911.92538;
        double cx_orig = 961.52687;
        double cy_orig = 629.60832;

        // Store original parameters (will be scaled when first image is received)
        camera_matrix_ = (cv::Mat_<double>(3, 3) << fx_orig, 0, cx_orig,
                          0, fy_orig, cy_orig,
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
        RCLCPP_INFO(this->get_logger(), "Original intrinsics (1920x1280): fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
                    camera_matrix_.at<double>(0, 0), camera_matrix_.at<double>(1, 1),
                    camera_matrix_.at<double>(0, 2), camera_matrix_.at<double>(1, 2));
        RCLCPP_INFO(this->get_logger(), "Translation: tx=%.4f, ty=%.4f, tz=%.4f", tx, ty, tz);
        RCLCPP_INFO(this->get_logger(), "Quaternion: qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f", qx, qy, qz, qw);
        RCLCPP_INFO(this->get_logger(), "Camera parameters will be scaled automatically based on input image size");

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

    void FusionLidarCamera::scaleCalibrationParams(int image_width, int image_height)
    {
        if (!is_calibration_scaled_)
        {
            // Original image size
            double orig_width = 1920.0;
            double orig_height = 1280.0;

            // Calculate scaling factors based on actual image size
            current_scale_x_ = static_cast<double>(image_width) / orig_width;
            current_scale_y_ = static_cast<double>(image_height) / orig_height;

            // Scale intrinsic parameters
            camera_matrix_.at<double>(0, 0) *= current_scale_x_; // fx
            camera_matrix_.at<double>(1, 1) *= current_scale_y_; // fy
            camera_matrix_.at<double>(0, 2) *= current_scale_x_; // cx
            camera_matrix_.at<double>(1, 2) *= current_scale_y_; // cy

            is_calibration_scaled_ = true;

            RCLCPP_INFO(this->get_logger(), "Auto-scaled calibration for image size %dx%d", image_width, image_height);
            RCLCPP_INFO(this->get_logger(), "Scaling factors: x=%.4f, y=%.4f", current_scale_x_, current_scale_y_);
            RCLCPP_INFO(this->get_logger(), "Scaled intrinsics: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
                        camera_matrix_.at<double>(0, 0), camera_matrix_.at<double>(1, 1),
                        camera_matrix_.at<double>(0, 2), camera_matrix_.at<double>(1, 2));
        }
    }

    void FusionLidarCamera::debugCallback()
    {
        RCLCPP_INFO(this->get_logger(), "Debug: Node is running, waiting for synchronized messages");
        RCLCPP_INFO(this->get_logger(), "Subscribed topics: /livox/lidar, /image_raw/downsampled, /yolo/detections");
    }

    void FusionLidarCamera::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "First LiDAR point cloud received! Fusion starting...");
        _pointcloud = msg;
        _buffer_pointcloud.push_back({std::chrono::steady_clock::now(), msg});
        if (_buffer_pointcloud.size() > _buffer_size)
        {
            _buffer_pointcloud.pop_front(); // Maintain buffer size
        }
        // RCLCPP_INFO(this->get_logger(), "PointCloud2 Length: %zu", _buffer_pointcloud.size());
        // RCLCPP_INFO(this->get_logger(), "点群タイムスタンプ: %f",
        //            msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
        // 必要に応じて点群を処理
    }

    void FusionLidarCamera::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        _image = msg;
        RCLCPP_INFO_ONCE(this->get_logger(), "First image received! Fusion starting...");
        // RCLCPP_INFO(this->get_logger(), "画像タイムスタンプ: %f",
        //            msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
        // 必要に応じて画像を処理
    }

    void FusionLidarCamera::detectionCallback(const yolo_msgs::msg::DetectionArray::ConstSharedPtr msg)
    {
        _detections = msg;
        RCLCPP_INFO(this->get_logger(), "Received YOLO detections with %zu objects", msg->detections.size());
        // RCLCPP_INFO(this->get_logger(), "検出タイムスタンプ: %f",
        //            msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
        // 必要に応じて検出結果を処理
        process();
    }

    void FusionLidarCamera::process()
    {
        if (!_pointcloud || !_image || !_detections)
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for all inputs: point cloud, image, and detections");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Processing synchronized messages...");
        RCLCPP_INFO(this->get_logger(), "PointCloud2 timestamp: %f", _pointcloud->header.stamp.sec + _pointcloud->header.stamp.nanosec * 1e-9);
        RCLCPP_INFO(this->get_logger(), "Image timestamp: %f", _image->header.stamp.sec + _image->header.stamp.nanosec * 1e-9);
        RCLCPP_INFO(this->get_logger(), "Detections timestamp: %f", _detections->header.stamp.sec + _detections->header.stamp.nanosec * 1e-9);

        // imageのタイムスタンプと一番近いタイムスタンプを持つpointcloudのメッセージを探す
        auto closest_pointcloud = _buffer_pointcloud.front();
        for (const auto &timed_msg : _buffer_pointcloud)
        {
            double image_time = _image->header.stamp.sec + _image->header.stamp.nanosec * 1e-9;
            double pointcloud_time = timed_msg.msg->header.stamp.sec + timed_msg.msg->header.stamp.nanosec * 1e-9;
            if (std::abs(image_time - pointcloud_time) < std::abs(image_time - closest_pointcloud.msg->header.stamp.sec - closest_pointcloud.msg->header.stamp.nanosec * 1e-9))
            {
                closest_pointcloud = timed_msg;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Using PointCloud2 timestamp: %f",
                    closest_pointcloud.msg->header.stamp.sec + closest_pointcloud.msg->header.stamp.nanosec * 1e-9);

        // Call the main processing function
        callback(closest_pointcloud.msg, _image, _detections);
    };

    void FusionLidarCamera::callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg,
                                     const sensor_msgs::msg::Image::ConstSharedPtr &image_msg,
                                     const yolo_msgs::msg::DetectionArray::ConstSharedPtr &detections_msg)
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "First synchronized message received! Fusion starting...");

        try
        {
            // Convert ROS image to OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat image = cv_ptr->image;
            cv::Mat projected_image = image.clone();

            // Auto-scale calibration parameters based on actual image size
            scaleCalibrationParams(image.cols, image.rows);

            RCLCPP_INFO_ONCE(this->get_logger(), "Image size: %dx%d", image.cols, image.rows);

            // Convert ROS pointcloud to PCL
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*cloud_msg, *cloud);

            // Create colored point cloud
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

            // Classify points by detections (分別処理)
            auto [detection_clouds, colored_detection_clouds] =
                classifyPointsByDetections(cloud, image_msg, detections_msg, colored_cloud, projected_image);

            // Publish results and save PCD files (描画・保存処理)
            publishAndSaveResults(detection_clouds, colored_detection_clouds, colored_cloud, projected_image,
                                  cloud_msg, image_msg, detections_msg);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error in fusion callback: %s", e.what());
        }
    }

    std::pair<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>>
    FusionLidarCamera::classifyPointsByDetections(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        const sensor_msgs::msg::Image::ConstSharedPtr &image_msg,
        const yolo_msgs::msg::DetectionArray::ConstSharedPtr &detections_msg,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &colored_cloud,
        cv::Mat &projected_image)
    {
        // Convert ROS image to OpenCV for color extraction
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat image = cv_ptr->image;

        int processed_points = 0;
        int valid_projections = 0;
        int roi_projections = 0;

        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vec(detections_msg->detections.size());
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> colored_cloud_vec(detections_msg->detections.size());
        // pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_vec(detections_msg->detections.size());

        // Initialize point clouds for each detection
        for (size_t i = 0; i < detections_msg->detections.size(); ++i)
        {
            cloud_vec[i] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
            colored_cloud_vec[i] = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        }

        // Process each point
        for (const auto &point : cloud->points)
        {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
            {
                continue;
            }
            processed_points++;

            if (point.x < 0)
            {
                continue;
            }

            // Transform point from LiDAR to camera coordinate system
            Eigen::Vector4d lidar_point(point.x, point.y, point.z, 1.0);
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

            // Check if projection is within image bounds
            if (u >= 0 && u < image.cols && v >= 0 && v < image.rows)
            {
                valid_projections++;

                // Check each detection bounding box
                for (size_t detection_index = 0; detection_index < detections_msg->detections.size(); ++detection_index)
                {
                    const auto &detection = detections_msg->detections[detection_index];

                    // Calculate bounding box coordinates
                    double bbox_x_min = detection.bbox.center.position.x - detection.bbox.size.x / 2.0;
                    double bbox_x_max = detection.bbox.center.position.x + detection.bbox.size.x / 2.0;
                    double bbox_y_min = detection.bbox.center.position.y - detection.bbox.size.y / 2.0;
                    double bbox_y_max = detection.bbox.center.position.y + detection.bbox.size.y / 2.0;

                    // Check if the projected point is within the detection bounding box
                    if (u >= bbox_x_min && u <= bbox_x_max && v >= bbox_y_min && v <= bbox_y_max)
                    // if (1) // デバッグでバッグ！！
                    {
                        // Add colored point to detection-specific colored cloud
                        cv::Vec3b bgr_detection = image.at<cv::Vec3b>(v, u);

                        // Apply green color filtering for detection clouds too
                        double H_detection = rgb2hue(bgr_detection[2], bgr_detection[1], bgr_detection[0]);

                        // Only add points that are not green (Hue outside 100-140 degrees)
                        if (H_detection < 100.0 || H_detection > 140.0)
                        {
                            cloud_vec[detection_index]->points.push_back(point);

                            pcl::PointXYZRGB colored_detection_point;
                            colored_detection_point.x = point.x;
                            colored_detection_point.y = point.y;
                            colored_detection_point.z = point.z;
                            colored_detection_point.r = bgr_detection[2];
                            colored_detection_point.g = bgr_detection[1];
                            colored_detection_point.b = bgr_detection[0];

                            colored_cloud_vec[detection_index]->points.push_back(colored_detection_point);

                            // filtered_cloud_vec->points.push_back(point);
                        }

                        //////////////////////////////////////ここに注意してよーん　緑フィルター
                        // colored_cloud_vec[detection_index]->points.push_back(colored_detection_point);

                        // Draw detection bounding box on image
                        cv::rectangle(projected_image,
                                      cv::Point(static_cast<int>(bbox_x_min), static_cast<int>(bbox_y_min)),
                                      cv::Point(static_cast<int>(bbox_x_max), static_cast<int>(bbox_y_max)),
                                      cv::Scalar(0, 0, 255), 2); // Red rectangle for detection

                        // Add class label
                        cv::putText(projected_image,
                                    detection.class_name,
                                    cv::Point(static_cast<int>(bbox_x_min), static_cast<int>(bbox_y_min) - 10),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
                    }
                }

                roi_projections++;

                // Get color from image for colored point cloud
                cv::Vec3b bgr = image.at<cv::Vec3b>(v, u);

                // Create colored point using PCL's standard PointXYZRGB
                pcl::PointXYZRGB colored_point;
                colored_point.x = point.x;
                colored_point.y = point.y;
                colored_point.z = point.z;

                // RGB値を設定 (PCLはRGBを使用、BGRではない)
                colored_point.r = bgr[2];
                colored_point.g = bgr[1];
                colored_point.b = bgr[0];

                double H = rgb2hue(bgr[2], bgr[1], bgr[0]);

                if (H < 90.0 || H > 150.0)
                {
                    // colored_cloud->points.push_back(colored_point);
                }

                colored_cloud->points.push_back(colored_point);

                // Draw point on projected image
                cv::circle(projected_image, cv::Point(u, v), 2, cv::Scalar(0, 255, 0), -1);
            }
        }

        // 　ここまでですべての点群の処理が終わっている

        // Set cloud properties
        colored_cloud->width = colored_cloud->points.size();
        colored_cloud->height = 1;
        colored_cloud->is_dense = false;

        // Set properties for each detection colored cloud
        for (size_t i = 0; i < colored_cloud_vec.size(); ++i)
        {
            colored_cloud_vec[i]->width = colored_cloud_vec[i]->points.size();
            colored_cloud_vec[i]->height = 1;
            colored_cloud_vec[i]->is_dense = false;
        }

        // // Draw ROI rectangle on projected image
        // cv::rectangle(projected_image,
        //               cv::Point(roi_x_, roi_y_),
        //               cv::Point(roi_x_ + roi_width_, roi_y_ + roi_height_),
        //               cv::Scalar(255, 0, 0), 2); // Blue rectangle

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Processed %d points, %d valid projections, %d ROI projections, %zu colored points",
                             processed_points, valid_projections, roi_projections, colored_cloud->points.size());

        // //filtered_cloud_vecのパブリッシュ
        // sensor_msgs::msg::PointCloud2 filtered_cloud_msg;
        // pcl::toROSMsg(*filtered_cloud_vec, filtered_cloud_msg);
        // // filtered_cloud_msg.header = cloud_msg->header;
        // pub_colored_filtered_points_->publish(filtered_cloud_msg);

        return std::make_pair(cloud_vec, colored_cloud_vec);
    }

    // void FusionLidarCAmera::performRANSAC()
    // {
    //     // RANSAC implementation placeholder
    //     RCLCPP_INFO(this->get_logger(), "RANSAC function called - implementation pending");

    // }

    // このへんはビジュアライズ等
    void FusionLidarCamera::publishAndSaveResults(
        const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &detection_clouds,
        const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &colored_detection_clouds,
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &colored_cloud,
        const cv::Mat &projected_image,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr &image_msg,
        const yolo_msgs::msg::DetectionArray::ConstSharedPtr &detections_msg)
    {
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

        if (save_pcd_ && (pcd_counter_ % pcd_save_interval_ == 0))
        {
            save_counter_++;

            // 保存ディレクトリ
            std::string img_dir = "/home/yakan/lidar_ws/pointcloud_trunk4";
            if (!std::filesystem::exists(img_dir))
            {
                std::filesystem::create_directories(img_dir);
                RCLCPP_INFO(this->get_logger(), "Created image output directory: %s", img_dir.c_str());
            }

            // Image → cv::Mat 変換済み
            sensor_msgs::msg::Image projected_image_msg;
            cv_bridge::CvImage projected_cv_image;
            projected_cv_image.header = image_msg->header;
            projected_cv_image.encoding = sensor_msgs::image_encodings::BGR8;
            projected_cv_image.image = projected_image;
            projected_image_msg = *projected_cv_image.toImageMsg();

            // 時刻取得
            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                          now.time_since_epoch()) %
                      1000;

            // ファイル名作成
            std::stringstream filename;
            filename << img_dir << "/detection_move" << std::to_string(save_counter_)
                     << ".png";

            // 保存
            cv::imwrite(filename.str(), projected_image);
            RCLCPP_INFO(this->get_logger(), "Saved image: %s", filename.str().c_str());
        }

        // Publish point clouds for each detection and save as PCD files
        for (size_t i = 0; i < detection_clouds.size(); ++i)
        {
            if (!detection_clouds[i]->points.empty())
            {
                // Get class name for this detection
                std::string class_name = (i < detections_msg->detections.size()) ? detections_msg->detections[i].class_name : "unknown";

                // Use colored detection cloud for publishing
                sensor_msgs::msg::PointCloud2 detection_cloud_msg;
                pcl::toROSMsg(*colored_detection_clouds[i], detection_cloud_msg);
                detection_cloud_msg.header = cloud_msg->header;

                // Ensure we have enough publishers
                while (pub_detection_clouds_.size() <= i)
                {
                    std::string topic_name = "/detection_pointcloud_" + std::to_string(pub_detection_clouds_.size());
                    pub_detection_clouds_.push_back(
                        this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name, 10));
                }

                // Publish the colored point cloud
                pub_detection_clouds_[i]->publish(detection_cloud_msg);

                // Save point cloud as PCD file (if enabled and at specified interval)
                if (save_pcd_ && (pcd_counter_ % pcd_save_interval_ == 0))
                {
                    std::cout << "Saving PCD for detection " << i << " (" << class_name << ") with "
                              << colored_detection_clouds[i]->points.size() << " points." << std::endl;
                    // Create timestamp string for unique filename
                    auto now = std::chrono::system_clock::now();
                    auto time_t = std::chrono::system_clock::to_time_t(now);
                    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  now.time_since_epoch()) %
                              1000;

                    std::stringstream timestamp_ss;
                    timestamp_ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
                    timestamp_ss << "_" << std::setfill('0') << std::setw(3) << ms.count();

                    // Create output directory if it doesn't exist
                    std::string pcd_dir = "/home/yakan/lidar_ws/pointcloud_trunk4";
                    if (!std::filesystem::exists(pcd_dir))
                    {
                        std::filesystem::create_directories(pcd_dir);
                        RCLCPP_INFO(this->get_logger(), "Created PCD output directory: %s", pcd_dir.c_str());
                    }

                    std::string pcd_filename = pcd_dir + "/detection_move" + std::to_string(save_counter_) + "_" +
                                               std::to_string(i) + "_" + class_name + "_" +
                                               timestamp_ss.str() + ".pcd";

                    // Save point cloud as PCD file (binary format for efficiency)
                    if (pcl::io::savePCDFileBinary(pcd_filename, *colored_detection_clouds[i]) == 0)
                    {
                        RCLCPP_INFO(this->get_logger(),
                                    "Saved detection %zu (%s) colored point cloud to: %s (%zu points)",
                                    i, class_name.c_str(), pcd_filename.c_str(), colored_detection_clouds[i]->points.size());
                    }
                    else
                    {
                        RCLCPP_ERROR(this->get_logger(),
                                     "Failed to save PCD file: %s", pcd_filename.c_str());
                    }
                }

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Detection %zu (%s): %zu colored points published to %s", i,
                                     class_name.c_str(),
                                     colored_detection_clouds[i]->points.size(), pub_detection_clouds_[i]->get_topic_name());
            }
            else
            {
                // Ensure we have enough publishers even for empty clouds
                while (pub_detection_clouds_.size() <= i)
                {
                    std::string topic_name = "/detection_pointcloud_" + std::to_string(pub_detection_clouds_.size());
                    pub_detection_clouds_.push_back(
                        this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name, 10));
                }

                // Publish empty point cloud to clear previous data in RViz2
                sensor_msgs::msg::PointCloud2 empty_cloud_msg;
                empty_cloud_msg.header = cloud_msg->header;
                empty_cloud_msg.height = 1;
                empty_cloud_msg.width = 0;
                empty_cloud_msg.is_dense = true;
                empty_cloud_msg.is_bigendian = false;

                // Set up point cloud fields for XYZRGB
                empty_cloud_msg.fields.resize(6);

                empty_cloud_msg.fields[0].name = "x";
                empty_cloud_msg.fields[0].offset = 0;
                empty_cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
                empty_cloud_msg.fields[0].count = 1;

                empty_cloud_msg.fields[1].name = "y";
                empty_cloud_msg.fields[1].offset = 4;
                empty_cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
                empty_cloud_msg.fields[1].count = 1;

                empty_cloud_msg.fields[2].name = "z";
                empty_cloud_msg.fields[2].offset = 8;
                empty_cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
                empty_cloud_msg.fields[2].count = 1;

                empty_cloud_msg.fields[3].name = "rgb";
                empty_cloud_msg.fields[3].offset = 12;
                empty_cloud_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
                empty_cloud_msg.fields[3].count = 1;

                empty_cloud_msg.point_step = 16;
                empty_cloud_msg.row_step = 0;
                empty_cloud_msg.data.clear();

                pub_detection_clouds_[i]->publish(empty_cloud_msg);
            }
        }

        // Clear any remaining unused publishers with empty clouds
        // This handles the case where detection count decreases
        for (size_t i = detection_clouds.size(); i < pub_detection_clouds_.size(); ++i)
        {
            // Publish empty point cloud to clear previous data in RViz2
            sensor_msgs::msg::PointCloud2 empty_cloud_msg;
            empty_cloud_msg.header = cloud_msg->header;
            empty_cloud_msg.height = 1;
            empty_cloud_msg.width = 0;
            empty_cloud_msg.is_dense = true;
            empty_cloud_msg.is_bigendian = false;

            // Set up point cloud fields for XYZRGB
            empty_cloud_msg.fields.resize(4);

            empty_cloud_msg.fields[0].name = "x";
            empty_cloud_msg.fields[0].offset = 0;
            empty_cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
            empty_cloud_msg.fields[0].count = 1;

            empty_cloud_msg.fields[1].name = "y";
            empty_cloud_msg.fields[1].offset = 4;
            empty_cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
            empty_cloud_msg.fields[1].count = 1;

            empty_cloud_msg.fields[2].name = "z";
            empty_cloud_msg.fields[2].offset = 8;
            empty_cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
            empty_cloud_msg.fields[2].count = 1;

            empty_cloud_msg.fields[3].name = "rgb";
            empty_cloud_msg.fields[3].offset = 12;
            empty_cloud_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
            empty_cloud_msg.fields[3].count = 1;

            empty_cloud_msg.point_step = 16;
            empty_cloud_msg.row_step = 0;
            empty_cloud_msg.data.clear();

            pub_detection_clouds_[i]->publish(empty_cloud_msg);
        }

        // Increment PCD counter for save interval control
        if (save_pcd_)
        {
            pcd_counter_++;
        }
    }

    double FusionLidarCamera::rgb2hue(int r, int g, int b)
    {
        // 0-255 → 0-1 に正規化
        double R = r / 255.0;
        double G = g / 255.0;
        double B = b / 255.0;

        double cmax = std::max({R, G, B});
        double cmin = std::min({R, G, B});
        double delta = cmax - cmin;

        double h = 0.0;

        if (delta == 0)
        {
            h = 0.0; // 無彩色（グレー系）は Hue=0 とする
        }
        else if (cmax == R)
        {
            h = 60.0 * fmod(((G - B) / delta), 6.0);
        }
        else if (cmax == G)
        {
            h = 60.0 * (((B - R) / delta) + 2.0);
        }
        else
        { // cmax == B
            h = 60.0 * (((R - G) / delta) + 4.0);
        }

        if (h < 0)
            h += 360.0;
        return h;
    }

}