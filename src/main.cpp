#include "fusion_lidar_camera/fusion_node.hpp"

using namespace fusion_lidar_camera;
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FusionLidarCamera>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
