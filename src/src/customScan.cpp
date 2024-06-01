#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <pcl/common/transforms.h>
#include <vector>

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
class PointCloudToScan : public rclcpp::Node {
public:
    PointCloudToScan() : Node("point_cloud_to_scan") {
        // Subscribe to the 3D point cloud topic
        point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/ouster/points", rclcpp::SensorDataQoS(),
            std::bind(&PointCloudToScan::pointCloudCallback, this, std::placeholders::_1));

        // Create a 2D scan publisher
        scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
        // Convert the PointCloud2 message to a PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(*cloud_msg, pcl_cloud);

        // Create a 2D scan
        sensor_msgs::msg::LaserScan scan;
        scan.header = cloud_msg->header;

        // Set the scan properties (adjust as needed)
        scan.angle_min = -M_PI;
        scan.angle_max = M_PI;
        scan.angle_increment = M_PI / 360.0;
        scan.time_increment = 1/10240;
        scan.range_min = 0.10000000149011612;
        scan.scan_time = 0.10000000149011612;
        scan.range_max = 120.0;
        // Populate the scan data by projecting the 3D points to 2D
        scan.ranges.resize(pcl_cloud.size());
        for (size_t i = 0; i < pcl_cloud.size(); ++i) {
            scan.ranges[i] = hypot(pcl_cloud[i].x,pcl_cloud[i].y);
        }

        // Publish the 2D scan
        scan_publisher_->publish(scan);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::cout << GREEN << "Starting point_cloud_to_scan node..." << RESET << std::endl;
    auto node = std::make_shared<PointCloudToScan>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
