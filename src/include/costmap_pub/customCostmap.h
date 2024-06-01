#ifndef COSTMAP_PUB__CUSTOMCOSTMAP_HPP_
#define COSTMAP_PUB__CUSTOMCOSTMAP_HPP_

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pcl/common/centroid.h"
#include "pcl/common/common.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/point_field_conversion.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <boost/foreach.hpp>
#include <chrono>
#include <iostream>
#include <math.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <queue>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#define RESET "\033[0m"
#define RED "\033[31m"
#define GREEN "\033[32m"
#define YELLOW "\033[33m"
#define BLUE "\033[34m"

using std::placeholders::_1;
using namespace std;

enum GridStates : unsigned char
{
    TRAVERSABLE = 0,
    OCCUPIED = 100,
    UNKNOWN = 255,
    LETHAL_OBSTACLE = 94,
    INSCRIBED_INFLATED_OBSTACLE = 93,
    MAX_NON_OBSTACLE = 92
};
class LaserScanToOccupancyGrid : public rclcpp::Node
{
  public:
    LaserScanToOccupancyGrid(bool publish_gnd = false);
    ~LaserScanToOccupancyGrid();
    void initializeOccupancyGrid(int scan_size);
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
    struct PriorUpdate
    {
        double time_;
        int cell_x_;
        int cell_y_;
        GridStates state_;
    };
    pcl::PointCloud<pcl::PointXYZ> transformPCL(const pcl::PointCloud<pcl::PointXYZ> &pcl_cloud);
    sensor_msgs::msg::PointCloud2 groundSupportSegmentation(pcl::PointCloud<pcl::PointXYZ> pcl_cloud, int robot_cell_x,
                                                            int robot_cell_y);
    inline void updateOccGrid(int cellX, int cellY, GridStates state);
    inline bool checkbounds(int cellX, int cellY);
    inline void inflateCostAroundCells(int cellX, int cellY, int radius, int cost);
    inline unsigned char getCostforCell(double distance) const;
    inline void updatePriorMap();
    inline void fillwithMemory();
    void updateOdom(double delta_x, double delta_y, double delta_yaw);
    void handleOdom(const nav_msgs::msg::Odometry::SharedPtr odom);
    int getGridValue(int cellX, int cellY);
    void setGridValue(int cellX, int cellY, int value);

  private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_gnd_;
    nav_msgs::msg::OccupancyGrid occupancy_grid_msg_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    nav_msgs::msg::Odometry prev_odom_{}; // Previous odometry message

    Eigen::MatrixXd prior_map_;
    const double map_resolution_ = 0.05; // Map resolution in meters
    const double gndClearance = 0.254;   // 254mm
    const double lidarShift_ = 1.235217;
    int scan_size = 25; // in meters

    // Define a hash map to keep track of cell data including their time of update and index
    map<int, PriorUpdate> prior_map_update_dict_;
};

#endif // COSTMAP_PUB__CUSTOMCOSTMAP_HPP_