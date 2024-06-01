#include "costmap_pub/customCostmap.h"
#include "rclcpp/rclcpp.hpp"

LaserScanToOccupancyGrid::LaserScanToOccupancyGrid(bool publish_gnd) : rclcpp::Node("laser_scan_to_occupancy_grid")
{
    // Subscribe to the 3D Pointcloud topic
    point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/ouster/points", rclcpp::SensorDataQoS(),
        std::bind(&LaserScanToOccupancyGrid::pointCloudCallback, this, std::placeholders::_1));
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", rclcpp::SensorDataQoS(),
        std::bind(&LaserScanToOccupancyGrid::handleOdom, this, std::placeholders::_1));

    // Create an occupancy grid and pcl publisher

    if (publish_gnd)
        pcl_pub_gnd_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_cloud_gnd", 1000);

    occupancy_grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy_grid", 1000);
    // Initialize the occupancy grid
    LaserScanToOccupancyGrid::initializeOccupancyGrid(scan_size); // Set the size of your scan
    cout << BLUE << "Occupancy grid initialized with size : " << (scan_size / map_resolution_) << RESET << endl;
}

LaserScanToOccupancyGrid::~LaserScanToOccupancyGrid()
{
    cout << YELLOW << "[+] Performing Clean up..." << RESET << endl;
}

void LaserScanToOccupancyGrid::initializeOccupancyGrid(int scan_size)
{
    // Set occupancy grid parameters (width, height, resolution, origin, etc.)
    occupancy_grid_msg_.header.frame_id = "map";
    occupancy_grid_msg_.info.width = scan_size / map_resolution_;
    occupancy_grid_msg_.info.height = scan_size / map_resolution_;
    occupancy_grid_msg_.info.resolution = map_resolution_; // Set your desired resolution
    occupancy_grid_msg_.info.origin.position.x = 0.0;
    occupancy_grid_msg_.info.origin.position.y = 0.0;
    // Initialize the occupancy grid data to all unknown (-1) values
    occupancy_grid_msg_.data.resize(occupancy_grid_msg_.info.height * occupancy_grid_msg_.info.width,
                                    GridStates::UNKNOWN);

    // Initialize the prior map to all unknown (-1) values
    prior_map_.resize(scan_size / map_resolution_, scan_size / map_resolution_);
    prior_map_.setConstant(GridStates::UNKNOWN);
}

pcl::PointCloud<pcl::PointXYZ> LaserScanToOccupancyGrid::transformPCL(pcl::PointCloud<pcl::PointXYZ> pcl_cloud)
{
    // Transform the 3d pointcloud such that the robot is at the origin

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud_transformed;
    for (size_t i = 0; i < pcl_cloud.size(); ++i)
    {
        pcl::PointXYZ p;
        p.x = pcl_cloud[i].x;
        p.y = pcl_cloud[i].y;
        p.z = pcl_cloud[i].z + lidarShift_;
        pcl_cloud_transformed.push_back(p);
    }
    return pcl_cloud_transformed;
}

bool LaserScanToOccupancyGrid::checkbounds(int cellX, int cellY)
{
    if (cellX < 0 || cellX >= occupancy_grid_msg_.info.width || cellY < 0 || cellY >= occupancy_grid_msg_.info.height)
    {
        return false;
    }
    return true;
}

unsigned char LaserScanToOccupancyGrid::getCostforCell(double distance) const
{
    double inscribed_radius_ = 0.1;
    double cost_scaling_factor_ = 2.0; // A higher value means the cost decays faster with distance
    unsigned char cost = 0;

    if (distance == 0.0)
        cost = GridStates::LETHAL_OBSTACLE;
    else if (distance * map_resolution_ <= inscribed_radius_)
        cost = GridStates::INSCRIBED_INFLATED_OBSTACLE;
    else
    {
        // Make sure that the cost decays by euclidean distance
        double factor = exp(-1.0 * cost_scaling_factor_ * (distance * map_resolution_ - inscribed_radius_));
        cost = static_cast<unsigned char>((GridStates::INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
    }
    return cost;
}

void LaserScanToOccupancyGrid::inflateCostAroundCells(int cellX, int cellY, int radius, int cost)
{
    // Iterate through the cells around the occupied cell within the inflation radius
    for (int i = -radius; i <= radius; ++i)
    {
        for (int j = -radius; j <= radius; ++j)
        {
            int neighbor_x = cellX + i;
            int neighbor_y = cellY + j;
            // Check if the neighbor cell indices are within the grid bounds
            if (LaserScanToOccupancyGrid::checkbounds(neighbor_x, neighbor_y))
            {
                int current_cost = occupancy_grid_msg_.data[neighbor_y * occupancy_grid_msg_.info.width + neighbor_x];

                // cout << "Current cost: " << current_cost << endl;
                if (current_cost == GridStates::OCCUPIED)
                {
                    // Calculate the distance from an obstacle in cells
                    double distance = std::sqrt(std::pow(neighbor_x - cellX, 2) + std::pow(neighbor_y - cellY, 2));
                    occupancy_grid_msg_.data[neighbor_y * occupancy_grid_msg_.info.width + neighbor_x] =
                        GridStates(LaserScanToOccupancyGrid::getCostforCell(distance));
                }
                else if (current_cost == GridStates::TRAVERSABLE || current_cost == GridStates::UNKNOWN ||
                         current_cost == -1)
                {
                    // cout << "Occupied cell" << endl;
                    // map_(neighbor_y, neighbor_x) = max_inflation_cost;
                    occupancy_grid_msg_.data[neighbor_y * occupancy_grid_msg_.info.width + neighbor_x] = cost;
                }
            }
        }
    }
}

void LaserScanToOccupancyGrid::updateOccGrid(int cellX, int cellY, GridStates state)
{
    if (LaserScanToOccupancyGrid::checkbounds(cellX, cellY))
    {
        switch (state)
        {
        case GridStates::OCCUPIED: {
            occupancy_grid_msg_.data[cellY * occupancy_grid_msg_.info.width + cellX] = GridStates::OCCUPIED;
            LaserScanToOccupancyGrid::inflateCostAroundCells(cellX, cellY, 5, GridStates::OCCUPIED);
            break;
        }
        case GridStates::TRAVERSABLE: {
            occupancy_grid_msg_.data[cellY * occupancy_grid_msg_.info.width + cellX] = GridStates::TRAVERSABLE;
            LaserScanToOccupancyGrid::inflateCostAroundCells(cellX, cellY, 6, GridStates::TRAVERSABLE);
            break;
        }
        case GridStates::UNKNOWN: {
            occupancy_grid_msg_.data[cellY * occupancy_grid_msg_.info.width + cellX] = GridStates::UNKNOWN;
            break;
        }
        default:
            break;
        }
    }
}

sensor_msgs::msg::PointCloud2 LaserScanToOccupancyGrid::groundSupportSegmentation(
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud, int robot_cell_x, int robot_cell_y)
{
    GridStates state;
    sensor_msgs::msg::PointCloud2 point_cloud_2d;
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud_2d;
    for (size_t i = 0; i < pcl_cloud.size(); ++i)
    {
        // Extract the 3D point
        pcl::PointXYZRGB pointrgb;
        pointrgb.x = pcl_cloud[i].x;
        pointrgb.y = pcl_cloud[i].y;
        pointrgb.z = pcl_cloud[i].z;

        int cell_x = robot_cell_x + floor(pointrgb.x / map_resolution_);
        int cell_y = robot_cell_y + floor(pointrgb.y / map_resolution_);
        state = GridStates::UNKNOWN;
        LaserScanToOccupancyGrid::updateOccGrid(cell_x, cell_y, state);
        prior_map_update_dict_.insert({i, PriorUpdate{0.0, cell_x, cell_y, state}});

        // check if consecutive points have a difference of less than 0.15 in z
        if (std::abs(pcl_cloud[i].z) <= gndClearance)
        {
            // Ground Support
            pointrgb.r = 0;
            pointrgb.g = 255;
            pointrgb.b = 0;
            state = GridStates::TRAVERSABLE;
            LaserScanToOccupancyGrid::updateOccGrid(cell_x, cell_y, state);
            prior_map_update_dict_.insert({i, PriorUpdate{0.0, cell_x, cell_y, state}});
        }
        else
        {
            // Non Ground Support
            pointrgb.r = 255;
            pointrgb.g = 0;
            pointrgb.b = 0;
            state = GridStates::OCCUPIED;
            LaserScanToOccupancyGrid::updateOccGrid(cell_x, cell_y, state);
            prior_map_update_dict_.insert({i, PriorUpdate{0.0, cell_x, cell_y, state}});
        }
        // Add the 2D point to the PCL PointCloud
        pcl_cloud_2d.push_back(pointrgb);
    }
    pcl::toROSMsg(pcl_cloud_2d, point_cloud_2d);
    return point_cloud_2d;
}

void LaserScanToOccupancyGrid::updatePriorMap()
{
    bool printFlag = false;
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());

    for (auto &entry : prior_map_update_dict_)
    {
        auto timeElapsed = ms.count() - entry.second.time_;
        if (LaserScanToOccupancyGrid::checkbounds(entry.second.cell_x_, entry.second.cell_y_) && timeElapsed >= 2000)
        {
            prior_map_(entry.second.cell_y_, entry.second.cell_x_) = static_cast<int>(
                occupancy_grid_msg_.data[entry.second.cell_y_ * occupancy_grid_msg_.info.width + entry.second.cell_x_]);
            entry.second.time_ = ms.count();
            printFlag = true;
        }
    }
    if (printFlag)
        cout << "[+] Prior map updated " << endl;
}

void LaserScanToOccupancyGrid::fillwithMemory()
{
    bool printFlag = false;
    for (auto &entry : prior_map_update_dict_)
    {
        if (LaserScanToOccupancyGrid::checkbounds(entry.second.cell_x_, entry.second.cell_y_))
        {
            int occState =
                occupancy_grid_msg_.data[entry.second.cell_y_ * occupancy_grid_msg_.info.width + entry.second.cell_x_];
            int priorState = prior_map_(entry.second.cell_y_, entry.second.cell_x_);
            if ((occState == -1 && priorState != -1) ||
                (occState == GridStates::UNKNOWN && priorState != GridStates::UNKNOWN) ||
                (occState == 255 && priorState != 255))
            {
                // cout << "Occupancy state Before : " << occState << " | "
                //           << "Prior state : " << priorState << " | ";
                occupancy_grid_msg_.data[entry.second.cell_y_ * occupancy_grid_msg_.info.width + entry.second.cell_x_] =
                    priorState;
                // cout << "Occupancy state After : "
                //           << int(occupancy_grid_msg_
                //                      .data[entry.second.cell_y_ * occupancy_grid_msg_.info.width +
                //                      entry.second.cell_x_])
                //           << endl;

                printFlag = true;
            }
        }
    }
    if (printFlag)
        cout << "[+] Filled with memory" << endl;
}

void LaserScanToOccupancyGrid::updateOdom(double delta_x, double delta_y, double delta_yaw)
{
    Eigen::MatrixXd tempMap;
    tempMap.resize(occupancy_grid_msg_.info.height, occupancy_grid_msg_.info.width);
    tempMap.setConstant(GridStates::UNKNOWN);

    // convert the change in position from meters to cell indices
    double delta_x_cell = delta_x / map_resolution_;
    double delta_y_cell = delta_y / map_resolution_;
    double delta_yaw_cos = cos(delta_yaw);
    double delta_yaw_sin = sin(delta_yaw);

    //   convert these values into a transformation matrix.
    Eigen::MatrixXd transform(3, 3);
    transform << delta_yaw_cos, -delta_yaw_sin, delta_x_cell, delta_yaw_sin, delta_yaw_cos, delta_y_cell, 0.0, 0.0, 1.0;

    //    Iterate through the occupancy grid and transform each cell
    for (auto i = 0; i < occupancy_grid_msg_.info.height; ++i)
    {
        for (auto j = 0; j < occupancy_grid_msg_.info.width; ++j)
        {
            Eigen::MatrixXd cell(3, 1);
            cell << j - occupancy_grid_msg_.info.height / 2, i - occupancy_grid_msg_.info.width / 2,
                1.0; // Transform the cell indices to the center of the grid

            Eigen::MatrixXd transformed_cell = transform * cell;

            transformed_cell(0) += occupancy_grid_msg_.info.height / 2;
            transformed_cell(1) += occupancy_grid_msg_.info.width / 2;

            // Check if the transformed cell indices are within the grid bounds
            if (LaserScanToOccupancyGrid::checkbounds(int(transformed_cell(0)), int(transformed_cell(1))))
            {
                // Update the temporary map with the cell data
                tempMap(int(transformed_cell(0)), int(transformed_cell(1))) =
                    static_cast<int>(occupancy_grid_msg_.data[i * occupancy_grid_msg_.info.width + j]);
            }
        }
    }

    // Update the occupancy grid with the temporary map
    for (auto i = 0; i < occupancy_grid_msg_.info.height; ++i)
    {
        for (auto j = 0; j < occupancy_grid_msg_.info.width; ++j)
        {
            occupancy_grid_msg_.data[i * occupancy_grid_msg_.info.width + j] = static_cast<int>(tempMap(i, j));
        }
    }
}

void LaserScanToOccupancyGrid::handleOdom(const nav_msgs::msg::Odometry::SharedPtr odom)
{
    //   cout << "Odom callback" << endl;
    // transform based on current and previous odom data
    double delta_x = odom->pose.pose.position.x - prev_odom_.pose.pose.position.x;
    double delta_y = odom->pose.pose.position.y - prev_odom_.pose.pose.position.y;
    double delta_yaw = 0.0;

    LaserScanToOccupancyGrid::updateOdom(delta_x, delta_y, delta_yaw);
    prev_odom_ = *odom;

    //   Fill the message with the occupancy grid map and publish it
    //   occupancy_grid_msg_.header = cloud_msg->header;
    occupancy_grid_msg_.header.frame_id = "os_lidar";
    occupancy_grid_msg_.header.stamp = rclcpp::Clock().now();
    occupancy_grid_msg_.info.width = scan_size / map_resolution_;
    occupancy_grid_msg_.info.height = scan_size / map_resolution_;
    occupancy_grid_msg_.info.resolution = map_resolution_; // Set your desired resolution
    occupancy_grid_msg_.info.origin.position.x = 0.0;
    occupancy_grid_msg_.info.origin.position.y = 0.0;
    occupancy_grid_publisher_->publish(occupancy_grid_msg_);
}

void LaserScanToOccupancyGrid::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
    auto start = std::chrono::high_resolution_clock::now();
    //   cout << BLUE << "Point cloud callback..." << RESET << endl;

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*cloud_msg, pcl_cloud);                        // Convert from ROS message to PCL point cloud
    pcl_cloud = LaserScanToOccupancyGrid::transformPCL(pcl_cloud); // Transform the point cloud to the 0,0

    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud_2d; // Create a PCL PointCloud for 2D points
    sensor_msgs::msg::PointCloud2 point_cloud_2d;   // Create a PointCloud2 message for 2D points with RGB color
    pcl::toROSMsg(pcl_cloud_2d, point_cloud_2d);    // Create a PointCloud2 message for 2D points with intensity

    std::fill(occupancy_grid_msg_.data.begin(), occupancy_grid_msg_.data.end(), GridStates::UNKNOWN);

    // Calculate the grid cell indices for the robot's position
    int robot_cell_x = occupancy_grid_msg_.info.width / 2;
    int robot_cell_y = occupancy_grid_msg_.info.height / 2;

    // point_cloud_2d = updateGrid(pcl_cloud);
    // point_cloud_2d = groundSegmentation(pcl_cloud,robot_cell_x,robot_cell_y);
    // point_cloud_2d_gnd = PCAgndSeg(pcl_cloud,pts_z);
    // point_cloud_2d = naiveSegmentation(pcl_cloud,robot_cell_x,robot_cell_y);
    point_cloud_2d = LaserScanToOccupancyGrid::groundSupportSegmentation(pcl_cloud, robot_cell_x, robot_cell_y);
    updatePriorMap();
    fillwithMemory();

    // Publish the 2D point cloud
    if (pcl_pub_gnd_)
    {
        point_cloud_2d.header = cloud_msg->header;
        point_cloud_2d.header.frame_id = cloud_msg->header.frame_id;
        point_cloud_2d.header.stamp = cloud_msg->header.stamp;
        point_cloud_2d.height = 1;
        point_cloud_2d.width = pcl_cloud.size();
        point_cloud_2d.is_dense = true;
        point_cloud_2d.is_bigendian = false;
        pcl_pub_gnd_->publish(point_cloud_2d);
    }

    // Initialize the occupancy grid data to all unknown (-1) values
    occupancy_grid_msg_.header = cloud_msg->header;
    occupancy_grid_msg_.header.frame_id = cloud_msg->header.frame_id;
    occupancy_grid_msg_.header.stamp = rclcpp::Clock().now();
    occupancy_grid_msg_.info.width = scan_size / map_resolution_;
    occupancy_grid_msg_.info.height = scan_size / map_resolution_;
    occupancy_grid_msg_.info.resolution = map_resolution_; // Set your desired resolution
    occupancy_grid_msg_.info.origin.position.x = 0.0;
    occupancy_grid_msg_.info.origin.position.y = 0.0;
    occupancy_grid_publisher_->publish(occupancy_grid_msg_);

    //   cout << BLUE << "Point cloud callback finished..." << RESET << endl;
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    cout << GREEN << "Time taken for computation: " << duration.count() << " ms" << RESET << endl;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    cout << BLUE << "[+] Starting custom Occupancy node..." << RESET << endl;
    auto node = std::make_shared<LaserScanToOccupancyGrid>(false);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
