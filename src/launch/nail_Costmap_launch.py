from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    customScanNode = Node(
        package="costmap_pub",
        executable="customScan",
        name="customScan",
        output="screen",
    )

    customOccupancyGridNode = Node(
        package="costmap_pub",
        executable="customCostmap",
        name="customCostmap",
        output="screen",
    )
    rosbag_play = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            "--l",
            "--read-ahead-queue-size 5000",
            "Bag_Files/Tue_Oct-03-2023_12-46-44_PM/Tue_Oct-03-2023_12-46-44_PM_0.mcap",
            "--clock",
        ],
        shell=True,
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            "config/custom_occ_map.rviz",
        ],
    )

    return LaunchDescription(
        [
            rosbag_play,
            # customScanNode,
            customOccupancyGridNode,
            rviz2,
        ]
    )
