/**
 * @file gridmap_to_pointcloud.cpp
 * @author Toshiki Nakamura
 * @brief Convert gridmap to pointcloud
 * @copyright Copyright (c) 2024
 */

#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

/**
 * @class GridmapToPointcloud
 * @brief Class to convert gridmap to pointcloud
 */
class GridmapToPointcloud
{
public:
  /**
   * @brief Construct a new Gridmap To Pointcloud object
   */
  GridmapToPointcloud(void) : private_nh_("~")
  {
    private_nh_.param<int>("hz", hz_, 1);
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/map_cloud", 1);
    ROS_INFO_STREAM(ros::this_node::getName() << " node has started..");
    ROS_INFO_STREAM("hz: " << hz_);
  }

  /**
   * @brief Client function to get map
   */
  nav_msgs::OccupancyGrid get_map(void)
  {
    nav_msgs::GetMap::Request req;
    nav_msgs::GetMap::Response resp;
    while (ros::ok() && !ros::service::call("/static_map", req, resp))
    {
      ROS_WARN_THROTTLE(2.0, "Waiting for a map");
      ros::Duration(0.5).sleep();
    }
    ROS_WARN("Received a map");

    return resp.map;
  }

  /**
   * @brief Convert gridmap to pointcloud
   * @param gridmap Gridmap message
   * @param cloud_msg Pointcloud message
   */
  void convert_gridmap_to_pointcloud(const nav_msgs::OccupancyGrid &gridmap, sensor_msgs::PointCloud2 &cloud_msg)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.points.reserve(gridmap.data.size() / 2);

    for (int i = 0; i < gridmap.data.size(); i++)
    {
      if (gridmap.data[i] == 100)
      {
        pcl::PointXYZ point;
        const int grid_index_x = i % gridmap.info.width;
        const int grid_index_y = static_cast<int>(i / gridmap.info.width);
        point.x =
            grid_index_x * gridmap.info.resolution + gridmap.info.origin.position.x + gridmap.info.resolution / 2.0;
        point.y =
            grid_index_y * gridmap.info.resolution + gridmap.info.origin.position.y + gridmap.info.resolution / 2.0;
        point.z = 0.0;
        cloud.points.push_back(point);
      }
    }
    pcl::toROSMsg(cloud, cloud_msg);
  }

  /**
   * @brief Process function
   */
  void process(void)
  {
    const nav_msgs::OccupancyGrid gridmap = get_map();
    sensor_msgs::PointCloud2 cloud_msg;
    convert_gridmap_to_pointcloud(gridmap, cloud_msg);
    cloud_msg.header.frame_id = gridmap.header.frame_id;
    cloud_msg.header.stamp = ros::Time::now();

    ros::Rate rate(hz_);
    while (ros::ok())
    {
      cloud_pub_.publish(cloud_msg);
      rate.sleep();
    }
  }

private:
  int hz_;
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher cloud_pub_;
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "gridmap_to_pointcloud");
  GridmapToPointcloud gridmap_to_pointcloud;
  gridmap_to_pointcloud.process();

  return 0;
}
