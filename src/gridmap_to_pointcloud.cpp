#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class GridmapToPointcloud
{
public:
  GridmapToPointcloud(void) : private_nh_("~")
  {
    private_nh_.param<int>("hz", hz_, 1);
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/map_cloud", 1);
  }

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

  void convert_gridmap_to_pointcloud(const nav_msgs::OccupancyGrid &gridmap, sensor_msgs::PointCloud2 &cloud_msg)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::toROSMsg(cloud, cloud_msg);
  }

  void process(void)
  {
    ros::Rate rate(hz_);
    const nav_msgs::OccupancyGrid gridmap = get_map();
    sensor_msgs::PointCloud2 cloud_msg;
    convert_gridmap_to_pointcloud(gridmap, cloud_msg);
    cloud_msg.header.frame_id = gridmap.header.frame_id;
    cloud_msg.header.stamp = ros::Time::now();

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
