#ifndef PCL_SAC_ROS_NODE_HH_
#define PCL_SAC_ROS_NODE_HH_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "pcl_sac/pcl_sac.hh"

namespace pcl_sac
{
  class RosNode
  {
  public:
    RosNode(int max_planes = -1, bool publish_cloud = false, bool verbose = false);
    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input);
    const PclSAC &pcl_sac() const { return _pcl_sac; }

  protected:
    PclSAC _pcl_sac;
    ros::NodeHandle _nh;
    ros::Subscriber _sub_cloud;
    ros::Publisher _pub_coeffs;
    ros::Publisher _pub_colored_planes;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud{new pcl::PointCloud<pcl::PointXYZ>};
    sensor_msgs::PointCloud2 _output;
    bool _publish_cloud;
  };
}


#endif