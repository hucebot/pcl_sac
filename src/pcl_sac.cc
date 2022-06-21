#include <random>
#include <limits>

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>

#include "pcl_sac/timer.hpp"

// refs :
// ROS: http://wiki.ros.org/pcl/Tutorials
// PCL: https://pcl.readthedocs.io/projects/tutorials/en/latest/planar_segmentation.html
// https://pcl.readthedocs.io/en/latest/extract_indices.html
ros::Publisher pub_coeffs;
ros::Publisher pub_colored_planes;

using color_t = std::tuple<uint32_t, uint32_t, uint32_t>;
std::vector<std::pair<pcl::ModelCoefficients, color_t>> color_map;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)
{
  pcl_sac::Timer timer;

  /// random generator for random colors
  std::mt19937 rd;
  rd.seed(0);
  std::uniform_int_distribution<uint32_t> rand_color(0, 255);

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  timer.begin("convert");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::fromROSMsg(*input, *cloud);
  timer.end("convert");

  // downsample
  timer.begin("downsample");
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(0.10f, 0.10f, 0.05f);
  sor.filter(*cloud_filtered);

  std::cout << "Downsampling from:" << cloud->size() << " to " << cloud_filtered->size() << std::endl;
  cloud_filtered.swap(cloud);
  timer.end("downsample");

  // setup SAC
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(0.01);
  seg.setNumberOfThreads(8);

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>> planes;
  size_t n_points = cloud->size();
  while (cloud->size() > 0.25 * n_points)
  {
    // call the segmentation
    timer.begin("sac");
    seg.setInputCloud(cloud);
    seg.segment(*inliers, coefficients);
    if (inliers->indices.size() == 0)
    {
      std::cerr << "Could not estimate a planar model!" << std::endl;
      break;
    }
    timer.end("sac");

    // Publish the model coefficients
    // pcl_msgs::ModelCoefficients ros_coefficients;
    // pcl_conversions::fromPCL(coefficients, ros_coefficients);
    // pub_coeffs.publish(ros_coefficients);

    // make a new plane
    timer.begin("store");
    pcl::PointCloud<pcl::PointXYZRGB> plane;
    std::vector<uint32_t> color;
    // find a color
    int best_k = -1;
    double best_dist = std::numeric_limits<double>::max();
    for (size_t k = 0; k < color_map.size(); ++k)
    {
      // compute distance
      double d = 0;
      for (size_t i = 0; i < coefficients.values.size(); ++i)
        d += (coefficients.values[i] - color_map[k].first.values[i]) * (coefficients.values[i] - color_map[k].first.values[i]);
      d = sqrt(d);
      if (d < best_dist) {
        best_k = k;
        best_dist = d;
      }
    }
    //std::cout<<best_dist<<"  "<<best_k<<std::endl;
    if (best_k == -1 || best_dist > 0.3)
    {
      color = {rand_color(rd), rand_color(rd), rand_color(rd)};
      color_t c = std::make_tuple(color[0], color[1], color[2]);
     color_map.push_back(std::make_pair(coefficients, c));
    }
    else
      color = std::vector<uint32_t>({std::get<0>(color_map[best_k].second),
               std::get<1>(color_map[best_k].second),
               std::get<2>(color_map[best_k].second)});

    // give the color to the point
    for (const auto &idx : inliers->indices)
    {
      pcl::PointXYZRGB p(color[0], color[1], color[2]);
      p.x = cloud->points[idx].x;
      p.y = cloud->points[idx].y;
      p.z = cloud->points[idx].z;

      plane.push_back(p);
    }
    (*colored_cloud) += plane;
    planes.push_back(plane);
    timer.end("store");

    // remove the plane for the next loop
    timer.begin("filter");
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_filtered);
    cloud_filtered.swap(cloud);
    timer.end("filter");
  }
  std::cout << "Number of planes:" << planes.size() << std::endl;

  timer.begin("convert");
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*colored_cloud, output);
  output.header.stamp = ros::Time::now();
  output.header.frame_id = input->header.frame_id;
  pub_colored_planes.publish(output);
  timer.end("convert");

  timer.report();
}

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "pcl_ransac");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);

  // Create a ROS publisher for the output model coefficients
  pub_coeffs = nh.advertise<pcl_msgs::ModelCoefficients>("plane_coeffs", 1);
  pub_colored_planes = nh.advertise<sensor_msgs::PointCloud2>("colored_planes", 1);

  std::cout << "topic created" << std::endl;
  // Spin
  ros::spin();
}