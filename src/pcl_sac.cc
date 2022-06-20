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

// refs :
// ROS: http://wiki.ros.org/pcl/Tutorials
// PCL: https://pcl.readthedocs.io/projects/tutorials/en/latest/planar_segmentation.html
// https://stackoverflow.com/questions/44921987/removing-points-from-a-pclpointcloudpclpointxyzrgb

ros::Publisher pub_coeffs;
ros::Publisher pub_colored_planes;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*input, cloud);

  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  // Optional
  seg.setOptimizeCoefficients(true);

  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);

  seg.setInputCloud(cloud.makeShared());
  seg.segment(inliers, coefficients);

  // Publish the model coefficients
  pcl_msgs::ModelCoefficients ros_coefficients;
  pcl_conversions::fromPCL(coefficients, ros_coefficients);
  pub_coeffs.publish(ros_coefficients);

  pcl::PointCloud<pcl::PointXYZRGB> plane;
//  plane.resize(inliers.indices.size());

  std::cerr << "Model inliers: " << inliers.indices.size() << std::endl;
  for (const auto &idx : inliers.indices) {
      pcl::PointXYZRGB p(255, 0, 0);
      p.x = cloud.points[idx].x;
      p.y = cloud.points[idx].y;
      p.z = cloud.points[idx].z;
      
      plane.push_back(p);
  }

  // //https://pointclouds.org/documentation/classpcl_1_1_extract_indices_3_01pcl_1_1_p_c_l_point_cloud2_01_4.html
  // pcl::ExtractIndices<pcl::PointXYZ> extract;
  // extract.setInputCloud(&cloud);
  // extract.setIndices(inliers);
  // extract.setNegative(true);
  // pcl::PointCloud<pcl::PointXYZ> cloud_out;
  // extract.filter(cloud_out);
  

  std::cout<<"converting..."<<std::endl;
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(plane, output);
  output.header.stamp = ros::Time::now();
  output.header.frame_id = input->header.frame_id;
  pub_colored_planes.publish (output);
  std::cerr << "published: " << plane.size() << std::endl;

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
  pub_colored_planes = nh.advertise<sensor_msgs::PointCloud2> ("colored_planes", 1);

  std::cout << "topic created" << std::endl;
  // Spin
  ros::spin();
}