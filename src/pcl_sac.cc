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

namespace pcl_sac
{
  class PclSAC
  {
  public:
    using color_cloud_t = pcl::PointCloud<pcl::PointXYZRGB>;
    using cloud_t = pcl::PointCloud<pcl::PointXYZ>;
    PclSAC(int max_planes = -1, bool verbose = false);

    void update(cloud_t::Ptr cloud);
    // list of planes
    const std::vector<color_cloud_t> &planes() const { return _planes; }
    // the full cloud, with a different color for each plane
    const pcl::PointCloud<pcl::PointXYZRGB> &colored_cloud() const { return *_colored_cloud; };
    // the plane coefficients
    const std::vector<pcl::ModelCoefficients> &planes_coefficients() const { return _planes_coefficients; }

  protected:
    int _max_planes = -1;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> _planes;
    std::vector<pcl::ModelCoefficients> _planes_coefficients;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _colored_cloud{new pcl::PointCloud<pcl::PointXYZRGB>};

    using color_t = std::tuple<uint32_t, uint32_t, uint32_t>;
    std::vector<std::pair<pcl::ModelCoefficients, color_t>> _color_map;
    pcl::SACSegmentation<pcl::PointXYZ> _seg;
    pcl::VoxelGrid<pcl::PointXYZ> _resampler;
    pcl::ExtractIndices<pcl::PointXYZ> _extractor;

    pcl::PointIndices::Ptr _inliers{new pcl::PointIndices};
    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud_filtered{new pcl::PointCloud<pcl::PointXYZ>};
    pcl::PointCloud<pcl::PointXYZ>::Ptr _tmp_cloud{new pcl::PointCloud<pcl::PointXYZ>};

    /// random generator for random colors
    std::mt19937 _rd;
    std::uniform_int_distribution<uint32_t> _rand{0, 255};
    // find the closest plan from the color_map
    color_t _get_color(const pcl::ModelCoefficients &coeffs);

    pcl_sac::Timer _timer;
    bool _verbose;
  };
}
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

namespace pcl_sac
{
  RosNode::RosNode(int max_planes, bool publish_cloud, bool verbose) : _pcl_sac(max_planes, verbose),
                                                         _publish_cloud(publish_cloud)
  {
    // you can/should remap input to to the realsense/xtion/etc.
    _sub_cloud = _nh.subscribe("input", 1, &RosNode::cloud_cb, this);
    _pub_coeffs = _nh.advertise<pcl_msgs::ModelCoefficients>("plane_coeffs", 1);
    if (_publish_cloud) // for debugging / visualization
      _pub_colored_planes = _nh.advertise<sensor_msgs::PointCloud2>("colored_planes", 1);
  }

  void RosNode::cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)
  {
    pcl::fromROSMsg(*input, *_cloud);
    _pcl_sac.update(_cloud);
    //       timer.begin("convert");
    if (_publish_cloud)
    {
      pcl::toROSMsg(_pcl_sac.colored_cloud(), _output);
      _output.header.stamp = ros::Time::now();
      _output.header.frame_id = input->header.frame_id;

      _pub_colored_planes.publish(_output);
    }
    // timer.end("convert");
  }
}

namespace pcl_sac
{
  PclSAC::PclSAC(int max_planes, bool verbose) : _max_planes(max_planes), _verbose(verbose)
  {
    // resampling
    _resampler.setLeafSize(0.10f, 0.10f, 0.05f);

    // segmentation object
    _seg.setOptimizeCoefficients(true);
    _seg.setModelType(pcl::SACMODEL_PLANE);
    _seg.setMethodType(pcl::SAC_RANSAC);
    _seg.setMaxIterations(2000);
    _seg.setDistanceThreshold(0.06);
    _seg.setNumberOfThreads(8);

    // extract a plane from the cloud
    _extractor.setNegative(true);
  }

  void PclSAC::update(const cloud_t::Ptr cloud)
  {
    // clear the results
    _planes.clear();
    _planes_coefficients.clear();
    _colored_cloud->clear();
    _rd.seed(0);

    // downsample
    _timer.begin("downsampling");
    _resampler.setInputCloud(cloud);
    _resampler.setLeafSize(0.10f, 0.10f, 0.05f);
    _resampler.filter(*_cloud_filtered);
    _timer.end("downsampling");

    // main loop
    _timer.begin("ransac");
    size_t n_points = _cloud_filtered->size();
    while (_cloud_filtered->size() > 0.25 * n_points && (_max_planes == -1 || _planes.size() < _max_planes))
    {
      pcl::ModelCoefficients coefficients;
      _seg.setInputCloud(_cloud_filtered);
      _seg.segment(*_inliers, coefficients);

      _planes_coefficients.push_back(coefficients);

      pcl::PointCloud<pcl::PointXYZRGB> plane;

      auto color = _get_color(coefficients);
      // give the color to the point
      for (const auto &idx : _inliers->indices)
      {
        pcl::PointXYZRGB p(std::get<0>(color), std::get<1>(color), std::get<2>(color));
        p.x = _cloud_filtered->points[idx].x;
        p.y = _cloud_filtered->points[idx].y;
        p.z = _cloud_filtered->points[idx].z;

        // add the point to the plane
        plane.push_back(p);
      }
      // add the plane to the list of planes
      (*_colored_cloud) += plane;
      _planes.push_back(plane);

      // rempove the plane from the cloud
      _extractor.setInputCloud(_cloud_filtered);
      _extractor.setIndices(_inliers);
      _extractor.filter(*_tmp_cloud);
      _tmp_cloud.swap(_cloud_filtered);
    }
    _timer.end("ransac");
    if (_verbose)
      _timer.report(std::cout, 0, 10);
  }

  color_t PclSAC::_get_color(const pcl::ModelCoefficients &coeffs)
  {
    // find a color
    int best_k = -1;
    double best_dist = std::numeric_limits<double>::max();
    for (size_t k = 0; k < _color_map.size(); ++k)
    {
      // compute distance
      double d = 0;
      for (size_t i = 0; i < coeffs.values.size(); ++i)
        d += (coeffs.values[i] - _color_map[k].first.values[i]) * (coeffs.values[i] - _color_map[k].first.values[i]);
      d = sqrt(d);
      if (d < best_dist)
      {
        best_k = k;
        best_dist = d;
      }
    }
    if (best_k == -1 || best_dist > 0.3)
    {
      color_t c = std::make_tuple(_rand(_rd), _rand(_rd), _rand(_rd));
      _color_map.push_back(std::make_pair(coeffs, c));
      return c;
    }
    else
      return _color_map[best_k].second;
  }

} // namespace

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
  seg.setMaxIterations(2000);
  seg.setDistanceThreshold(0.05);
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
      if (d < best_dist)
      {
        best_k = k;
        best_dist = d;
      }
    }
    // std::cout<<best_dist<<"  "<<best_k<<std::endl;
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
  // ros::NodeHandle nh;

  // // Create a ROS subscriber for the input point cloud
  // ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);

  // // Create a ROS publisher for the output model coefficients
  // pub_coeffs = nh.advertise<pcl_msgs::ModelCoefficients>("plane_coeffs", 1);
  // pub_colored_planes = nh.advertise<sensor_msgs::PointCloud2>("colored_planes", 1);

  // std::cout << "topic created" << std::endl;

  pcl_sac::RosNode ros_node(-1, true);
  // Spin
  ros::spin();
}