#include <limits>
#include "pcl_sac/timer.hh"
#include "pcl_sac/pcl_sac.hh"
// refs :
// ROS: http://wiki.ros.org/pcl/Tutorials
// PCL: https://pcl.readthedocs.io/projects/tutorials/en/latest/planar_segmentation.html
// https://pcl.readthedocs.io/en/latest/extract_indices.html


namespace pcl_sac
{
  PclSAC::PclSAC(const Params& params) : _params(params)
  {
    // resampling
    _resampler.setLeafSize(_params.leaf_size[0], _params.leaf_size[1],_params.leaf_size[2]);

    // segmentation object
    _seg.setOptimizeCoefficients(true);
    _seg.setModelType(pcl::SACMODEL_PLANE);
    _seg.setMethodType(pcl::SAC_RANSAC);
    _seg.setMaxIterations(_params.max_iterations);
    _seg.setDistanceThreshold(_params.distance_threshold);
    _seg.setNumberOfThreads(_params.num_threads);

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
    _resampler.filter(*_cloud_filtered);
    _timer.end("downsampling");

    if (_params.verbose > 1)
      std::cout<<"downsampling from " << cloud->size() << " to " << _cloud_filtered->size() << " points" << std::endl;


    // main loop
    _timer.begin("ransac");
    size_t n_points = _cloud_filtered->size();
    while (_cloud_filtered->size() > 0.25 * n_points && (_params.max_planes == -1 || _planes.size() < _params.max_planes))
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
    if (_params.verbose > 1)
      std::cout<<"found " << _planes.size() << " planes with " << _colored_cloud->size()<< " points" << std::endl;
    if (_params.verbose)
      _timer.report(std::cout, 0, 10);
  }

  PclSAC::color_t PclSAC::_get_color(const pcl::ModelCoefficients &coeffs)
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
