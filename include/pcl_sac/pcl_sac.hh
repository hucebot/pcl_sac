#ifndef PCL_SAC_HH_
#define PCL_SAC_HH_

#include <random>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>

#include "pcl_sac/timer.hh"

namespace pcl_sac
{
    struct Params {
        int verbose = 1; // 0, 1, 2
        int max_planes = -1;
        std::vector<float> leaf_size = { 0.10f, 0.10f, 0.06f }; 
        int num_threads = 8;
        double distance_threshold = 0.03;
        int max_iterations = 15000;
    };

    class PclSAC
    {
    public:
        using color_cloud_t = pcl::PointCloud<pcl::PointXYZRGB>;
        using cloud_t = pcl::PointCloud<pcl::PointXYZ>;
        PclSAC(const Params& params = Params());

        void update(cloud_t::Ptr cloud);
        // list of planes
        const std::vector<color_cloud_t> &planes() const { return _planes; }
        // the full cloud, with a different color for each plane
        const pcl::PointCloud<pcl::PointXYZRGB> &colored_cloud() const { return *_colored_cloud; };
        // the plane coefficients
        const std::vector<pcl::ModelCoefficients> &planes_coefficients() const { return _planes_coefficients; }

    protected:
        Params _params;
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
    };
}
#endif