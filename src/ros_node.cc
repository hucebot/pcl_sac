#include "pcl_sac/ros_node.hh"

namespace pcl_sac
{
    RosNode::RosNode(const Params &params, bool publish_cloud) : _pcl_sac(params),
                                                                 _publish_cloud(publish_cloud)
    {
        // you can/should remap input to to the realsense/xtion/etc.
        _sub_cloud = _nh.subscribe("input", 1, &RosNode::cloud_cb, this);
        _pub_coeffs = _nh.advertise<pcl_msgs::ModelCoefficients>("/pcl_sac/plane_coeffs", 1);

        if (params.verbose)
        {
            std::cout << "Subscribing to: " << _sub_cloud.getTopic() << std::endl;
            std::cout << "Publishing: " << _pub_coeffs.getTopic() << std::endl;
        }

        if (_publish_cloud) // for debugging / visualization
            _pub_colored_planes = _nh.advertise<sensor_msgs::PointCloud2>("/pcl_sac/colored_planes", 1);

        if (params.verbose && _publish_cloud)
            std::cerr << "Publishing: " << _pub_colored_planes.getTopic() << std::endl;
    }

    void RosNode::cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)
    {
        pcl::fromROSMsg(*input, *_cloud);
        _pcl_sac.update(_cloud);
        if (_publish_cloud)
        {
            pcl::toROSMsg(_pcl_sac.colored_cloud(), _output);
            _output.header.stamp = ros::Time::now();
            _output.header.frame_id = input->header.frame_id;

            _pub_colored_planes.publish(_output);
        }
    }
}

// rosrun pcl_sac pcl_sac input:=/camera/depth/color/points
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_ransac");
    pcl_sac::Params  params = { .verbose = true, .max_planes = 5 };
    pcl_sac::RosNode ros_node(params, true); 
    ros::spin();
}