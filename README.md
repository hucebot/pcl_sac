# pcl_sac
ROS module for plane extraction using PCL SAC

## usage :
- you can use the rgbd_docker to get the real sense: `sh ./run_realsense.sh localhost`
- `rosrun pcl_sac pcl_sac input:=/camera/depth/color/points`
- you can vizualize the colored point cloud with rviz