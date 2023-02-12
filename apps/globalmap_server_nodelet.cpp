#include <mutex>
#include <memory>
#include <iostream>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/filters/voxel_grid.h>

namespace hdl_localization {

class GlobalmapServerNodelet : public nodelet::Nodelet {
public:
  using PointT = pcl::PointXYZI;

  GlobalmapServerNodelet() {
  }
  virtual ~GlobalmapServerNodelet() {
  }

  void onInit() override {
    nh = getNodeHandle();
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();

    initialize_params();

    // publish globalmap with "latched" publisher
    globalmap_pub = nh.advertise<sensor_msgs::PointCloud2>("/globalmap", 5, true);
    map_update_sub = nh.subscribe("/map_request/pcd", 10,              &GlobalmapServerNodelet::map_update_callback, this);

    globalmap_pub_timer = nh.createWallTimer(ros::WallDuration(1.0), &GlobalmapServerNodelet::pub_once_cb, this, true, true);
  }

private:
  void initialize_params() {
    // read globalmap from a pcd file
    std::string globalmap_file = private_nh.param<std::string>("globalmap_file", "");

    std::vector<std::string> strs;
    boost.split(strs, globalmap_file,boost::is_any_of("."));
    std::string extension = strs[strs.size()-1];

    globalmap.reset(new pcl::PointCloud<PointT>());

    if (extension == "pcd"){
      pcl::io::loadPCDFile(globalmap_file, *globalmap);
    }
    elif (extension == "ply"){
      pcl::io::loadPLYFile(globalmap_file, *globalmap);
    }
    else{
      ROS_ERROR("The extension of globalmap should be either .pcd or .ply ...");
    }

    globalmap->header.frame_id = "map";

    std::ifstream utm_file(globalmap_file + ".utm");
    if (utm_file.is_open() && private_nh.param<bool>("convert_utm_to_local", true)) {
      double utm_easting;
      double utm_northing;
      double altitude;
      utm_file >> utm_easting >> utm_northing >> altitude;
      for(auto& pt : globalmap->points) {
        pt.getVector3fMap() -= Eigen::Vector3f(utm_easting, utm_northing, altitude);
      }
      ROS_INFO_STREAM("Global map offset by UTM reference coordinates (x = "
                      << utm_easting << ", y = " << utm_northing << ") and altitude (z = " << altitude << ")");
    }

    // downsample globalmap
    double downsample_resolution = private_nh.param<double>("downsample_resolution", 0.1);
    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    voxelgrid->setInputCloud(globalmap);

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    voxelgrid->filter(*filtered);

    globalmap = filtered;
  }

  void pub_once_cb(const ros::WallTimerEvent& event) {
    globalmap_pub.publish(globalmap);
  }

  void map_update_callback(const std_msgs::String &msg){
    ROS_INFO_STREAM("Received map request, map path : "<< msg.data);
    std::string globalmap_file = msg.data;
    globalmap.reset(new pcl::PointCloud<PointT>());

    std::vector<std::string> strs;
    boost.split(strs, globalmap_file,boost::is_any_of("."));
    std::string extension = strs[strs.size()-1];

    if (extension == "pcd"){
      pcl::io::loadPCDFile(globalmap_file, *globalmap);
    }
    elif (extension == "ply"){
      pcl::io::loadPLYFile(globalmap_file, *globalmap);
    }
    else{
      ROS_ERROR("The extension of globalmap should be either .pcd or .ply ...");
    }

    globalmap->header.frame_id = "map";

    // downsample globalmap
    double downsample_resolution = private_nh.param<double>("downsample_resolution", 0.1);
    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    voxelgrid->setInputCloud(globalmap);

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    voxelgrid->filter(*filtered);

    globalmap = filtered;
    globalmap_pub.publish(globalmap);
  }

private:
  // ROS
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;

  ros::Publisher globalmap_pub;
  ros::Subscriber map_update_sub;

  ros::WallTimer globalmap_pub_timer;
  pcl::PointCloud<PointT>::Ptr globalmap;
};

}


PLUGINLIB_EXPORT_CLASS(hdl_localization::GlobalmapServerNodelet, nodelet::Nodelet)
