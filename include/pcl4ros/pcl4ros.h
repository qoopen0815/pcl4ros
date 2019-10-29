#ifndef PCL4ROS_H
#define PCL4ROS_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/octree/octree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

namespace pcl4ros
{
class PCL4ROS
{
public:
  PCL4ROS(ros::NodeHandle& nh);
  void publish(void);
  void updateBaseCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  void updateDiffCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

private:
  void downsampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                       const double leaf_size);                              // ダウンサンプリング
  void segmentSurface(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);  // 平面除去
  void removeOutlier(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);   // 外れ値除去
  void clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);      // 一番大きいクラスタを抽出
  void extractDifference(pcl::PointCloud<pcl::PointXYZ>::Ptr base_cloud,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr diff_cloud);

private:
  ros::NodeHandle nh_;
  ros::Publisher extracted_cloud_pub_;
  ros::Subscriber base_cloud_sub_;
  ros::Subscriber diff_cloud_sub_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr base_cloud_;       // 被差分点群
  pcl::PointCloud<pcl::PointXYZ>::Ptr diff_cloud_;       // 差分点群
  pcl::PointCloud<pcl::PointXYZ>::Ptr extracted_cloud_;  // 出力点群

  bool base_flag_;
  bool diff_flag_;
  double octree_resolution_;
  double leaf_size_;
  double cluster_tolerance_;
  double cluster_min_size_;
  double cluster_max_size_;
};
}  // namespace pcl4ros

#endif  // PCL4ROS_H
