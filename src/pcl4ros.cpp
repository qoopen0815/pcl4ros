#include <pcl4ros/pcl4ros.h>

using pcl4ros::PCL4ROS;

// Class methods definitions
PCL4ROS::PCL4ROS(ros::NodeHandle& nh)
  : nh_(nh)
  , base_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
  , diff_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
  , extracted_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
{
  extracted_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("extracted_cloud", 10);
  base_cloud_sub_ = nh.subscribe("base_cloud", 1, &PCL4ROS::updateBaseCloud, this);
  diff_cloud_sub_ = nh.subscribe("diff_cloud", 1, &PCL4ROS::updateDiffCloud, this);

  base_flag_ = false;
  diff_flag_ = false;
  octree_resolution_ = 0.02;
  leaf_size_ = 0.004;
  cluster_tolerance_ = 0.02;
  cluster_min_size_ = 50;
  cluster_max_size_ = 250000;
}

void PCL4ROS::publish(void)
{
  if (base_flag_ == true && diff_flag_ == true)
  {
    extractDifference(base_cloud_, diff_cloud_);
    removeOutlier(extracted_cloud_);
    clustering(extracted_cloud_);
    extracted_cloud_->header.frame_id = base_cloud_->header.frame_id;

    // publish
    sensor_msgs::PointCloud2 published_cloud;
    pcl::toROSMsg(*extracted_cloud_, published_cloud);
    // std::cout << __LINE__ << std::endl;
    extracted_cloud_pub_.publish(published_cloud);
    // std::cout << __LINE__ << std::endl;
  }
}

void PCL4ROS::updateBaseCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  pcl::fromROSMsg(*cloud_msg, *base_cloud_);
  downsampleCloud(base_cloud_, leaf_size_);
  base_flag_ = true;
}

void PCL4ROS::updateDiffCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  pcl::fromROSMsg(*cloud_msg, *diff_cloud_);
  downsampleCloud(diff_cloud_, leaf_size_);
  diff_flag_ = true;
}

void PCL4ROS::downsampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, const double leaf_size)
{
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(input_cloud);
  sor.setLeafSize(leaf_size, leaf_size, leaf_size);
  sor.filter(*input_cloud);
}

void PCL4ROS::segmentSurface(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);  // 検出するモデルのタイプを指定
  seg.setMethodType(pcl::SAC_RANSAC);     // 検出に使用する方法を指定
  seg.setDistanceThreshold(0.01);         // RANSACの最小二乗法の許容誤差範囲
  seg.setMaxIterations(2000);
  seg.setProbability(0.95);

  seg.setInputCloud(input_cloud);
  seg.segment(*inliers, *coefficients);

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(input_cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);  // trueの場合出力は検出された平面以外のデータ falseの場合は平面のデータ
  extract.filter(*input_cloud);
}

void PCL4ROS::removeOutlier(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(input_cloud);  // 外れ値を除去する点群を入力
  sor.setMeanK(50);                // MeanKを設定 点群数
  sor.setStddevMulThresh(0.1);
  sor.setNegative(false);    // 外れ値を出力する場合はtrueにする
  sor.filter(*input_cloud);  // 出力
}

void PCL4ROS::clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(cluster_tolerance_);
  ec.setMinClusterSize(cluster_min_size_);
  ec.setMaxClusterSize(cluster_max_size_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  uint32_t max_num = 0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr max_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // std::cout << "begin indices : " << cluster_indices.begin() << std::endl;
  // std::cout << "end indices   : " << cluster_indices.end() << std::endl;

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      cloud_cluster->points.push_back(cloud->points[*pit]);

    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    if (cloud_cluster->width > max_num)
    {
      max_num = cloud_cluster->width;
      max_cloud = cloud_cluster;
    }
  }

  copyPointCloud(*max_cloud, *cloud);

  // Empty Buffer
  cluster_indices.clear();
}

void PCL4ROS::extractDifference(pcl::PointCloud<pcl::PointXYZ>::Ptr base_cloud,
                                        pcl::PointCloud<pcl::PointXYZ>::Ptr diff_cloud)
{
  pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(octree_resolution_);  // Octreeを作成
  octree.setInputCloud(diff_cloud);  // 元となる点群を入力
  octree.addPointsFromInputCloud();
  octree.switchBuffers();            // バッファの切り替え
  octree.setInputCloud(base_cloud);  // 比較対象の点群を入力
  octree.addPointsFromInputCloud();

  std::vector<int> newPointIdxVector;
  octree.getPointIndicesFromNewVoxels(newPointIdxVector);  // 比較の結果差分と判断された点郡の情報を保管

  // 保管先のサイズの設定
  extracted_cloud_->width = diff_cloud->points.size() + base_cloud->points.size();
  extracted_cloud_->height = 1;
  extracted_cloud_->points.resize(extracted_cloud_->width * extracted_cloud_->height);

  int n = 0;  // 差分点群の数を保存する
  for (size_t i = 0; i < newPointIdxVector.size(); i++)
  {
    extracted_cloud_->points[i].x = base_cloud->points[newPointIdxVector[i]].x;
    extracted_cloud_->points[i].y = base_cloud->points[newPointIdxVector[i]].y;
    extracted_cloud_->points[i].z = base_cloud->points[newPointIdxVector[i]].z;
    n++;
  }
  // 差分点群のサイズの再設定
  extracted_cloud_->width = n;
  extracted_cloud_->height = 1;
  extracted_cloud_->points.resize(extracted_cloud_->width * extracted_cloud_->height);

  std::cout << "original cloud size  : " << base_cloud->width << std::endl;
  std::cout << "extracted cloud size : " << extracted_cloud_->width << std::endl;
}
