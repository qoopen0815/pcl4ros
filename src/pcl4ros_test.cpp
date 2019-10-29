#include <pcl4ros/pcl4ros.h>

using pcl4ros::PCL4ROS;

// Node
int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl4ros_test");
  ros::NodeHandle nh;

  PCL4ROS extractor(nh);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    extractor.publish();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}