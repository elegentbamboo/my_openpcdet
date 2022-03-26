#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

using std::cout;
using std::endl;
using std::stringstream;
using std::string;
using namespace pcl;

void lidarsaveCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{ 
    int count = 0;
    while (ros::ok())
  { 
    ros::Rate rate(10);
    std::string filefolder = "/Project/pycharm_project/OpenPCDet-master/mycode/pointcloud_pub_sub_ws/lidar";
    pcl::PointCloud<pcl::PointXYZI> cloud; // With color
  
    pcl::fromROSMsg(*input, cloud); // sensor_msgs::PointCloud2 ----> pcl::PointCloud<T>
        stringstream stream;
//        stream << "inputCloud"<< filesNum<< ".pcd";
        string filename = filefolder + "/" + std::to_string(count++) + ".pcd";
        pcl::io::savePCDFile(filename, cloud);
        cout << filename<<" Saved."<<endl;
   }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_listener");
  ros::NodeHandle nh;
  ros::Subscriber pcl_sub;

  pcl_sub = nh.subscribe("/rslidar_points_80", 1, lidarsaveCallback);
  ros::spin();

}

