 #include "ros/ros.h"
#include "alfa_interface.h"

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr OutputCloud;
     inputCloud.reset (new  pcl::PointCloud<pcl::PointXYZI>);
     OutputCloud.reset (new  pcl::PointCloud<pcl::PointXYZI>);
     ros::init (argc, argv, "alfa_node");
       if (!ros::master::check()) {
           cout <<"Failed to inicialize ros"<<endl;
       }

    AlfaInterface mRosThread(inputCloud);
    while(ros::ok())
    {

    }
}
