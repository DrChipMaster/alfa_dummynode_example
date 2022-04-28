#include "alfa_interface.h"
#include <thread>
#include <unistd.h>
#include <chrono>


AlfaInterface::AlfaInterface(pcl::PointCloud<pcl::PointXYZI>::Ptr pcloud)
{
    this->pcloud = pcloud;
    subscrive_topics();
    alive_ticker = new boost::thread(&AlfaInterface::ticker_thread,this);
}

void AlfaInterface::cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud)
{
    //cout<<"Recieved pointcloud"<<endl;
    if ((cloud->width * cloud->height) == 0)
    {
        cout <<"Recieved empty point cloud"<<endl;
        return;
    }
    cout<<"Recieved cloud"<<endl;
    pcl::fromROSMsg(*cloud,*pcloud);
    
}

bool AlfaInterface::parameters_cb(alfa_msg::AlfaConfigure::Request &req, alfa_msg::AlfaConfigure::Response &res)
{
    alfa_msg::AlfaMetrics output;
    cout<<"Recieved FilterSettings with size" <<req.configurations.size()<<"... Updating"<<endl;
    for (int i=0; i< req.configurations.size();i++) {
        cout <<"Configuration: "<<i<< " With name: "<< req.configurations[i].config_name<< " with value: "<< req.configurations[i].config<<endl;
        alfa_msg::MetricMessage new_metric;
        new_metric.metric =  req.configurations[i].config;
        new_metric.metric_name = req.configurations[i].config_name;
        output.metrics.push_back(new_metric);
    }
    filter_metrics.publish(output);
    res.return_status=1;
    return true;

}

void AlfaInterface::init()
{
        char arg0[]= "filter_node";
        char *argv[]={arg0,NULL};
        int argc=(int)(sizeof(argv) / sizeof(char*)) - 1;;
        ros::init (argc, argv, "Dummy node");
          if (!ros::master::check()) {
              cout <<"Failed to inicialize ros"<<endl;
            return;
          }

}

void AlfaInterface::subscrive_topics()
{
    sub_cloud = nh.subscribe("alfa_pointcloud",0,&AlfaInterface::cloud_cb,this);


    sub_parameters = nh.advertiseService("dummy_settings",&AlfaInterface::parameters_cb,this);
    ros::NodeHandle n;
    filter_metrics = n.advertise<alfa_msg::AlfaMetrics>("dummy_metrics", 0);
    alive_publisher = n.advertise<alfa_msg::AlfaAlivePing>("dummy_alive",0);
    m_spin_thread = new boost::thread(&AlfaInterface::spin, this);


}

void AlfaInterface::ticker_thread()
{
    while(ros::ok())
    {
        alfa_msg::AlfaAlivePing newPing;
        newPing.node_name= "Dummy node";
        newPing.node_type = "Test";
        newPing.config_service_name = "dummy_settings";
        newPing.config_tag = "Dummy config tag";
        alfa_msg::ConfigMessage parameter1,parameter2,parameter3,parameter4;

        parameter1.config = 10;
        parameter1.config_name = "Best orientador";

        parameter2.config = 10.30;
        parameter2.config_name = "Best Ricardo";

        parameter3.config = 40;
        parameter3.config_name = "Best Luiz";

        parameter4.config = -90;
        parameter4.config_name = "Best Amaguinhos novos";

        newPing.default_configurations.push_back(parameter1);
        newPing.default_configurations.push_back(parameter2);
        newPing.default_configurations.push_back(parameter3);
        newPing.default_configurations.push_back(parameter4);

        alive_publisher.publish(newPing);
        std::this_thread::sleep_for(std::chrono::milliseconds(TIMER_SLEEP));
    }
}

void AlfaInterface::spin()
{
    int threads = std::thread::hardware_concurrency();
    std::cout << "Started Spinning with processor_count threads"<<threads << std::endl;
    ros::spin();
}
