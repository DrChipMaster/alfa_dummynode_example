#include "alfa_interface.h"
#include <thread>
#include <unistd.h>
#include <chrono>


AlfaInterface::AlfaInterface(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud)
{
    this->pcloud = input_cloud;
    subscrive_topics();
    alive_ticker = new boost::thread(&AlfaInterface::ticker_thread,this);
}

void AlfaInterface::publish_pointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud)
{
    alfa_msg::AlfaMetrics output;
    alfa_msg::MetricMessage new_metric;
    new_metric.metric_name = "Point cloud points";
    new_metric.metric = output_cloud->size();
    new_metric.units = "Points";
    output.metrics.push_back(new_metric);
    output.message_tag = "Dummy point cloud";
    sensor_msgs::PointCloud2 pcl2_frame;
    pcl::toROSMsg(*output_cloud,pcl2_frame);

    cloud_publisher.publish(pcl2_frame);
    publish_metrics(output);

}

void AlfaInterface::publish_metrics(alfa_msg::AlfaMetrics &metrics)
{
    filter_metrics.publish(metrics);

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
    publish_pointcloud(pcloud);
    
}

bool AlfaInterface::parameters_cb(alfa_msg::AlfaConfigure::Request &req, alfa_msg::AlfaConfigure::Response &res)
{
    cout<<"Recieved FilterSettings with size" <<req.configurations.size()<<"... Updating"<<endl;
    for (int i=0; i< req.configurations.size();i++) {
        cout <<"Configuration: "<<i<< " With name: "<< req.configurations[i].config_name<< " with value: "<< req.configurations[i].config<<endl;
    }

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
    filter_metrics = n.advertise<alfa_msg::AlfaMetrics>("dummy_metrics", 1);
    alive_publisher = n.advertise<alfa_msg::AlfaAlivePing>("dummy_alive",1);
    cloud_publisher = n.advertise<sensor_msgs::PointCloud2>("alfa_dummy_cloud",1);
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
