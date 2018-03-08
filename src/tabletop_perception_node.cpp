#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

// class TabletopPerceptionNode
// {
// public:
//     TabletopPerceptionNode()
//     {
    
//     }

//     ros::NodeHandle nh_;
//     ros::Publisher

// };

ros::Publisher pc_pub;


void pc_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    ROS_INFO_STREAM("PC has " << msg->data.size() << " points");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*msg, *cloud);

    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    
    float theta = M_PI/4; // The angle of rotation in radians
    //translate 2.5 meters in the x axis
    transform_2.translation() << 2.5, 0.0, 0.0;

    transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));    
    //print the transformation
    printf("\nMethod using Affine3f\n");
    std::cout << transform_2.matrix() << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*cloud, *transformed_cloud, transform_2);
    // pcl::PassThrough<pcl::PointXYZ> filter;
    // filter.setInputCloud(cloud);
    // filter.setFilterFieldName("x");
    // filter.setFilterLimits(-0.5, 0.5);
    // filter.filter(*cloud_filtered);

    sensor_msgs::PointCloud2 msg_filtered;
    pcl::toROSMsg(*transformed_cloud, msg_filtered);

    pc_pub.publish(msg_filtered);
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tabletop_perception_node");

    ROS_INFO("Hello, world!");

    ros::NodeHandle nh;
    pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/tabletop_perception/cloud_filtered", 10);
    ros::Subscriber pc_sub = nh.subscribe("/zed/point_cloud/cloud_registered", 10, pc_callback);

    ros::spin();

    // ros::Rate rate(10.0);
    // while (ros::ok())
    // {

        // rate.sleep();
        // ros::spinOnce();
    // }

    return 0;
}