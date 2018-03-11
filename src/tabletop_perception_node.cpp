#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
//#include <tf2/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>


// #include <iostream>
// #include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>



class TableTopPerception
{
public:
    TableTopPerception(void);
    void run(void);

    void pc_callback(const sensor_msgs::PointCloud2ConstPtr &msg);

private:
    typedef pcl::PointXYZ PointType;

    tf2_ros::Buffer m_tfBuffer;
    geometry_msgs::TransformStamped m_transformation;
    tf2_ros::TransformListener m_tfListener;
    ros::NodeHandle m_nh;
    ros::Publisher m_pc_pub;
    ros::Subscriber m_pc_sub;

};

TableTopPerception::TableTopPerception()
	: m_tfListener(m_tfBuffer)
{
    //tf2::TransformListener tfListener(TableTopPerception::tfBuffer);
    m_pc_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/tabletop_perception/cloud_filtered", 10);
    m_pc_sub = m_nh.subscribe("/zed/point_cloud/cloud_registered", 10, &TableTopPerception::pc_callback, this);
}

void TableTopPerception::pc_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    ROS_INFO_STREAM("PC has " << msg->data.size() << " points");
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>);

    pcl::PointCloud<PointType>::Ptr cloud_p(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud_f(new pcl::PointCloud<PointType>);

    pcl::fromROSMsg(*msg, *cloud);
    // Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    try{
        m_transformation = m_tfBuffer.lookupTransform("world", "zed_center", ros::Time(0));
        //tfBuffer.lookupTransform()
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
    }

    Eigen::Affine3d world_to_map = tf2::transformToEigen(m_transformation);

    ROS_INFO_STREAM("Transformation: " << m_transformation);
    // ROS_INFO_STREAM("World to map: " << world_to_map);

    pcl::PointCloud<PointType>::Ptr transformed_cloud (new pcl::PointCloud<PointType> ());
    pcl::transformPointCloud (*cloud, *transformed_cloud, world_to_map);

    pcl::PassThrough<PointType> filter_x, filter_y;

    // Filter in x
    filter_x.setInputCloud(transformed_cloud);
    filter_x.setFilterFieldName("x");
    filter_x.setFilterLimits(-1.32, 0.3);
    filter_x.filter(*cloud_filtered);

    //Filter in y
    filter_y.setInputCloud(cloud_filtered);
    filter_y.setFilterFieldName("y");
    filter_y.setFilterLimits(-0.3, 0.5);
    filter_y.filter(*cloud_filtered);

    //downsample the dataset using leaf size of 1cm
    ROS_INFO_STREAM("Before filtering: " << cloud_filtered->width * cloud_filtered->height);
    pcl::VoxelGrid<PointType> filter_voxel;
    filter_voxel.setInputCloud(cloud_filtered);
    filter_voxel.setLeafSize (0.005f, 0.005f, 0.005f);
    filter_voxel.filter(*cloud_filtered);
    ROS_INFO_STREAM("After filtering: " << cloud_filtered->width * cloud_filtered->height);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    //create the segmentation object
    pcl::SACSegmentation<PointType> seg;
    seg.setOptimizeCoefficients (true);

    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.01);

    //create the filtering object
    pcl::ExtractIndices<PointType> extract;
    int i = 0, nr_points = (int) cloud_filtered->points.size ();

    //while 30% of the original cloud is still there
    while (cloud_filtered->points.size () > 0.3 * nr_points)
    {
        //segment the largest planer component from the remaining cloud
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            ROS_ERROR_STREAM("Could not estimate a planar model for the given dataset.");
            break;
        }
        
        // Extract the inliers
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_p);

        ROS_INFO_STREAM("PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." );

        std::stringstream ss;
        ss << "interact_scene_plane_" << i;

        //convert to message
        sensor_msgs::PointCloud2 msg_p;
        pcl::toROSMsg(*cloud_p, msg_p);
        msg_p.header.frame_id = "world";

        //publish pc
        m_pc_plane_pub.publish(msg_p);

        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud_filtered.swap (cloud_f);
        i++;
    }

    //convert to message
    sensor_msgs::PointCloud2 msg_filtered;
    pcl::toROSMsg(*cloud_filtered, msg_filtered);
    msg_filtered.header.frame_id = "world";

    //publish pc
    m_pc_pub.publish(msg_filtered);
}

void TableTopPerception::run(void)
{
    // ros::spin();
    ros::Rate rate(10.0);
    while (ros::ok())
    {
        ROS_INFO_STREAM("Time is " << ros::Time::now());
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tabletop_perception_node");

    ROS_INFO("Hello, world!");

    TableTopPerception table_perception;
    table_perception.run();

    // ros::Rate rate(10.0);
    // while (ros::ok())
    // {

        // rate.sleep();
        // ros::spinOnce();
    // }

    return 0;
}