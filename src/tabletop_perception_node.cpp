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

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <dynamic_reconfigure/server.h>
#include <interact_perception/tabletopPerceptionParamConfig.h>



class TableTopPerception
{
public:
    typedef interact_perception::tabletopPerceptionParamConfig DynamicReconfigureType;

    TableTopPerception(void);
    void run(void);
    void pc_callback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void dyn_data_callback(DynamicReconfigureType &config, uint32_t level); 

private:
    typedef pcl::PointXYZ PointType;

    tf2_ros::Buffer m_tfBuffer;
    geometry_msgs::TransformStamped m_transformation;
    tf2_ros::TransformListener m_tfListener;
    ros::NodeHandle m_nh;
    ros::Publisher m_pc_pub;
    ros::Publisher m_pc_plane_pub;

    ros::Subscriber m_pc_sub;

    dynamic_reconfigure::Server<DynamicReconfigureType> m_server;
    dynamic_reconfigure::Server<DynamicReconfigureType>::CallbackType m_f;

    double m_paramPlaneDistThresh;
    bool m_paramSetPlaneExtractNeg;
    int m_paramStatisticFilterMeanK;
    double m_paramStddevMulThresh;
    double m_paramVoxelDownSampleLeafSize;

    double m_paramXMaskMin;
    double m_paramXMaskMax;

    double m_paramYMaskMin;
    double m_paramYMaskMax;

    double m_paramZMaskMin;
    double m_paramZMaskMax;
};

TableTopPerception::TableTopPerception()
	: m_tfListener(m_tfBuffer)
{
    //tf2::TransformListener tfListener(TableTopPerception::tfBuffer);
    m_pc_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/tabletop_perception/cloud_filtered", 10);
    m_pc_plane_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/tabletop_perception/plane", 10);

    m_pc_sub = m_nh.subscribe("/zed/point_cloud/cloud_registered", 10, &TableTopPerception::pc_callback, this);

    m_f = boost::bind(&TableTopPerception::dyn_data_callback,this, _1, _2);
    m_server.setCallback(m_f);

    //set initial parameters, can be changed using dynParam
    m_paramPlaneDistThresh = 0.02;
    m_paramSetPlaneExtractNeg = true;
    m_paramStatisticFilterMeanK = 50;
    m_paramStddevMulThresh = 1;
    m_paramVoxelDownSampleLeafSize = 0.005;

    m_paramXMaskMin = -1.32;
    m_paramXMaskMax = 0.3;

    m_paramYMaskMin = -0.3;
    m_paramYMaskMax = 0.5;

    m_paramZMaskMin = -2;
    m_paramZMaskMax = 10;

}

void TableTopPerception::dyn_data_callback(interact_perception::tabletopPerceptionParamConfig &config, uint32_t level) 
{
  ROS_INFO("Reconfigure Request: PlaneDist, %f PlaneExtract, %s FilterK, %d StdDev, %f  VoxelSize, %f", 
            config.paramPlaneDistThresh,
            config.paramSetPlaneExtractNeg?"True":"False", 
            config.paramStatisticFilterMeanK, 
            config.paramStddevMulThresh,
            config.paramVoxelDownSampleLeafSize);

    m_paramPlaneDistThresh = config.paramPlaneDistThresh;
    m_paramSetPlaneExtractNeg = config.paramSetPlaneExtractNeg;
    m_paramStatisticFilterMeanK = config.paramStatisticFilterMeanK;
    m_paramStddevMulThresh = config.paramStddevMulThresh;
    m_paramVoxelDownSampleLeafSize = config.paramVoxelDownSampleLeafSize;

    m_paramXMaskMin = config.paramXMaskMin;
    m_paramXMaskMax = config.paramXMaskMax;

    m_paramYMaskMin = config.paramYMaskMin;
    m_paramYMaskMax = config.paramYMaskMax;

    m_paramZMaskMin = config.paramZMaskMin;
    m_paramZMaskMax = config.paramZMaskMax;

}

void TableTopPerception::pc_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    ROS_INFO_STREAM("PC has " << msg->data.size() << " points");
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>);

    pcl::PointCloud<PointType>::Ptr cloud_p(new pcl::PointCloud<PointType>);

    pcl::fromROSMsg(*msg, *cloud);

    try{
        m_transformation = m_tfBuffer.lookupTransform("world", "zed_center", ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
    }

    Eigen::Affine3d world_to_map = tf2::transformToEigen(m_transformation);

    ROS_INFO_STREAM("Transformation: " << m_transformation);
    pcl::PointCloud<PointType>::Ptr transformed_cloud (new pcl::PointCloud<PointType> ());
    pcl::transformPointCloud (*cloud, *transformed_cloud, world_to_map);

    pcl::PassThrough<PointType> filter_x, filter_y, filter_z;

    //mask in x
    filter_x.setInputCloud(transformed_cloud);
    filter_x.setFilterFieldName("x");
    filter_x.setFilterLimits(m_paramXMaskMin, m_paramXMaskMax);
    filter_x.filter(*cloud_filtered);

    //Filter in y
    filter_y.setInputCloud(cloud_filtered);
    filter_y.setFilterFieldName("y");
    filter_y.setFilterLimits(m_paramYMaskMin, m_paramYMaskMax);
    filter_y.filter(*cloud_filtered);

    //Filter in z
    filter_z.setInputCloud(cloud_filtered);
    filter_z.setFilterFieldName("z");
    filter_z.setFilterLimits(m_paramZMaskMin, m_paramZMaskMax);
    filter_z.filter(*cloud_filtered);

    // Create the filtering object
    // pcl::StatisticalOutlierRemoval<PointType> sor;
    // sor.setInputCloud (cloud_filtered);
    // sor.setMeanK (m_paramStatisticFilterMeanK);
    // sor.setStddevMulThresh (m_paramStddevMulThresh);
    // sor.filter (*cloud_filtered);

    //downsample the dataset using leaf size of 1cm
    ROS_INFO_STREAM("Before filtering: " << cloud_filtered->width * cloud_filtered->height);
    pcl::VoxelGrid<PointType> filter_voxel;
    filter_voxel.setInputCloud(cloud_filtered);
    //TODO ask to check this
    filter_voxel.setLeafSize ((float) m_paramVoxelDownSampleLeafSize,(float) m_paramVoxelDownSampleLeafSize,(float) m_paramVoxelDownSampleLeafSize);
    filter_voxel.filter(*cloud_filtered);
    ROS_INFO_STREAM("After filtering: " << cloud_filtered->width * cloud_filtered->height);

    //////// Plane Segmentation ////////////
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    //create the segmentation object
    pcl::SACSegmentation<PointType> seg;
    seg.setOptimizeCoefficients (true);

    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (m_paramPlaneDistThresh);

    //create the filtering object
    pcl::ExtractIndices<PointType> extract;

    //segment the largest planer component from the remaining cloud
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        ROS_ERROR_STREAM("Could not estimate a planar model for the given dataset.");
        // break;
    }
    
    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (m_paramSetPlaneExtractNeg);
    extract.filter (*cloud_p);

    ROS_INFO_STREAM("PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." );

    //convert to message
    sensor_msgs::PointCloud2 msg_p;
    pcl::toROSMsg(*cloud_p, msg_p);
    msg_p.header.frame_id = "world";

    //publish pc
    m_pc_plane_pub.publish(msg_p);

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