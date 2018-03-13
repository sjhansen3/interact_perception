#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>

class TableTopPerception {
    ros::Publisher pc_pub;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

public:
    TableTopPerception(int x, int y);
    void pc_callback(const sensor_msgs::PointCloud2ConstPtr&msg);
    void run(void);
}