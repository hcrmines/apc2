#include <ros/ros.h>

#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

class Save {
    public:
        Save();
        void save(const sensor_msgs::PointCloud2&);

        ros::NodeHandle nh;
        ros::Subscriber sub;

        bool saved;
};

Save::Save() {
    saved = false;
    sub = nh.subscribe("/camera/depth/points", 1,
                                     &Save::save, this);
}

void Save::save(const sensor_msgs::PointCloud2& c) {
    if (saved) { return; }
    ROS_INFO("> received cloud");

    pcl::PointCloud<pcl::PointXYZ>::Ptr p (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(c, *p);
    ROS_INFO("> converted cloud");
 
    pcl::io::savePCDFileASCII("/home/hcr-ws/ros_ws/src/apc2/data/bg.pcd", *p);
    saved = true;
    ROS_INFO("> saved cloud, ctrl-C to exit");
}

int main(int argc, char** argv) {
    ROS_INFO("> starting save node");
    ros::init(argc, argv, "save");

    Save s;
    ros::spin();
}
