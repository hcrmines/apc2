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
	Eigen::Affine3f transform;

        bool saved;
};

Save::Save() {
    saved = false;
    sub = nh.subscribe("/camera/depth/points", 1,
                                     &Save::save, this);
    transform = Eigen::Affine3f::Identity();
    transform.translation() << 0.43424, -0.1013, 0.48206;
    transform.rotate(Eigen::AngleAxisf(-1.57, Eigen::Vector3f::UnitX()));
}

void Save::save(const sensor_msgs::PointCloud2& c) {
    if (saved) { return; }
    ROS_INFO("> received cloud");
    ROS_INFO("> transformed cloud");

    pcl::PointCloud<pcl::PointXYZ>::Ptr p (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(c, *p);
    ROS_INFO("> converted cloud");
    pcl::transformPointCloud(*p, *p, transform);
    ROS_INFO("> transformed cloud");
 
    pcl::io::savePCDFileASCII("/home/hcr-ws/ros_ws/src/apc2/data/bg.pcd", *p);
    saved = true;
    ROS_INFO("> saved cloud");
}

int main(int argc, char** argv) {
    ROS_INFO("> starting save node");
    ros::init(argc, argv, "save");

    Save s;
    ros::spin();
}
