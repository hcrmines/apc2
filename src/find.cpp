#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>

#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/extract_indices.h>

#include <limits>

//#include <apc2/Status.h>

#define X_TRANSLATION 0.055             // - left, + right
#define Y_TRANSLATION -0.16             // - up, + down
#define Z_TRANSLATION 0.14              // - towards, + away
#define X_ROTATION 0.0
#define Y_ROTATION 0.0
#define Z_ROTATION 0.0

float NAAN = std::numeric_limits<float>::quiet_NaN();

class Find {
    public:
        Find();
        void get_cloud(const sensor_msgs::PointCloud2&);

        ros::NodeHandle nh;
        ros::Subscriber cloud_sub;
        ros::Publisher cloud_pub;
        //ros::Publisher status_pub;
        ros::Publisher centroids_pub;
        Eigen::Affine3f transform;

        pcl::PointCloud<pcl::PointXYZ>::Ptr bg;
};

Find::Find() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr p (new pcl::PointCloud<pcl::PointXYZ>);
    int ret = pcl::io::loadPCDFile<pcl::PointXYZ>(
                   "/home/hcr-ws/ros_ws/src/apc2/data/bg.pcd", *p);
    if (ret != -1) {
        ROS_INFO("> loaded background");
    bg = p;
    } else {
        ROS_INFO("> background failed to load");
    }

    cloud_sub = nh.subscribe("/camera/depth/points", 1, &Find::get_cloud, this);
    //status_pub = nh.advertise<apc2::Status>("/apc2/status", 1);
    centroids_pub = nh.advertise<sensor_msgs::PointCloud2>("/apc2/centroids", 1);
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/apc2/foreground", 1);

    transform = Eigen::Affine3f::Identity();
    transform.translation() << X_TRANSLATION, Y_TRANSLATION, Z_TRANSLATION;
    transform.rotate(Eigen::AngleAxisf(X_ROTATION, Eigen::Vector3f::UnitX()));
    transform.rotate(Eigen::AngleAxisf(Y_ROTATION, Eigen::Vector3f::UnitY()));
    transform.rotate(Eigen::AngleAxisf(Z_ROTATION, Eigen::Vector3f::UnitZ()));
    ROS_INFO("> using translation of (%.02f, %.02f, %.02f)",
             X_TRANSLATION, Y_TRANSLATION, Z_TRANSLATION);
    ROS_INFO("> using rotation of (%.02f, %.02f, %.02f)\n",
             X_ROTATION, Y_ROTATION, Z_ROTATION);
}

void Find::get_cloud(const sensor_msgs::PointCloud2& c) {
    ROS_INFO("> beginning analysis...");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(c, *cloud);
    ROS_INFO("> converted cloud");

    pcl::PointCloud<pcl::PointXYZ>::Ptr fg (new pcl::PointCloud<pcl::PointXYZ>);
    fg->width = cloud->width;
    fg->height = cloud->height;
    fg->resize(fg->width * fg->height);
    fg->is_dense = false;
    ROS_INFO("> initialized foreground");

    for (int i = 0; i < cloud->width * cloud->height; i++) {
        pcl::PointXYZ p = cloud->points[i];
        pcl::PointXYZ b = bg->points[i];
    if (sqrt(pow(p.x-b.x, 2) + pow(p.y-b.y, 2) + pow(p.z-b.z, 2)) < 0.05) {
        fg->points[i].x = NAAN;
        fg->points[i].y = NAAN;
        fg->points[i].z = NAAN;
        } else {
        fg->points[i].x = p.x;
        fg->points[i].y = p.y;
        fg->points[i].z = p.z;
        }
    }
    ROS_INFO("> subtracted cloud");

    pcl::transformPointCloud(*fg, *fg, transform);
    ROS_INFO("> transformed cloud");

    sensor_msgs::PointCloud2::Ptr fg_msg (new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*fg, *fg_msg);
    fg_msg->header = c.header;
    cloud_pub.publish(fg_msg);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*fg, *fg, indices);
    ROS_INFO("> removed NaNs");

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(fg);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_ex;
    cluster_ex.setClusterTolerance(0.02);
    cluster_ex.setMinClusterSize(1000);
    cluster_ex.setMaxClusterSize(10000);
    cluster_ex.setSearchMethod(tree);
    cluster_ex.setInputCloud(fg);
    cluster_ex.extract(cluster_indices);
    ROS_INFO("> clustered points");

    pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<float> centroids_x;
    std::vector<float> centroids_y;
    std::vector<float> centroids_z;
    for (int i = 0; i < cluster_indices.size(); i++) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (int j = 0; j < cluster_indices[i].indices.size(); j++) {
            cluster->points.push_back(
                     fg->points[cluster_indices[i].indices[j]]);
        }
        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        Eigen::Vector4f cent;
        pcl::compute3DCentroid(*cluster, cent);

        float dist = sqrt(pow(cent[0], 2) + pow(cent[1], 2) + pow(cent[2], 2));
        if (dist > 1.5 || dist < 0.2) { continue; }

        centroid_cloud->points.push_back(pcl::PointXYZ(cent[0], cent[1], cent[2]));
        centroids_x.push_back(cent[0]);
        centroids_y.push_back(cent[1]);
        centroids_z.push_back(cent[2]);
        ROS_INFO("> found centroid (%.02f, %.02f, %.02f), %d points", 
                 cent[0], cent[1], cent[2], (int)cluster->width);
    }
    if (centroid_cloud->points.size() == 0) {
        ROS_INFO("> found no centroids\n");
    } else {
        sensor_msgs::PointCloud2::Ptr centmsg (new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*centroid_cloud, *centmsg);
        centmsg->header = c.header;
        centroids_pub.publish(centmsg);
        ROS_INFO("> published centroids\n");
    }

    ros::Duration(5).sleep();
}

int main(int argc, char** argv) {
    ROS_INFO("> init centroid finder");
    ros::init(argc, argv, "find");

    Find f;

    ros::spin();
}
