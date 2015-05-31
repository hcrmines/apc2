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

float NAAN = std::numeric_limits<float>::quiet_NaN();

class Find {
    public:
        Find();
        void get_cloud(const sensor_msgs::PointCloud2&);

        ros::NodeHandle nh;
        ros::Subscriber cloud_sub;
        //ros::Publisher status_pub;
        ros::Publisher centroids_pub;
	Eigen::Affine3f transform;

        pcl::PointCloud<pcl::PointXYZ>::Ptr bg;
};

Find::Find() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr p (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/hcr-ws/ros_ws/src/apc2/data/bg.pcd", *p) != -1) {
        ROS_INFO("> loaded background");
	bg = p;
    } else {
        ROS_INFO("> background failed to load");
    }

    cloud_sub = nh.subscribe("/camera/depth/points", 1, &Find::get_cloud, this);
    //status_pub = nh.advertise<apc2::Status>("/apc2/status", 1);
    centroids_pub = nh.advertise<sensor_msgs::PointCloud2>("/apc2/centroids", 1);

    transform = Eigen::Affine3f::Identity();
    transform.translation() << 0.43424, -0.1013, 0.48206;
    transform.rotate(Eigen::AngleAxisf(-1.57, Eigen::Vector3f::UnitX()));
}

void Find::get_cloud(const sensor_msgs::PointCloud2& c) {
    ROS_INFO("> received cloud");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(c, *cloud);
    ROS_INFO("> converted cloud");
    pcl::transformPointCloud(*cloud, *cloud, transform);
    ROS_INFO("> transformed cloud");

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
    }
    ROS_INFO("> found centroids");

    //apc2::Status s;
    //s.n_centroids = centroid_list.size();
    //s.centroids_x = centroids_x;
    //s.centroids_y = centroids_y;
    //s.centroids_z = centroids_z;
    ///status_pub.publish(s);

    sensor_msgs::PointCloud2::Ptr centmsg (new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*centroid_cloud, *centmsg);
    centmsg->header = c.header;
    centroids_pub.publish(centmsg);

    ROS_INFO("> published centroids\n");
    ros::Duration(5).sleep();
}

int main(int argc, char** argv) {
    ROS_INFO("> init centroid finder");
    ros::init(argc, argv, "find");

    Find f;

    ros::spin();
}
