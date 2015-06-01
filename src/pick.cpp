#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <baxter_core_msgs/EndEffectorCommand.h>

#include <stdlib.h>

#include <apc2/recognize.h>

class Pick {
    public:
        Pick();
        void get_locations(const sensor_msgs::PointCloud2&);
        void get_status(const std_msgs::Bool&);

        void open();
        void close();
        void nap();

        ros::NodeHandle nh;
        ros::Subscriber centroid_sub;
        ros::Subscriber stat_sub;
        ros::Publisher move_pub;
        ros::Publisher drop_pub;
        ros::Publisher grip_pub;

        ros::ServiceClient recog;

        baxter_core_msgs::EndEffectorCommand msg_open;
        baxter_core_msgs::EndEffectorCommand msg_close;
        geometry_msgs::Point drop_point;

        bool is_picking;
        bool is_success;
};

Pick::Pick() {
    centroid_sub = nh.subscribe("/apc2/centroids", 1, &Pick::get_locations, this);
    stat_sub = nh.subscribe("/apc2/movestat", 1, &Pick::get_status, this);
    move_pub = nh.advertise<geometry_msgs::Point>("/apc2/move", 1);
    drop_pub = nh.advertise<geometry_msgs::Point>("/apc2/drop", 1);
    grip_pub = nh.advertise<baxter_core_msgs::EndEffectorCommand>(
                  "/robot/end_effector/right_gripper/command", 1);

    recog = nh.serviceClient<apc2::recognize>("recognize");

    baxter_core_msgs::EndEffectorCommand calib;
    calib.id = 65538;
    calib.command = "calibrate";
    ROS_INFO("> calibrating grippers");
    grip_pub.publish(calib);
    ros::Duration(3).sleep();

    // 65664 for left
    msg_open.id = 65538;
    msg_open.command = "release";

    msg_close.id = 65538;
    msg_close.command = "grip";

    drop_point.x = 0.46;
    drop_point.y = -0.8;
    drop_point.z = -0.27;

    is_picking = false;
    is_success = true;

    open();
}

void Pick::get_status(const std_msgs::Bool& b) {
    is_success = b.data;
}

void Pick::get_locations(const sensor_msgs::PointCloud2& c) {
    if (is_picking) { return; }
    is_picking = true;
    open();
    ROS_INFO("> received locations");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(c, *cloud);
    ROS_INFO("> converted cloud");

    if (cloud->points.size() == 0) {
        ROS_INFO("> no centroids found");
        is_picking = false;
        return;
    }

    int idx = rand() % cloud->points.size();

    geometry_msgs::Point pt;

    pt.x = cloud->points[idx].x;
    pt.y = cloud->points[idx].y + 0.3;
    pt.z = cloud->points[idx].z;
    move_pub.publish(pt);
    nap();
    if (!is_success) {
        ROS_INFO("> moving to approach failed");
        is_picking = false;
        return;
    }
 
    pt.y = cloud->points[idx].y;
    apc2::recognize srv;
    srv.request.p = pt;
    if (!recog.call(srv)) {
        ROS_INFO("> service call failed");
        is_picking = false;
        return;
    }

    if (!srv.response.want_to_pick.data) {
        ROS_INFO("> object not wanted");
        is_picking = false;
        return;
    }
    nap();

    move_pub.publish(pt);
    nap();
    if (!is_success) {
        ROS_INFO("> moving to object failed");
        is_picking = false;
        return;
    }

    close();
    nap();

    pt.y = cloud->points[idx].y + 0.3;
    move_pub.publish(pt);
    nap();
    if (!is_success) {
        ROS_INFO("> moving object out failed");
        is_picking = false;
        return;
    }

    move_pub.publish(drop_point);
    nap();
    if (!is_success) {
        ROS_INFO("> moving to drop failed");
        is_picking = false;
        return;
    }

    open();
    nap();

    is_picking = false;
}

void Pick::open() {
    ROS_INFO("> opening grippers");
    grip_pub.publish(msg_open);
}

void Pick::close() {
    ROS_INFO("> closing grippers");
    grip_pub.publish(msg_close);
}

void Pick::nap() {
    ros::Duration(10).sleep();
}

int main(int argc, char** argv) {
    ROS_INFO("> init point picker");
    ros::init(argc, argv, "pick");

    Pick p;

    ros::spin();
}
