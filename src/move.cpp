#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include <moveit_msgs/DisplayTrajectory.h>

class Move {
    public:
        Move();
        void move(const geometry_msgs::Point&);
        void drop(const geometry_msgs::Point&);
        bool execute(const geometry_msgs::Pose&);

        moveit::planning_interface::MoveGroup mg;
        moveit::planning_interface::PlanningSceneInterface plan;

        ros::NodeHandle nh;
        ros::Subscriber move_sub;
        ros::Subscriber drop_sub;
        ros::Publisher stat_pub;
        ros::Publisher disp_pub;

        geometry_msgs::Quaternion approach_pick;
        geometry_msgs::Quaternion approach_drop;

        moveit_msgs::DisplayTrajectory trajectory;
};

Move::Move() : mg("right_arm") {
    approach_pick.x = 0.707;
    approach_pick.y = 0;
    approach_pick.z = 0.707;
    approach_pick.w = 0;

    approach_drop.x = 0.742014471757;
    approach_drop.y = -0.670177574301;
    approach_drop.z = -0.00191265099284;
    approach_drop.w = 0.0165192122305;

    mg.setPlanningTime(20.0);

    move_sub = nh.subscribe("/apc2/move", 1, &Move::move, this);
    drop_sub = nh.subscribe("/apc2/drop", 1, &Move::drop, this);
    stat_pub = nh.advertise<std_msgs::Bool>("/apc2/movestat", 1);
    disp_pub = nh.advertise<moveit_msgs::DisplayTrajectory>(
                  "/move_group/display_planned_path", 1);
}

void Move::drop(const geometry_msgs::Point& pt) {
    ROS_INFO("> received drop point (%.02f, %.02f, %.02f)", pt.x, pt.y, pt.z);
    
    ROS_INFO("> moving to drop");
    geometry_msgs::Pose pose;
    pose.orientation = approach_drop;
    pose.position = pt;
    if (!execute(pose)) {
        ROS_INFO("> movement failed, aborting");
        std_msgs::Bool b;
        b.data = false;
        stat_pub.publish(b);
    } else {
        ROS_INFO("> movement successful");
        std_msgs::Bool b;
        b.data = true;
        stat_pub.publish(b);
    }
}

void Move::move(const geometry_msgs::Point& pt) {
    ROS_INFO("> received move point (%.02f, %.02f, %.02f)", pt.x, pt.y, pt.z);

    ROS_INFO("> moving to point");
    geometry_msgs::Pose pose;
    pose.orientation = approach_pick;
    pose.position = pt;
    if (!execute(pose)) {
        ROS_INFO("> movement failed, aborting");
        std_msgs::Bool b;
        b.data = false;
        stat_pub.publish(b);
    } else {
        ROS_INFO("> movement successful");
        std_msgs::Bool b;
        b.data = true;
        stat_pub.publish(b);
    }
}

bool Move::execute(const geometry_msgs::Pose& pose) {
    ROS_INFO("> planning");

    mg.setStartState(*mg.getCurrentState());
    mg.setPoseTarget(pose);
    //mg.setRandomTarget();
    mg.setGoalTolerance(0);
    ROS_INFO("> set pose");

    moveit::planning_interface::MoveGroup::Plan plan;
    if (mg.plan(plan)) {
        ROS_INFO("> planning successful");
    } else {
        ROS_INFO("> planning failed");
        return false;
    }
    ros::Duration(3).sleep();
    /*
    if (mg.execute(plan)) {
        ROS_INFO("> execution successful");
    } else {
        ROS_INFO("> execution failed");
        return false;
    }
    return true;
    */
    trajectory.trajectory_start = plan.start_state_;
    trajectory.trajectory.push_back(plan.trajectory_);
    disp_pub.publish(trajectory);

    if (mg.asyncMove()) {
        ROS_INFO("> execution successful");
        return true;
    } else {
        ROS_INFO("> execution failed");
        return false;
    }
}

int main(int argc, char** argv) {
    ROS_INFO("> init movement");
    ros::init(argc, argv, "move");

    Move m;

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}   
