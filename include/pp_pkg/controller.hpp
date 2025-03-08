#pragma once

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <autoware_msgs/VehicleCmd.h>
#include <cmath>


namespace purepursuit {

    class PurePursuit
    {

        ros::NodeHandle& nh_;
        ros::Subscriber path_sub;
        ros::Subscriber odom_sub;
        ros::Publisher ctrl_pub;
        ros::Publisher path_pub;

        double lookahead_distance;
        double bicycle_length;
        double current_heading;

        nav_msgs::Path path;
        nav_msgs::Odometry vehicle_odom;

        double velocity = 10;

        bool ReadParameters();
        void PathCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void ControlOutput();
        void LocalTransform(geometry_msgs::Pose& current_point_pose, geometry_msgs::Pose& target_point_pose, double transformed_vector[3]);
        geometry_msgs::Pose ChooseWaypoint(geometry_msgs::Pose& current_point_pose, nav_msgs::Path& path, double lookahed_distance);
        double PurePursuitAlgorithm(double target_x, double target_y, double length);

        public:

            PurePursuit(ros::NodeHandle& nh);


    };


}