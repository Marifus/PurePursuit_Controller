#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <autoware_msgs/VehicleCmd.h>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <visualization_msgs/Marker.h>


namespace purepursuit {

    class PurePursuit
    {
        bool input_log, output_log;

        ros::NodeHandle& nh_;
        ros::Subscriber path_sub;
        ros::Subscriber odom_sub;
        ros::Publisher ctrl_pub;
        ros::Publisher path_pub;
        ros::Publisher mark_pub;

        double lookahead_distance;
        double axle_length;
        double current_heading;
        int marker_id;

        nav_msgs::Path path;
        nav_msgs::Odometry vehicle_odom;

        double velocity;

        bool ReadParameters();
/*         void PathCallback(const nav_msgs::Odometry::ConstPtr& msg); */
        void PathCallback(const nav_msgs::Path::ConstPtr& msg);
        void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void ControlOutput();
        int ClosestWaypointIndex(geometry_msgs::Pose& current_point_pose, nav_msgs::Path& path);
        geometry_msgs::Pose ChooseLookaheadPoint(geometry_msgs::Pose& current_point_pose, nav_msgs::Path& path, double t_lookahead_distance, int closest_point_index);
        double PurePursuitAlgorithm(double target_x, double target_y, double length);
        geometry_msgs::Pose LocalTransform(const geometry_msgs::Pose& origin_pose, const geometry_msgs::Pose& target_point_pose);

        public:

            PurePursuit(ros::NodeHandle& nh);


    };


}