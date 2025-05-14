#include "pp_pkg/controller.hpp"

namespace purepursuit 
{

    PurePursuit::PurePursuit(ros::NodeHandle& nh) : nh_(nh)
    {

        if (!ReadParameters())
        {
            ROS_ERROR("Parametreler Okunamadi.");
            ros::requestShutdown();
        }

        marker_id = 0;

        path_sub = nh_.subscribe("/shortest_path", 10, &PurePursuit::PathCallback, this);
        odom_sub = nh_.subscribe("/odom", 10, &PurePursuit::OdomCallback, this);
        ctrl_pub = nh_.advertise<autoware_msgs::VehicleCmd>("/vehicle_cmd", 10);
        path_pub = nh_.advertise<nav_msgs::Path>("/path", 10);        
        mark_pub = nh_.advertise<visualization_msgs::Marker>("/waypoint", 10);

    }


    bool PurePursuit::ReadParameters()
    {
        if (!nh_.getParam("lookahead_distance", lookahead_distance)) return false;
        if (!nh_.getParam("axle_length", axle_length)) return false;
        if (!nh_.getParam("velocity", velocity)) return false;
        if (!nh_.getParam("input_log", input_log)) return false;
        if (!nh_.getParam("output_log", output_log)) return false;

        ROS_INFO("Lookahead Distance: [%f]", lookahead_distance);
        ROS_INFO("Sase Uzunlugu: [%f]", axle_length);
        ROS_INFO("Arac Hizi: [%f]", velocity);
        ROS_INFO("Girdi/Cikti Gosterme: [%d, %d]", input_log, output_log);

        return true;
    }


/*     void PurePursuit::PathCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        geometry_msgs::PoseStamped pose_stamped;

        pose_stamped.pose = msg->pose.pose;
        pose_stamped.header = msg->header;

        path.poses.push_back(pose_stamped);
        path.header = msg->header;
        path_pub.publish(path);
    } */


    void PurePursuit::PathCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        path = *msg;

        for (int i = 0; i < path.poses.size(); ++i)
        {
            if (i+1<path.poses.size())
            {
                while (path.poses[i].pose.position.x == path.poses[i+1].pose.position.x && path.poses[i].pose.position.y == path.poses[i+1].pose.position.y)
                {
                    path.poses.erase(path.poses.begin()+i);
                    if (i+1>=path.poses.size()) break;
                }
            }
        }
    }


    void PurePursuit::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        vehicle_odom = *msg;

        vehicle_odom.pose.pose.position.x += cos(current_heading)*axle_length*0;
        vehicle_odom.pose.pose.position.y += sin(current_heading)*axle_length*0;

        geometry_msgs::Quaternion& q = vehicle_odom.pose.pose.orientation;
        current_heading = atan2(2.0 * (q.w * q.z + q.x * q.y) , 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

        if(input_log)
        {
            ROS_INFO("Simdiki Konum: [%f, %f]", vehicle_odom.pose.pose.position.x, vehicle_odom.pose.pose.position.y);
            ROS_INFO("Simdiki Yaw: [%f]", current_heading);
        }

        if (path.poses.size() != 0) ControlOutput();
    }


    void PurePursuit::ControlOutput()
    {
        geometry_msgs::Pose target_pose = ChooseLookaheadPoint(vehicle_odom.pose.pose, path, lookahead_distance, ClosestWaypointIndex(vehicle_odom.pose.pose, path));

        geometry_msgs::Pose transformed_target_pose = LocalTransform(vehicle_odom.pose.pose, target_pose);

        double steering_angle = PurePursuitAlgorithm(transformed_target_pose.position.x, transformed_target_pose.position.y, axle_length);

        double steering_angle_degree = steering_angle * (180 / M_PI);

        if (steering_angle_degree > 100) {

            steering_angle = 100 * (M_PI / 180);

        }

        if (steering_angle_degree < -100) {

            steering_angle = -100 * (M_PI / 180);

        }

        if(output_log)
        {
            ROS_INFO("Donus Acisi: [%f]", steering_angle);
        }

        autoware_msgs::VehicleCmd control_msg;
        control_msg.twist_cmd.twist.angular.z = steering_angle;
        control_msg.twist_cmd.twist.linear.x = velocity;
        ctrl_pub.publish(control_msg);
    }


    int PurePursuit::ClosestWaypointIndex(geometry_msgs::Pose& current_point_pose, nav_msgs::Path& path)
    {
        int closest_point_index;
        double min_distance2;

        for (int i = 0; i < path.poses.size(); ++i)
        {
            geometry_msgs::PoseStamped& waypoint = path.poses[i];

            if (i == 0)
            {
                closest_point_index = i;
                min_distance2 = std::pow((waypoint.pose.position.x - current_point_pose.position.x), 2) + std::pow((waypoint.pose.position.y - current_point_pose.position.y), 2);
            }

            else 
            {
                double distance2 = std::pow((waypoint.pose.position.x - current_point_pose.position.x), 2) + std::pow((waypoint.pose.position.y - current_point_pose.position.y), 2);

                if (distance2 < min_distance2)
                {
                    closest_point_index = i;
                    min_distance2 = distance2;
                }
            }
        }

        return closest_point_index;
    }

    
    geometry_msgs::Pose PurePursuit::ChooseLookaheadPoint(geometry_msgs::Pose& current_point_pose, nav_msgs::Path& t_path, double t_lookahead_distance, int closest_point_index) 
    {

        int chosen_point_index = closest_point_index;
        geometry_msgs::PoseStamped& waypoint = t_path.poses[chosen_point_index];

        double distance = sqrt(std::pow((waypoint.pose.position.x - current_point_pose.position.x), 2) + std::pow((waypoint.pose.position.y - current_point_pose.position.y), 2));

        while (t_lookahead_distance > distance) 
        {  

            if (++chosen_point_index<t_path.poses.size()) 
            {
            geometry_msgs::PoseStamped& waypoint = t_path.poses[chosen_point_index];
            distance = sqrt(std::pow((waypoint.pose.position.x - current_point_pose.position.x), 2) + std::pow((waypoint.pose.position.y - current_point_pose.position.y), 2));
            }

            else 
            {
                chosen_point_index = t_path.poses.size()-1;
                break;
            }

        }

/* 
        for (int i = closest_point_index; i < path.poses.size(); ++i)
        {

            geometry_msgs::PoseStamped& waypoint = path.poses[i];
            double distance = sqrt(std::pow((waypoint.pose.position.x - current_point.pose.position.x), 2) + std::pow((waypoint.pose.position.y - current_point.pose.position.y), 2));
            double best_distance;
            
            if (i = closest_point_index)
            {
                chosen_point_index = closest_point_index;
                best_distance = t_lookahead_distance - distance;
            }

            if ((lookahead_distance - distance) < best_distance)
            {
                chosen_point_index = closest_point_index;
                best_distance = t_lookahead_distance - distance;
            }

        }
*/

        geometry_msgs::Pose& chosen_point = t_path.poses[chosen_point_index].pose;

        visualization_msgs::Marker marker;
        marker.header = t_path.header;
        marker.id = marker_id;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = chosen_point.position;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.scale.x = 0.15;
        marker.scale.y = 0.15;
        marker.scale.z = 0.15;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        
        mark_pub.publish(marker);

        return chosen_point;
    }


    double PurePursuit::PurePursuitAlgorithm(double target_x, double target_y, double length)
    {
        double distance2 = (target_x * target_x) + (target_y * target_y);

        double steering = atan2((2*target_y*length), (distance2));

        return steering;
    }


    geometry_msgs::Pose PurePursuit::LocalTransform(const geometry_msgs::Pose& origin_pose, const geometry_msgs::Pose& target_point_pose)
    {   
        tf2::Transform origin_tf, target_point_tf, g2l_transform, transformed_point_tf;

        tf2::fromMsg(origin_pose, origin_tf);
        g2l_transform = origin_tf.inverse();

        tf2::fromMsg(target_point_pose, target_point_tf);

        transformed_point_tf = g2l_transform * target_point_tf;
        
        geometry_msgs::Pose transformed_pose;
        tf2::toMsg(transformed_point_tf, transformed_pose);

        return transformed_pose;
    }

}