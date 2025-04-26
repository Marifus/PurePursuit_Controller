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

        path_sub = nh_.subscribe("/shortest_path", 10, &PurePursuit::PathCallback, this);
        odom_sub = nh_.subscribe("/odom", 10, &PurePursuit::OdomCallback, this);
        ctrl_pub = nh_.advertise<autoware_msgs::VehicleCmd>("/vehicle_cmd", 10);
        path_pub = nh_.advertise<nav_msgs::Path>("/path", 10);        

    }


    bool PurePursuit::ReadParameters()
    {
        if (!nh_.getParam("lookahead_distance", lookahead_distance)) return false;
        if (!nh_.getParam("bicycle_length", bicycle_length)) return false;

        ROS_INFO("Lookahead Distance: [%f]", lookahead_distance);
        ROS_INFO("Sase Uzunlugu: [%f]", bicycle_length);

        return true;
    }


    void PurePursuit::PathCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        geometry_msgs::PoseStamped pose_stamped;

        pose_stamped.pose = msg->pose.pose;
        pose_stamped.header = msg->header;

        path.poses.push_back(pose_stamped);
        path.header = msg->header;
        path_pub.publish(path);
    }


    void PurePursuit::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        vehicle_odom = *msg;

        geometry_msgs::Quaternion& q = vehicle_odom.pose.pose.orientation;
        current_heading = atan2(2.0 * (q.w * q.z + q.x * q.y) , 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

        ROS_INFO("Simdiki Konum: [%f, %f]", vehicle_odom.pose.pose.position.x, vehicle_odom.pose.pose.position.y);
        ROS_INFO("Simdiki Yaw: [%f]", current_heading);

        if (path.poses.size() != 0) ControlOutput();
    }


    void PurePursuit::ControlOutput()
    {
        geometry_msgs::Pose target_pose = ChooseLookaheadPoint(vehicle_odom.pose.pose, path, lookahead_distance, ClosestWaypointIndex(vehicle_odom.pose.pose, path));
        double transformed_vec[3] = {0, 0, 0};

        LocalTransform(vehicle_odom.pose.pose, target_pose, transformed_vec);

        double steering_angle = PurePursuitAlgorithm(transformed_vec[0], transformed_vec[1], bicycle_length);

        double steering_angle_degree = steering_angle * (180 / M_PI);

        if (steering_angle_degree > 100) {

            steering_angle = 100 * (M_PI / 180);

        }

        if (steering_angle_degree < -100) {

            steering_angle = -100 * (M_PI / 180);

        }

        ROS_INFO("Donus Acisi: [%f]", steering_angle);

        autoware_msgs::VehicleCmd control_msg;
        control_msg.twist_cmd.twist.angular.z = steering_angle;
        control_msg.twist_cmd.twist.linear.x = velocity;
        ctrl_pub.publish(control_msg);
    }


    void PurePursuit::LocalTransform(geometry_msgs::Pose& current_point_pose, geometry_msgs::Pose& target_point_pose, double transformed_vector[3])
    {
        
        double tx = -1 * current_point_pose.position.x;
        double ty = -1 * current_point_pose.position.y;

        geometry_msgs::Quaternion& q = current_point_pose.orientation;
        double current_heading_ = atan2(2.0 * (q.w * q.z + q.x * q.y) , 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
/*         double current_heading_ = tf::getYaw(current_point_pose.orientation); */

        double target_vec[3] = {target_point_pose.position.x, target_point_pose.position.y, 1};

        double TransformationMatrix[3][3] = {
            {cos(-current_heading_), -sin(-current_heading_), cos(-current_heading_) * tx - sin(-current_heading_) * ty},
            {sin(-current_heading_), cos(-current_heading_), sin(-current_heading_) * tx + cos(-current_heading_) * ty},
            {0.0, 0.0, 1.0}
        };

        for (int i=0; i<3; i++) {
        
            transformed_vector[i] = 0;

            for (int j=0; j<3; j++) {

                transformed_vector[i] += TransformationMatrix[i][j] * target_vec[j];

           }
        }

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

    
    geometry_msgs::Pose PurePursuit::ChooseLookaheadPoint(geometry_msgs::Pose& current_point_pose, nav_msgs::Path& path, double t_lookahead_distance, int closest_point_index) 
    {

        int chosen_point_index = closest_point_index;
        geometry_msgs::PoseStamped& waypoint = path.poses[chosen_point_index];

        double distance = sqrt(std::pow((waypoint.pose.position.x - current_point_pose.position.x), 2) + std::pow((waypoint.pose.position.y - current_point_pose.position.y), 2));

        while (t_lookahead_distance > distance) 
        {  
            chosen_point_index++;
            if (chosen_point_index<path.poses.size()) 
            {
            geometry_msgs::PoseStamped& waypoint = path.poses[chosen_point_index];
            distance = sqrt(std::pow((waypoint.pose.position.x - current_point_pose.position.x), 2) + std::pow((waypoint.pose.position.y - current_point_pose.position.y), 2));
            }
            else break;
        }

        chosen_point_index--;

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

        geometry_msgs::Pose& chosen_point = path.poses[chosen_point_index].pose;
        return chosen_point;
    }


    double PurePursuit::PurePursuitAlgorithm(double target_x, double target_y, double length)
    {
        double distance2 = (target_x * target_x) + (target_y * target_y);

        double steering = atan2((2*target_y*length), (distance2));

        return steering;
    }

}