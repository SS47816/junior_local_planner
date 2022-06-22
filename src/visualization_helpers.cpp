#include "junior_local_planner/visualization_helpers.h"

void VisualizationHelpers::createCurrentPoseMarker(const VehicleState& current_state, geometry_msgs::PoseStamped& pose)
{
    pose.header.frame_id = "map";
    pose.pose.position.x = current_state.x;
    pose.pose.position.y = current_state.y;
    geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(current_state.yaw);
    pose.pose.orientation = pose_quat;
}

void VisualizationHelpers::createCarFootprintMarker(const std::vector<Waypoint>& car_footprint, visualization_msgs::Marker& car_footprint_marker)
{
    car_footprint_marker.header.frame_id = "map";
    car_footprint_marker.header.stamp = ros::Time();
    car_footprint_marker.ns = "car_footprint_rviz";
    car_footprint_marker.type = visualization_msgs::Marker::LINE_STRIP;
    car_footprint_marker.action = visualization_msgs::Marker::ADD;
    car_footprint_marker.scale.x = 0.05;
    car_footprint_marker.scale.y = 0.05;
    //car_footprint_marker.scale.z = 0.1;
    car_footprint_marker.frame_locked = false;
    car_footprint_marker.color.r = 0.0;
    car_footprint_marker.color.g = 0.0;
    car_footprint_marker.color.b = 1.0;
    car_footprint_marker.color.a = 0.6;

    for(int i = 0; i < car_footprint.size(); i++)
    {
        geometry_msgs::Point p;
        p.x = car_footprint[i].x;
        p.y = car_footprint[i].y;
        // p.z = car_footprint[i].z;

        car_footprint_marker.points.push_back(p);
    }

    if(car_footprint.size() > 0)
    {
        geometry_msgs::Point p;
        p.x = car_footprint[0].x;
        p.y = car_footprint[0].y;
        // p.z = car_footprint[0].z;
        car_footprint_marker.points.push_back(p);
    }
}

void VisualizationHelpers::createSafetyBoxMarker(const std::vector<Waypoint>& safety_box, visualization_msgs::Marker& safety_box_marker)
{
    safety_box_marker.header.frame_id = "map";
    safety_box_marker.header.stamp = ros::Time();
    safety_box_marker.ns = "safety_box_rviz";
    safety_box_marker.type = visualization_msgs::Marker::LINE_STRIP;
    safety_box_marker.action = visualization_msgs::Marker::ADD;
    safety_box_marker.scale.x = 0.05;
    safety_box_marker.scale.y = 0.05;
    //safety_box_marker.scale.z = 0.1;
    safety_box_marker.frame_locked = false;
    safety_box_marker.color.r = 0.0;
    safety_box_marker.color.g = 1.0;
    safety_box_marker.color.b = 1.0;
    safety_box_marker.color.a = 0.6;

    for(int i = 0; i < safety_box.size(); i++)
    {
        geometry_msgs::Point p;
        p.x = safety_box[i].x;
        p.y = safety_box[i].y;
        // p.z = safety_box[i].z;

        safety_box_marker.points.push_back(p);
    }

    if(safety_box.size() > 0)
    {
        geometry_msgs::Point p;
        p.x = safety_box[0].x;
        p.y = safety_box[0].y;
        // p.z = safety_box[0].z;
        safety_box_marker.points.push_back(p);
    }
}

void VisualizationHelpers::createBoxObstacleMarker(const BoxObstacle& b, visualization_msgs::Marker& box_obstacle_marker)
{
    box_obstacle_marker.header.frame_id = "map";
    box_obstacle_marker.header.stamp = ros::Time();
    box_obstacle_marker.ns = "box_obstacle_rviz";
    box_obstacle_marker.type = visualization_msgs::Marker::LINE_STRIP;
    box_obstacle_marker.action = visualization_msgs::Marker::ADD;
    box_obstacle_marker.scale.x = 0.05;
    box_obstacle_marker.scale.y = 0.05;
    //box_obstacle_marker.scale.z = 0.1;
    box_obstacle_marker.frame_locked = false;
    box_obstacle_marker.color.r = 1.0;
    box_obstacle_marker.color.g = 0.0;
    box_obstacle_marker.color.b = 0.0;
    box_obstacle_marker.color.a = 1.0;

    Mat3 rotationMat(b.heading-M_PI_2);
    Mat3 translationMat(b.x, b.y);

    Waypoint bottom_left;
    bottom_left.x = -b.width/2.0;
    bottom_left.y = -b.length/2.0;

    Waypoint bottom_right; 
    bottom_right.x = b.width/2.0;
    bottom_right.y = -b.length/2.0;

    Waypoint top_right;
    top_right.x = b.width/2.0;
    top_right.y = b.length/2.0;

    Waypoint top_left;
    top_left.x = -b.width/2.0;
    top_left.y = b.length/2.0;

    bottom_left = rotationMat*bottom_left;
    bottom_left = translationMat*bottom_left;

    bottom_right = rotationMat*bottom_right;
    bottom_right = translationMat*bottom_right;

    top_right = rotationMat*top_right;
    top_right = translationMat*top_right;

    top_left = rotationMat*top_left;
    top_left = translationMat*top_left;

    geometry_msgs::Point p;
    p.x = bottom_left.x;
    p.y = bottom_left.y;
    box_obstacle_marker.points.push_back(p);

    p.x = bottom_right.x;
    p.y = bottom_right.y;
    box_obstacle_marker.points.push_back(p);

    p.x = top_right.x;
    p.y = top_right.y;
    box_obstacle_marker.points.push_back(p);

    p.x = top_left.x;
    p.y = top_left.y;
    box_obstacle_marker.points.push_back(p);

    p.x = bottom_left.x;
    p.y = bottom_left.y;
    box_obstacle_marker.points.push_back(p);
}

void VisualizationHelpers::createGlobalPathMarker(const std::vector<Waypoint>& global_path, nav_msgs::Path& path)
{
    path.header.frame_id = "map";

    for(unsigned int i = 0; i < global_path.size(); i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = global_path[i].x;
        pose.pose.position.y = global_path[i].y;
        geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(global_path[i].heading);
        pose.pose.orientation = pose_quat;
        path.poses.push_back(pose);
    }
}

void VisualizationHelpers::createExtractedPathMarker(const std::vector<Waypoint>& extracted_path, nav_msgs::Path& path)
{
    path.header.frame_id = "map";

    for(unsigned int i = 0; i < extracted_path.size(); i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = extracted_path[i].x;
        pose.pose.position.y = extracted_path[i].y;
        geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(extracted_path[i].heading);
        pose.pose.orientation = pose_quat;
        path.poses.emplace_back(pose);
    }
}

void VisualizationHelpers::createRollOutsMarker(const std::vector<std::vector<Waypoint>>& roll_outs, visualization_msgs::MarkerArray& roll_out_marker_array)
{
    visualization_msgs::Marker roll_out_marker;
    roll_out_marker.header.frame_id = "map";
    roll_out_marker.header.stamp = ros::Time();
    roll_out_marker.ns = "local_roll_out_marker";
    roll_out_marker.type = visualization_msgs::Marker::LINE_STRIP;
    roll_out_marker.action = visualization_msgs::Marker::ADD;
    roll_out_marker.scale.x = 0.01;
    roll_out_marker.scale.y = 0.01;
    //roll_out_marker.scale.z = 0.1;
    roll_out_marker.frame_locked = false;

    int count = 0;
    for(int i = 0; i < roll_outs.size(); i++)
    {
        roll_out_marker.points.clear();
        roll_out_marker.id = count;
        
        for(int j = 0; j < roll_outs[i].size(); j++)
        {            
            geometry_msgs::Point point;
            point.x = roll_outs[i][j].x;
            point.y = roll_outs[i][j].y;
            point.z = 0;
            roll_out_marker.points.push_back(point);
        }

        roll_out_marker.color.a = 0.9;
        roll_out_marker.color.r = 0.0;
        roll_out_marker.color.g = 0.0;
        roll_out_marker.color.b = 1.0;

        roll_out_marker_array.markers.push_back(roll_out_marker);
        count++;
    }
}

void VisualizationHelpers::createWeightedRollOutsMarker(const std::vector<std::vector<Waypoint>>& roll_outs, const std::vector<PathCost>& path_costs, const int& best_index, visualization_msgs::MarkerArray& weighted_roll_out_marker_array)
{
    visualization_msgs::Marker weighted_roll_out_marker;
    weighted_roll_out_marker.header.frame_id = "map";
    weighted_roll_out_marker.header.stamp = ros::Time();
    weighted_roll_out_marker.ns = "weighted_trajectories_colored";
    weighted_roll_out_marker.type = visualization_msgs::Marker::LINE_STRIP;
    weighted_roll_out_marker.action = visualization_msgs::Marker::ADD;
    weighted_roll_out_marker.scale.x = 0.05;
    weighted_roll_out_marker.scale.y = 0.05;
    // weighted_roll_out_marker.scale.z = 0.1;
    weighted_roll_out_marker.color.a = 0.9;
    weighted_roll_out_marker.color.r = 1.0;
    weighted_roll_out_marker.color.g = 1.0;
    weighted_roll_out_marker.color.b = 1.0;
    weighted_roll_out_marker.frame_locked = false;

    int count = 0;
    for(int i = 0; i < roll_outs.size(); i++)
    {
        weighted_roll_out_marker.points.clear();
        weighted_roll_out_marker.id = count;

        for(int j = 0; j < roll_outs[i].size(); j++)
        {
            geometry_msgs::Point point;

            point.x = roll_outs[i][j].x;
            point.y = roll_outs[i][j].y;

            weighted_roll_out_marker.points.push_back(point);
        }

        weighted_roll_out_marker.color.b = 0;

        if(path_costs.size() == roll_outs.size())
        {
            float norm_cost = path_costs[i].cost * roll_outs.size();
            if(norm_cost <= 1.0)
            {
                weighted_roll_out_marker.color.r = norm_cost;
                weighted_roll_out_marker.color.g = 1.0;
            }
            else if(norm_cost > 1.0)
            {
                weighted_roll_out_marker.color.r = 1.0;
                weighted_roll_out_marker.color.g = 2.0 - norm_cost;
            }
        }
        else
        {
            weighted_roll_out_marker.color.r = 1.0;
            weighted_roll_out_marker.color.g = 0.0;
        }

        if(path_costs[i].bBlocked)
        {
            std::cout << "Path " << i << " blocked!" << std::endl;
            weighted_roll_out_marker.color.r = 1.0;
            weighted_roll_out_marker.color.g = 0.0;
            weighted_roll_out_marker.color.b = 0.0;
        }

        if(i == best_index)
        {
            weighted_roll_out_marker.color.r = 1.0;
            weighted_roll_out_marker.color.g = 0.0;
            weighted_roll_out_marker.color.b = 1.0;
        }

        weighted_roll_out_marker_array.markers.push_back(weighted_roll_out_marker);
        count++;
    }
}