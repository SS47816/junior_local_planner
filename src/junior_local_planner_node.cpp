#include "junior_local_planner/junior_local_planner_node.h"

// Constructor
KarcherLocalPlannerNode::KarcherLocalPlannerNode() : tf_listener(tf_buffer)
{
    ros::NodeHandle private_nh("~");

    // Topics
    std::string odom_topic_;
    std::string obstacles_topic_;
    std::string global_path_topic_;

    std::string global_path_rviz_topic_;
    std::string extracted_path_rviz_topic_;
    std::string current_pose_rviz_topic_;
    std::string roll_outs_rviz_topic_;
    std::string weighted_trajectories_rviz_topic_;
    std::string safety_box_rviz_topic_;
    std::string car_footprint_rviz_topic_;
    std::string box_obstacle_rviz_topic_;
    std::string cmd_vel_topic_;

    // Parameters from launch file: topic names
    ROS_ASSERT(private_nh.getParam("odom_topic", odom_topic_));
    ROS_ASSERT(private_nh.getParam("obstacles_topic", obstacles_topic_));
    ROS_ASSERT(private_nh.getParam("global_path_topic", global_path_topic_));

    ROS_ASSERT(private_nh.getParam("global_path_rviz_topic", global_path_rviz_topic_));
    ROS_ASSERT(private_nh.getParam("extracted_path_rviz_topic", extracted_path_rviz_topic_));
    ROS_ASSERT(private_nh.getParam("current_pose_rviz_topic", current_pose_rviz_topic_));
    ROS_ASSERT(private_nh.getParam("roll_outs_rviz_topic", roll_outs_rviz_topic_));
    ROS_ASSERT(private_nh.getParam("weighted_trajectories_rviz_topic", weighted_trajectories_rviz_topic_));
    ROS_ASSERT(private_nh.getParam("safety_box_rviz_topic", safety_box_rviz_topic_));
    ROS_ASSERT(private_nh.getParam("car_footprint_rviz_topic", car_footprint_rviz_topic_));
    ROS_ASSERT(private_nh.getParam("box_obstacle_rviz_topic", box_obstacle_rviz_topic_));
    ROS_ASSERT(private_nh.getParam("cmd_vel_topic", cmd_vel_topic_));

    // Hyperparameters
    ROS_ASSERT(private_nh.getParam("planning_frequency", planning_frequency_));

    // Parameters from launch file: Planner Parameters
    ROS_ASSERT(private_nh.getParam("max_speed", MAX_SPEED_));    
    ROS_ASSERT(private_nh.getParam("max_local_plan_distance", MAX_LOCAL_PLAN_DISTANCE_));   
    ROS_ASSERT(private_nh.getParam("path_density", PATH_DENSITY_));        
    ROS_ASSERT(private_nh.getParam("roll_outs_number", ROLL_OUTS_NUMBER_));           
    ROS_ASSERT(private_nh.getParam("sampling_tip_margin", SAMPLING_TIP_MARGIN_));     
    ROS_ASSERT(private_nh.getParam("sampling_out_margin", SAMPLING_OUT_MARGIN_));     
    ROS_ASSERT(private_nh.getParam("roll_out_density", ROLL_OUT_DENSITY_)); 
    ROS_ASSERT(private_nh.getParam("roll_in_speed_factor", ROLL_IN_SPEED_FACTOR_)); 
    ROS_ASSERT(private_nh.getParam("roll_in_margin", ROLL_IN_MARGIN_)); 
    ROS_ASSERT(private_nh.getParam("lane_change_speed_factor", LANE_CHANGE_SPEED_FACTOR_)); 
    ROS_ASSERT(private_nh.getParam("horizon_distance", HORIZON_DISTANCE_)); 

    ROS_ASSERT(private_nh.getParam("horizontal_safety_distance", HORIZONTAL_SAFETY_DISTANCE_)); 
    ROS_ASSERT(private_nh.getParam("vertical_safety_distance", VERTICAL_SAFETY_DISTANCE_)); 
    ROS_ASSERT(private_nh.getParam("max_steer_angle", MAX_STEER_ANGLE_)); 
    ROS_ASSERT(private_nh.getParam("min_speed", MIN_SPEED_)); 
    ROS_ASSERT(private_nh.getParam("lateral_skip_distance", LATERAL_SKIP_DISTANCE_)); 
    
    ROS_ASSERT(private_nh.getParam("min_following_distance", MIN_FOLLOWING_DISTANCE_));   
    ROS_ASSERT(private_nh.getParam("max_following_distance", MAX_FOLLOWING_DISTANCE_));       
    ROS_ASSERT(private_nh.getParam("min_distance_to_avoid", MIN_DISTANCE_TO_AVOID));         
    
    ROS_ASSERT(private_nh.getParam("vehicle_width", VEHICLE_WIDTH_));  
    ROS_ASSERT(private_nh.getParam("vehicle_length", VEHICLE_LENGTH_));       
    ROS_ASSERT(private_nh.getParam("wheelbase_length", WHEELBASE_LENGTH_));       
    ROS_ASSERT(private_nh.getParam("turning_radius", TURNING_RADIUS_));
    ROS_ASSERT(private_nh.getParam("safety_radius", SAFETY_RADIUS_));   

    // Smoothing weights
    ROS_ASSERT(private_nh.getParam("smooth_data_weight", SMOOTH_DATA_WEIGHT_));
    ROS_ASSERT(private_nh.getParam("smooth_weight", SMOOTH_WEIGHT_));
    ROS_ASSERT(private_nh.getParam("smooth_tolerance", SMOOTH_TOLERANCE_));

    ROS_ASSERT(private_nh.getParam("priority_weight", PRIORITY_WEIGHT_));
    ROS_ASSERT(private_nh.getParam("transition_weight", TRANSITION_WEIGHT_));
    ROS_ASSERT(private_nh.getParam("lat_weight", LAT_WEIGHT_));
    ROS_ASSERT(private_nh.getParam("long_weight", LONG_WEIGHT_));
    ROS_ASSERT(private_nh.getParam("collision_weight", COLLISION_WEIGHT_));
    ROS_ASSERT(private_nh.getParam("curvature_weight", CURVATURE_WEIGHT_));

    // Subscribe & Advertise
    odom_sub = nh.subscribe(odom_topic_, 1, &KarcherLocalPlannerNode::odomCallback, this);
    obstacles_sub = nh.subscribe(obstacles_topic_, 1, &KarcherLocalPlannerNode::obstaclesCallback, this);
    global_path_sub = nh.subscribe(global_path_topic_, 1, &KarcherLocalPlannerNode::globalPathCallback, this);

    global_path_rviz_pub = nh.advertise<nav_msgs::Path>(global_path_rviz_topic_, 1, true);
    extracted_path_rviz_pub = nh.advertise<nav_msgs::Path>(extracted_path_rviz_topic_, 1, true);
    current_pose_rviz_pub = nh.advertise<geometry_msgs::PoseStamped>(current_pose_rviz_topic_, 1, true);
    roll_outs_rviz_pub = nh.advertise<visualization_msgs::MarkerArray>(roll_outs_rviz_topic_, 1, true);
    weighted_trajectories_rviz_pub = nh.advertise<visualization_msgs::MarkerArray>(weighted_trajectories_rviz_topic_, 1, true);
    safety_box_rviz_pub = nh.advertise<visualization_msgs::Marker>(safety_box_rviz_topic_, 1, true);
    car_footprint_rviz_pub = nh.advertise<visualization_msgs::Marker>(car_footprint_rviz_topic_, 1, true);
    box_obstacle_rviz_pub = nh.advertise<visualization_msgs::Marker>(box_obstacle_rviz_topic_, 1, true);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);

    // timer
    timer = nh.createTimer(ros::Duration(1.0 / planning_frequency_), &KarcherLocalPlannerNode::mainTimerCallback, this);

    b_global_path = false;
    b_vehicle_state = false;
    b_obstacles = false;
    prev_closest_index = 0;
    prev_cost = 0;
};

void KarcherLocalPlannerNode::mainTimerCallback(const ros::TimerEvent& timer_event)
{
    if(!b_global_path)
    {
        ROS_WARN("Karcher Local Planner Node: Global path not received!");
        return;
    }
    if(!b_vehicle_state)
    {
        ROS_WARN("Karcher Local Planner Node: Odom data not received!");
        return;
    }

    publishCmdVel();

    if(!b_obstacles)
    {
        ROS_WARN("Karcher Local Planner Node: Obstacles data not received!");
        return;
    }

    std::vector<Waypoint> extracted_path;
    extractGlobalPathSection(extracted_path);

    std::vector<std::vector<Waypoint>> roll_outs;
    generateRollOuts(extracted_path, roll_outs);

    std::vector<PathCost> trajectory_costs;
    doOneStepStatic(roll_outs, extracted_path, trajectory_costs);
}

void KarcherLocalPlannerNode::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    b_vehicle_state = true;

    current_state.speed = odom_msg->twist.twist.linear.x;
    if(fabs(odom_msg->twist.twist.linear.x) > 0.25)
        current_state.steer = atan(WHEELBASE_LENGTH_ * odom_msg->twist.twist.angular.z/odom_msg->twist.twist.linear.x);

    geometry_msgs::TransformStamped transform_stamped;
    try
    {
        transform_stamped = tf_buffer.lookupTransform("map", "odom", ros::Time(0));
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }

    geometry_msgs::PoseStamped pose_before_transform, pose_after_transform;
    pose_before_transform.header.frame_id = odom_msg->header.frame_id;
    pose_before_transform.header.stamp = odom_msg->header.stamp;
    pose_before_transform.pose = odom_msg->pose.pose;
    tf2::doTransform(pose_before_transform, pose_after_transform, transform_stamped);

    tf::Quaternion q(pose_after_transform.pose.orientation.x, pose_after_transform.pose.orientation.y,
                    pose_after_transform.pose.orientation.z, pose_after_transform.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_state.yaw);

    // Current XY of robot (map frame)
    current_state.x = pose_after_transform.pose.position.x;
    current_state.y = pose_after_transform.pose.position.y;

    geometry_msgs::PoseStamped pose;
    VisualizationHelpers::createCurrentPoseMarker(current_state, pose);
    current_pose_rviz_pub.publish(pose);
}

void KarcherLocalPlannerNode::obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr& obstacle_msg)
{
    b_obstacles = true;

    circle_obstacles.clear();
    obstacle_waypoints.clear();
    box_obstacles.clear();

    // std::cout << "Number of obstacles detected: " << obstacle_msg->circles.size() << std::endl;

    for(int i = 0; i < obstacle_msg->circles.size(); i++)
    {
        CircleObstacle co;
        co.x = obstacle_msg->circles[i].center.x;
        co.y = obstacle_msg->circles[i].center.y;
        co.vx = obstacle_msg->circles[i].velocity.x;
        co.vy = obstacle_msg->circles[i].velocity.y;
        co.radius = obstacle_msg->circles[i].radius;
        co.true_radius = obstacle_msg->circles[i].true_radius;
        circle_obstacles.push_back(co);

        Waypoint wp;
        wp.x = obstacle_msg->circles[i].center.x;
        wp.y = obstacle_msg->circles[i].center.y;
        wp.left_width = obstacle_msg->circles[i].radius;            // TODO: for now, left width attribute represents radius of circle obstacle
        wp.right_width = obstacle_msg->circles[i].true_radius;      // TODO: for now, right width attribute represents true radius of circle obstacle
        obstacle_waypoints.push_back(wp);

        BoxObstacle bo;
        bo.x = obstacle_msg->circles[i].center.x;
        bo.y = obstacle_msg->circles[i].center.y;
        bo.heading = current_state.yaw;                             // TODO: for now, set arbitrary heading for testing purposes
        bo.width = obstacle_msg->circles[i].true_radius;            // TODO: for now, use true radius of circle obstacle as width attribute
        bo.length = obstacle_msg->circles[i].radius;                // TODO: for now, use radius of circle obstacle as length attribute
        box_obstacles.push_back(bo);

        visualization_msgs::Marker box_obstacle_marker;
        VisualizationHelpers::createBoxObstacleMarker(box_obstacles[i], box_obstacle_marker);
        box_obstacle_rviz_pub.publish(box_obstacle_marker);
    }
}

void KarcherLocalPlannerNode::globalPathCallback(const junior_local_planner::WaypointArray::ConstPtr& global_path_msg)
{
    b_global_path = true;

    global_path.clear();

    Waypoint wp;
    for(unsigned int i = 0; i < global_path_msg->waypoints.size(); i++)
    {
        wp.x = global_path_msg->waypoints[i].x;
        wp.y = global_path_msg->waypoints[i].y;
        wp.heading = global_path_msg->waypoints[i].heading;
        wp.left_width = global_path_msg->waypoints[i].left_width;
        wp.right_width = global_path_msg->waypoints[i].right_width;
        wp.cost = 0;
        wp.speed = 0;

        global_path.push_back(wp);
    }

    nav_msgs::Path path;
    // viz_helper.createGlobalPathMarker(global_path, path);
    VisualizationHelpers::createGlobalPathMarker(global_path, path);
    global_path_rviz_pub.publish(path);
}

void KarcherLocalPlannerNode::publishCmdVel()
{
    geometry_msgs::Twist cmd_vel_msg;

    cmd_vel_msg.linear.x = 0.5;
    cmd_vel_msg.linear.y = 0;
    cmd_vel_msg.linear.z = 0;

    cmd_vel_msg.angular.x = 0;
    cmd_vel_msg.angular.y = 0;
    cmd_vel_msg.angular.z = 0;

    cmd_vel_pub.publish(cmd_vel_msg);
}

void KarcherLocalPlannerNode::extractGlobalPathSection(std::vector<Waypoint>& extracted_path)
{
    if(global_path.size() < 2) return;

    extracted_path.clear();

    Waypoint car_pos;
    car_pos.x = current_state.x; car_pos.y = current_state.y; car_pos.heading = current_state.yaw;
    int closest_index = PlannerHelpers::getClosestNextWaypointIndex(global_path, car_pos);

    if(closest_index + 1 >= global_path.size())
        closest_index = global_path.size() - 2;
    
    // prev_closest_index = closest_index;
    // prev_cost = global_path[prev_closest_index].cost;

    double d = 0;

    // include points before the closest next point
    // for(int i = closest_index; i >= 0; i--)
    // {
    //     extracted_path.insert(extracted_path.begin(), global_path[i]);

    //     if(i < global_path.size())
    //         d += hypot(global_path[i].x - global_path[i+1].x, global_path[i].y - global_path[i+1].y);
    //     if(d > 10) 
    //         break;
    // }

    // d = 0;
    for(int i = closest_index; i < (int)global_path.size(); i++)
    {
        extracted_path.push_back(global_path[i]);
        if(i > 0)
            d += hypot(global_path[i].x - global_path[i-1].x, global_path[i].y - global_path[i-1].y);
        if(d > MAX_LOCAL_PLAN_DISTANCE_)
            break;
    }

    if(extracted_path.size() < 2)
    {
        ROS_WARN("Karcher Local Planner Node: Extracted Global Plan is too small, Size = %d", (int)extracted_path.size());
        return;
    }

    PlannerHelpers::fixPathDensity(extracted_path, PATH_DENSITY_);
    PlannerHelpers::smoothPath(extracted_path, SMOOTH_TOLERANCE_, SMOOTH_DATA_WEIGHT_, SMOOTH_WEIGHT_);
    PlannerHelpers::calculateAngleAndCost(extracted_path, prev_cost);

    nav_msgs::Path path;
    VisualizationHelpers::createExtractedPathMarker(extracted_path, path);
    extracted_path_rviz_pub.publish(path);
}

void KarcherLocalPlannerNode::generateRollOuts(const std::vector<Waypoint>& path, std::vector<std::vector<Waypoint>>& roll_outs)
{
    // std::cout << "path size: " << path.size() << std::endl;
    if(path.size() == 0) return;
    if(MAX_LOCAL_PLAN_DISTANCE_ <= 0) return;
    roll_outs.clear();

    int i_limit_index = (SAMPLING_TIP_MARGIN_/0.3)/PATH_DENSITY_;
    if(i_limit_index >= path.size())
        i_limit_index = path.size() - 1;
    // std::cout << "i_limit_index: " << i_limit_index << std::endl;

    int closest_index;
    double initial_roll_in_distance;
    
    RelativeInfo info;
    Waypoint car_pos;
    car_pos.x = current_state.x; car_pos.y = current_state.y; car_pos.heading = current_state.yaw;
    PlannerHelpers::getRelativeInfo(path, car_pos, info);
    initial_roll_in_distance = info.perp_distance;
    // std::cout << "closest_index: " << closest_index << std::endl;
    // std::cout << "initial_roll_in_distance: " << initial_roll_in_distance << std::endl;

    double remaining_distance = 0;
    for(int i = 0; i < path.size()-1; i++)
    {
        remaining_distance += distance2points(path[i], path[i+1]);
    }
    // std::cout << "remaining_distance: " << remaining_distance << std::endl;

    // calculate the starting index
    double d_limit = 0;
    int start_index = 0; 
    int end_index = 0;
    // int far_index = closest_index;

    // calculate end index
    double start_distance = ROLL_IN_SPEED_FACTOR_ * current_state.speed + ROLL_IN_MARGIN_;
    if(start_distance > remaining_distance)
        start_distance = remaining_distance;
    // std::cout << "start_distance: " << start_distance << std::endl;

    d_limit = 0;
    for(int i = 0; i < path.size()-1; i++)
    {
        d_limit += distance2points(path[i], path[i+1]);

        if(d_limit >= start_distance)
        {
            end_index = i;
            break;
        }
    }
    // std::cout << "far_index: " << far_index << std::endl;

    int central_trajectory_index = ROLL_OUTS_NUMBER_/2;
    std::vector<double> end_laterals;
    for(int i = 0; i< ROLL_OUTS_NUMBER_+1; i++)
    {
        double end_roll_in_distance = ROLL_OUT_DENSITY_ * (i - central_trajectory_index);
        end_laterals.push_back(end_roll_in_distance);
        // std::cout << "roll out num: " << i << ", end_roll_in_distance: " << end_roll_in_distance << std::endl;
    }

    // calculate the actual calculation starting index
    d_limit = 0;
    int smoothing_start_index = start_index;
    int smoothing_end_index = end_index;

    for(int i = smoothing_start_index; i < path.size(); i++)
    {
        if(i > 0)
            d_limit += distance2points(path[i], path[i-1]);
        if(d_limit > SAMPLING_TIP_MARGIN_)
            break;

        smoothing_start_index++;
    }

    d_limit = 0;
    for(int i = end_index; i < path.size(); i++)
    {
        if(i > 0)
            d_limit += distance2points(path[i], path[i-1]);
        if(d_limit > SAMPLING_TIP_MARGIN_)
            break;

        smoothing_end_index++;
    }
    // std::cout << "start_index: " << start_index << ", end_index: " << end_index << ", smoothing_start_index: " 
    //             << smoothing_start_index << ", smoothing_end_index: " << smoothing_end_index << std::endl;

    int nSteps = end_index - smoothing_start_index;
    // std::cout << "nSteps: " << nSteps << std::endl;

    std::vector<double> inc_list;
    std::vector<double> inc_list_inc;
    for(int i = 0; i< ROLL_OUTS_NUMBER_+1; i++)
    {
        double diff = end_laterals[i] - initial_roll_in_distance;
        // std::cout << "diff: " << diff << std::endl;
        inc_list.push_back(diff/(double)nSteps);
        roll_outs.push_back(std::vector<Waypoint>());
        inc_list_inc.push_back(0);
    }

    std::vector<std::vector<Waypoint>> excluded_from_smoothing;
    for(int i = 0; i < ROLL_OUTS_NUMBER_+1; i++)
        excluded_from_smoothing.push_back(std::vector<Waypoint>());

    Waypoint wp;
    // Insert first straight points within the tip of the car range
    for(int j = start_index; j < smoothing_start_index; j++)
    {
        wp = path[j];
        double original_speed = wp.speed;
        for(int i = 0; i < ROLL_OUTS_NUMBER_+1; i++)
        {
            wp.x = path[j].x - initial_roll_in_distance * cos(wp.heading + M_PI_2);
            wp.y = path[j].y - initial_roll_in_distance * sin(wp.heading + M_PI_2);
            if(i != central_trajectory_index)
                wp.speed = original_speed * LANE_CHANGE_SPEED_FACTOR_;
            else
                wp.speed = original_speed;

            if(j < i_limit_index)
                excluded_from_smoothing[i].push_back(wp);
            else
                roll_outs[i].push_back(wp);
        }
    }

    for(int j = smoothing_start_index; j < end_index; j++)
    {
        wp = path[j];
        double original_speed = wp.speed;
        for(int i = 0; i < ROLL_OUTS_NUMBER_+1; i++)
        {
            inc_list_inc[i] += inc_list[i];
            double d = inc_list_inc[i];
            wp.x = path[j].x - initial_roll_in_distance * cos(wp.heading + M_PI_2) - d * cos(wp.heading + M_PI_2);
            wp.y = path[j].y - initial_roll_in_distance * sin(wp.heading + M_PI_2) - d * sin(wp.heading + M_PI_2);

            if(i != central_trajectory_index)
                wp.speed = original_speed * LANE_CHANGE_SPEED_FACTOR_;
            else
                wp.speed = original_speed;

            roll_outs[i].push_back(wp);
        }
    }

    // Insert last straight points to make better smoothing
    for(int j = end_index; j < smoothing_end_index; j++)
    {
        wp = path[j];
        double original_speed = wp.speed;
        for(int i = 0; i < ROLL_OUTS_NUMBER_+1; i++)
        {
            double d = end_laterals[i];
            wp.x = path[j].x - d * cos(wp.heading + M_PI_2);
            wp.y = path[j].y - d * sin(wp.heading + M_PI_2);
            if(i != central_trajectory_index)
                wp.speed = original_speed * LANE_CHANGE_SPEED_FACTOR_;
            else
                wp.speed = original_speed;
            roll_outs[i].push_back(wp);
        }
    }

    for(int i = 0; i < ROLL_OUTS_NUMBER_+1; i++)
        roll_outs[i].insert(roll_outs[i].begin(), excluded_from_smoothing[i].begin(), excluded_from_smoothing[i].end());

    d_limit = 0;
    for(int j = smoothing_end_index; j < path.size(); j++)
    {
        if(j > 0)
            d_limit += distance2points(path[j], path[j-1]);

        if(d_limit > MAX_LOCAL_PLAN_DISTANCE_) //max_roll_distance)
            break;

        wp = path[j];
        double original_speed = wp.speed;
        for(int i = 0; i < roll_outs.size(); i++)
        {
            double d = end_laterals[i];
            wp.x = path[j].x - d * cos(wp.heading + M_PI_2);
            wp.y = path[j].y - d * sin(wp.heading + M_PI_2);

            if(i != central_trajectory_index)
                wp.speed = original_speed * LANE_CHANGE_SPEED_FACTOR_;
            else
                wp.speed = original_speed;

            roll_outs[i].push_back(wp);
        }
    }

    for(int i = 0; i < ROLL_OUTS_NUMBER_+1; i++)
    {
        PlannerHelpers::smoothPath(roll_outs[i], SMOOTH_TOLERANCE_, SMOOTH_DATA_WEIGHT_, SMOOTH_WEIGHT_);
        PlannerHelpers::calculateAngleAndCost(roll_outs[i], prev_cost);
        PlannerHelpers::predictConstantTimeCostForTrajectory(roll_outs[i], current_state);
    }

    visualization_msgs::MarkerArray roll_out_marker_array;
    // viz_helper.createRollOutsMarker(roll_outs, roll_out_marker_array);
    VisualizationHelpers::createRollOutsMarker(roll_outs, roll_out_marker_array);
    roll_outs_rviz_pub.publish(roll_out_marker_array);
}

PathCost KarcherLocalPlannerNode::doOneStepStatic(const std::vector<std::vector<Waypoint>>& roll_outs, const std::vector<Waypoint>& extracted_path, std::vector<PathCost>& trajectory_costs)
{
    PathCost bestTrajectory;
    bestTrajectory.bBlocked = false;
    bestTrajectory.closest_obj_distance = HORIZON_DISTANCE_;
    bestTrajectory.closest_obj_velocity = 0;
    bestTrajectory.index = -1;

    RelativeInfo car_info;
    Waypoint car_pos;
    car_pos.x = current_state.x; car_pos.y = current_state.y; car_pos.heading = current_state.yaw;
    PlannerHelpers::getRelativeInfo(extracted_path, car_pos, car_info);
    int curr_index = ROLL_OUTS_NUMBER_/2 + floor(car_info.perp_distance/ROLL_OUT_DENSITY_);
    //std::cout <<  "Current Index: " << curr_index << std::endl;
    if(curr_index < 0)
        curr_index = 0;
    else if(curr_index > ROLL_OUTS_NUMBER_)
        curr_index = ROLL_OUTS_NUMBER_;

    trajectory_costs.clear();
    if(roll_outs.size() > 0)
    {
        PathCost tc;
        int central_index = ROLL_OUTS_NUMBER_/2;
        tc.lane_index = 0;
        for(int it = 0; it < roll_outs.size(); it++)
        {
            tc.index = it;
            tc.relative_index = it - central_index;
            tc.distance_from_center = ROLL_OUT_DENSITY_*tc.relative_index;
            tc.priority_cost = fabs(tc.distance_from_center);
            tc.closest_obj_distance = HORIZON_DISTANCE_;
            // if(roll_outs[it].size() > 0)
            //     tc.lane_change_cost = roll_outs[it][0].lane_change_cost;
            tc.bBlocked = false;
            trajectory_costs.push_back(tc);
        }
    }

    PlannerHelpers::calculateTransitionCosts(trajectory_costs, curr_index, ROLL_OUT_DENSITY_);

    // // obstacle contour points
    // Waypoint p;
    // std::vector<Waypoint> contour_points;
    // // m_AllContourPoints.clear();
    // for(int io = 0; io < obj_list.size(); io++)
    // {
    //     for(int icon = 0; icon < obj_list[io].contour.size(); icon++)
    //     {
    //         p.pos = obj_list[io].contour[icon];
    //         p.v = obj_list[io].center.v;
    //         p.id = io;
    //         p.cost = sqrt(obj_list.at(io).w*obj_list.at(io).w + obj_list.at(io).l*obj_list.at(io).l);
    //         contour_points.push_back(p);
    //     }
    // }

    std::vector<Waypoint> contour_points;
    for(int io = 0; io < box_obstacles.size(); io++)
    {
        Mat3 rotationMat(box_obstacles[io].heading-M_PI_2);
        Mat3 translationMat(box_obstacles[io].x, box_obstacles[io].y);

        Waypoint bottom_left;
        bottom_left.x = -box_obstacles[io].width/2.0;
        bottom_left.y = -box_obstacles[io].length/2.0;

        Waypoint bottom_right; 
        bottom_right.x = box_obstacles[io].width/2.0;
        bottom_right.y = -box_obstacles[io].length/2.0;

        Waypoint top_right;
        top_right.x = box_obstacles[io].width/2.0;
        top_right.y = box_obstacles[io].length/2.0;

        Waypoint top_left;
        top_left.x = -box_obstacles[io].width/2.0;
        top_left.y = box_obstacles[io].length/2.0;

        bottom_left = rotationMat*bottom_left;
        bottom_left = translationMat*bottom_left;

        bottom_right = rotationMat*bottom_right;
        bottom_right = translationMat*bottom_right;

        top_right = rotationMat*top_right;
        top_right = translationMat*top_right;

        top_left = rotationMat*top_left;
        top_left = translationMat*top_left;

        contour_points.push_back(bottom_left);
        contour_points.push_back(bottom_right);
        contour_points.push_back(top_right);
        contour_points.push_back(top_left);
    }

    visualization_msgs::Marker car_footprint_marker, safety_box_marker;
    PlannerHelpers::calculateLateralAndLongitudinalCostsStatic(trajectory_costs, roll_outs, extracted_path, contour_points, 
                                                                current_state, car_footprint_marker, safety_box_marker,
                                                                VEHICLE_LENGTH_, VEHICLE_WIDTH_, 
                                                                WHEELBASE_LENGTH_, HORIZONTAL_SAFETY_DISTANCE_, 
                                                                VERTICAL_SAFETY_DISTANCE_, MAX_STEER_ANGLE_,
                                                                MIN_FOLLOWING_DISTANCE_, LATERAL_SKIP_DISTANCE_);

    car_footprint_rviz_pub.publish(car_footprint_marker);
    safety_box_rviz_pub.publish(safety_box_marker);

    PlannerHelpers::calculateCurvatureCosts(trajectory_costs, roll_outs);

    PlannerHelpers::normalizeCosts(trajectory_costs, PRIORITY_WEIGHT_, TRANSITION_WEIGHT_, LAT_WEIGHT_, LONG_WEIGHT_, CURVATURE_WEIGHT_);

    int smallestIndex = -1;
    double smallestCost = DBL_MAX;
    double smallestDistance = DBL_MAX;
    double velo_of_next = 0;

    //cout << "Trajectory Costs Log : CurrIndex: " << currIndex << " --------------------- " << endl;
    for(int ic = 0; ic < trajectory_costs.size(); ic++)
    {
        //cout << m_PathCosts.at(ic).ToString();
        if(!trajectory_costs[ic].bBlocked && trajectory_costs[ic].cost < smallestCost)
        {
            smallestCost = trajectory_costs[ic].cost;
            smallestIndex = ic;
        }

        if(trajectory_costs[ic].closest_obj_distance < smallestDistance)
        {
            smallestDistance = trajectory_costs[ic].closest_obj_distance;
            velo_of_next = trajectory_costs[ic].closest_obj_velocity;
        }
    }
    //cout << "Smallest Distance: " <<  smallestDistance << "------------------------------------------------------------- " << endl;

    if(smallestIndex == -1)
    {
        bestTrajectory.bBlocked = true;
        bestTrajectory.lane_index = 0;
        bestTrajectory.index = -1; // TODO
        bestTrajectory.closest_obj_distance = smallestDistance;
        bestTrajectory.closest_obj_velocity = velo_of_next;
    }
    else if(smallestIndex >= 0)
    {
        bestTrajectory = trajectory_costs[smallestIndex];
    }

    visualization_msgs::MarkerArray weighted_roll_out_marker_array;
    // viz_helper.createWeightedRollOutsMarker(roll_outs, trajectory_costs, smallestIndex, weighted_roll_out_marker_array);
    VisualizationHelpers::createWeightedRollOutsMarker(roll_outs, trajectory_costs, smallestIndex, weighted_roll_out_marker_array);
    weighted_trajectories_rviz_pub.publish(weighted_roll_out_marker_array);

    // m_PrevIndex = currIndex;
    return bestTrajectory;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "junior_local_planner_node");
    KarcherLocalPlannerNode junior_local_planner_obj;  
    ros::spin();
    return 0;
}