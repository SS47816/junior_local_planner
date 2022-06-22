#ifndef JUNIOR_LOCAL_PLANNER_NODE_H_
#define JUNIOR_LOCAL_PLANNER_NODE_H_

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "junior_local_planner/Waypoint.h"
#include "junior_local_planner/WaypointArray.h"

#include "junior_local_planner/planner_helpers.h"
#include "junior_local_planner/struct_defs.h"
#include "junior_local_planner/matrix.h"
#include "junior_local_planner/polygon.h"
#include "junior_local_planner/visualization_helpers.h"

#include "obstacle_detector/Obstacles.h"

#include <math.h>
#include <limits>
#include <vector>

class JuniorLocalPlannerNode
{
public:
    // Constructor
    JuniorLocalPlannerNode();

    // Destructor
    virtual ~JuniorLocalPlannerNode(){};

private:
    ros::NodeHandle nh;

    // Hyperparameters
    double planning_frequency_;

    // Parameters
    double MAX_SPEED_;                  // max speed that planner should not exceed
    double MAX_LOCAL_PLAN_DISTANCE_;    // length of local trajectory roll outs
    double PATH_DENSITY_;               // distance between waypoints of local trajectory
    int ROLL_OUTS_NUMBER_;              // number of roll outs not including the center tracjectory (this number should be even)
    double SAMPLING_TIP_MARGIN_;        // length of car tip margin
    double SAMPLING_OUT_MARGIN_;        // length of roll in margin (??)
    double ROLL_OUT_DENSITY_;           // distance between adjacent trajectories
    double ROLL_IN_SPEED_FACTOR_;
    double ROLL_IN_MARGIN_;
    double LANE_CHANGE_SPEED_FACTOR_;
    double HORIZON_DISTANCE_;

    double HORIZONTAL_SAFETY_DISTANCE_;
    double VERTICAL_SAFETY_DISTANCE_;
    double MAX_STEER_ANGLE_;
    double MIN_SPEED_;
    double LATERAL_SKIP_DISTANCE_;

    double MIN_FOLLOWING_DISTANCE_;     // distance threshold for exiting following behaviour
    double MAX_FOLLOWING_DISTANCE_;     // distance threshold for entering following behaviour
    double MIN_DISTANCE_TO_AVOID;       // distance threshold for obstacle avoidance behaviour

    double VEHICLE_WIDTH_;        
    double VEHICLE_LENGTH_;
    double WHEELBASE_LENGTH_;
    double TURNING_RADIUS_;
    double SAFETY_RADIUS_;
    
    // Smoothing Weights
    double SMOOTH_DATA_WEIGHT_; 
    double SMOOTH_WEIGHT_;
    double SMOOTH_TOLERANCE_;

    double PRIORITY_WEIGHT_;
    double TRANSITION_WEIGHT_;
    double LAT_WEIGHT_;
    double LONG_WEIGHT_;
    double COLLISION_WEIGHT_;
    double CURVATURE_WEIGHT_;

    // Subscribers and Publishers
    ros::Subscriber odom_sub;
    ros::Subscriber obstacles_sub;
    ros::Subscriber global_path_sub;

    ros::Publisher global_path_rviz_pub;
    ros::Publisher extracted_path_rviz_pub;
    ros::Publisher current_pose_rviz_pub;
    ros::Publisher roll_outs_rviz_pub;
    ros::Publisher weighted_trajectories_rviz_pub;
    ros::Publisher safety_box_rviz_pub;
    ros::Publisher car_footprint_rviz_pub;
    ros::Publisher box_obstacle_rviz_pub;
    ros::Publisher cmd_vel_pub;

    // Timer
    ros::Timer timer;

    // TF
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    // Main Function 
    void mainTimerCallback(const ros::TimerEvent& timer_event);

    // Functions for subscribing
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
    void obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr& obstacle_msg);
    void globalPathCallback(const junior_local_planner::WaypointArray::ConstPtr& global_path_msg);

    // Functions for publishing results
    void publishCmdVel();

    // Local Planning functions
    void extractGlobalPathSection(std::vector<Waypoint>& extracted_path);
    void generateRollOuts(const std::vector<Waypoint>& path, std::vector<std::vector<Waypoint>>& roll_outs);
    PathCost doOneStepStatic(const std::vector<std::vector<Waypoint>>& roll_outs, const std::vector<Waypoint>& extracted_path, 
                                std::vector<PathCost>& trajectory_costs);

    // Global Variables
    bool b_global_path;
    bool b_vehicle_state;
    bool b_obstacles;

    VehicleState current_state;
    std::vector<Waypoint> global_path;
    std::vector<CircleObstacle> circle_obstacles;
    std::vector<Waypoint> obstacle_waypoints;
    std::vector<BoxObstacle> box_obstacles;

    int prev_closest_index;
    double prev_cost;
};

#endif  // JUNIOR_LOCAL_PLANNER_NODE_H_