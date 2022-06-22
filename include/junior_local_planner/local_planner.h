#ifndef LOCAL_PLANNER_H_
#define LOCAL_PLANNER_H_

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

#include "junior_local_planner/struct_defs.h"
#include "junior_local_planner/matrix.h"
#include "junior_local_planner/polygon.h"
#include "junior_local_planner/visualization_helper.h"

#include "obstacle_detector/Obstacles.h"

#include <math.h>
#include <limits>
#include <vector>

#define _USE_MATH_DEFINES
#define RAD2DEG 180.0 / M_PI
#define distance2points(from , to) sqrt(pow(to.x - from.x, 2) + pow(to.y - from.y, 2))
#define distance2pointsSqr(from , to) pow(to.x - from.x, 2) + pow(to.y - from.y, 2)
#define pointNorm(v) sqrt(v.x*v.x + v.y*v.y)

class LocalPlanner
{
public:
    class Params
    {
    public: 
        Params(){};
        Params(double max_speed, double max_local_plan_distance, double path_density,
                int roll_outs_number, double sampling_tip_margin, double sampling_out_margin, 
                double roll_out_density, double roll_in_speed_factor, double roll_in_margin,
                double lane_change_speed_factor, double horizon_distance, double horizontal_safety_distance,
                double vertical_safety_distance, double max_steer_angle, double min_speed,
                double lateral_skip_distance, double min_following_distance, double max_following_distance,
                double min_distance_to_avoid, double vehicle_width, double vehicle_length,
                double wheelbase_length, double turning_radius, double safety_radius, double smooth_data_weight, 
                double smooth_weight, double smooth_tolerance, double priority_weight, double transition_weight, 
                double lat_weight, double long_weight, double collision_weight) :
                max_speed(max_speed), max_local_plan_distance(max_local_plan_distance), path_density(path_density),
                roll_outs_number(roll_outs_number), sampling_tip_margin(sampling_tip_margin), sampling_out_margin (sampling_out_margin), 
                roll_out_density(roll_out_density), roll_in_speed_factor(roll_in_speed_factor), roll_in_margin(roll_in_margin),
                lane_change_speed_factor(lane_change_speed_factor), horizon_distance(horizon_distance), horizontal_safety_distance(horizontal_safety_distance),
                vertical_safety_distance(vertical_safety_distance), max_steer_angle(max_steer_angle), min_speed(min_speed),
                lateral_skip_distance(lateral_skip_distance), min_following_distance(min_following_distance), max_following_distance(max_following_distance),
                min_distance_to_avoid(min_distance_to_avoid), vehicle_width(vehicle_width), vehicle_length(vehicle_length),
                wheelbase_length(wheelbase_length), turning_radius(turning_radius), safety_radius(safety_radius), smooth_data_weight(smooth_data_weight), 
                smooth_weight(smooth_weight), smooth_tolerance(smooth_tolerance), priority_weight(priority_weight), transition_weight(transition_weight), 
                lat_weight(lat_weight), long_weight(long_weight), collision_weight(collision_weight) {};
        virtual ~Params(){};

    private:
        // parameters
        double max_speed;                  // max speed that planner should not exceed
        double max_local_plan_distance;    // length of local trajectory roll outs
        double path_density;               // distance between waypoints of local trajectory
        int roll_outs_number;              // number of roll outs not including the center tracjectory (this number should be even)
        double sampling_tip_margin;        // length of car tip margin
        double sampling_out_margin;        // length of roll in margin (??)
        double roll_out_density;           // distance between adjacent trajectories
        double roll_in_speed_factor;
        double roll_in_margin;
        double lane_change_speed_factor;
        double horizon_distance;

        double horizontal_safety_distance;
        double vertical_safety_distance;
        double max_steer_angle;
        double min_speed;
        double lateral_skip_distance;

        double min_following_distance;      // distance threshold for exiting following behaviour
        double max_following_distance;      // distance threshold for entering following behaviour
        double min_distance_to_avoid;       // distance threshold for obstacle avoidance behaviour

        double vehicle_width;        
        double vehicle_length;
        double wheelbase_length;
        double turning_radius;
        double safety_radius;
        
        // Smoothing Weights
        double smooth_data_weight; 
        double smooth_weight;
        double smooth_tolerance;

        double priority_weight;
        double transition_weight;
        double lat_weight;
        double long_weight;
        double collision_weight;        
    };

    // Constructors
    LocalPlanner(){};
    LocalPlanner(Params params);

    // Destructor
    virtual ~LocalPlanner(){};

private:
    Params params_;

    // Local Planner functions
    void extractGlobalPathSection(std::vector<Waypoint>& extracted_path);
    int getClosestNextWaypointIndex(const std::vector<Waypoint>& path, const Waypoint& current_pos);
    double angleBetweenTwoAnglesPositive(const double& a1, const double& a2);
    double fixNegativeAngle(const double& a);
    void fixPathDensity(std::vector<Waypoint>& path);
    void smoothPath(std::vector<Waypoint>& path);
    double calculateAngleAndCost(std::vector<Waypoint>& path);
    void generateRollOuts(const std::vector<Waypoint>& path, std::vector<std::vector<Waypoint>>& roll_outs);
    bool getRelativeInfo(const std::vector<Waypoint>& path, const Waypoint& current_pos, RelativeInfo& info);
    void predictConstantTimeCostForTrajectory(std::vector<Waypoint>& path);
    PathCost doOneStepStatic(const std::vector<std::vector<Waypoint>>& roll_outs, const std::vector<Waypoint>& extracted_path, std::vector<PathCost>& trajectory_costs);
    void calculateTransitionCosts(std::vector<PathCost>& trajectory_costs, const int& curr_trajectory_index);
    void calculateLateralAndLongitudinalCostsStatic(std::vector<PathCost>& trajectory_costs, const std::vector<std::vector<Waypoint>>& roll_outs, 
                                                    const std::vector<Waypoint>& extracted_path, std::vector<Waypoint>& contour_points);
    double getExactDistanceOnTrajectory(const std::vector<Waypoint>& trajectory, const RelativeInfo& p1, const RelativeInfo& p2);
    void normalizeCosts(std::vector<PathCost>& trajectory_costs);
    double checkTrajectoryForCollision(const std::vector<Waypoint>& trajectory);
};

LocalPlanner::LocalPlanner(LocalPlanner::Params params)
{
    this->params_ = params;
}


// } // namespace local_planner

#endif  // LOCAL_PLANNER_H_