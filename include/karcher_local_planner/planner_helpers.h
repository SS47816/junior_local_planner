#ifndef PLANNER_HELPERS_H_
#define PLANNER_HELPERS_H_

#include <ros/ros.h>
#include <vector>

#include "junior_local_planner/struct_defs.h"
#include "junior_local_planner/matrix.h"
#include "junior_local_planner/polygon.h"
#include "junior_local_planner/visualization_helpers.h"

#define _USE_MATH_DEFINES
#define RAD2DEG 180.0 / M_PI
#define distance2points(from , to) sqrt(pow(to.x - from.x, 2) + pow(to.y - from.y, 2))
#define distance2pointsSqr(from , to) pow(to.x - from.x, 2) + pow(to.y - from.y, 2)
#define pointNorm(v) sqrt(v.x*v.x + v.y*v.y)

class PlannerHelpers
{
public:
    PlannerHelpers() {};
    virtual ~PlannerHelpers() {};

    static int getClosestNextWaypointIndex(const std::vector<Waypoint>& path, const Waypoint& current_pos);
    static double angleBetweenTwoAnglesPositive(const double& a1, const double& a2);
    static double fixNegativeAngle(const double& a);
    static void fixPathDensity(std::vector<Waypoint> &path, const double& path_density);
    static void smoothPath(std::vector<Waypoint> &path, const double& smooth_tolerance, const double& smooth_data_weight, const double& smooth_weight);
    static double calculateAngleAndCost(std::vector<Waypoint> &path, const double& prev_cost);
    static bool getRelativeInfo(const std::vector<Waypoint>& path, const Waypoint& current_pos, RelativeInfo& info);
    static void predictConstantTimeCostForTrajectory(std::vector<Waypoint>& path, const VehicleState& current_state);
    static double getExactDistanceOnTrajectory(const std::vector<Waypoint>& trajectory, const RelativeInfo& p1, const RelativeInfo& p2);
    static double checkTrajectoryForCollision(const std::vector<Waypoint>& trajectory, const std::vector<CircleObstacle>& circle_obstacles, const std::vector<BoxObstacle>& box_obstacles, 
                                                    const double& safety_radius, const double& vehicle_width, const double& vehicle_length, const double& wheelbase_length, 
                                                    const double& horizontal_safety_distance, const double& vertical_safety_distance);
    static void calculateTransitionCosts(std::vector<PathCost>& trajectory_costs, const int& curr_trajectory_index, const double& roll_out_density);
    static void calculateLateralAndLongitudinalCostsStatic(std::vector<PathCost>& trajectory_costs, const std::vector<std::vector<Waypoint>>& roll_outs, 
                                                                const std::vector<Waypoint>& extracted_path, std::vector<Waypoint>& contour_points, 
                                                                const VehicleState& current_state, visualization_msgs::Marker& car_footprint_marker, 
                                                                visualization_msgs::Marker& safety_box_marker,
                                                                const double& vehicle_length, const double& vehicle_width, 
                                                                const double& wheelbase_length, const double& horizontal_safety_distance, 
                                                                const double& vertical_safety_distance, const double& max_steer_angle,
                                                                const double& min_following_distance, const double& lateral_skip_distance);
    static void calculateCurvatureCosts(std::vector<PathCost>& trajectory_costs, const std::vector<std::vector<Waypoint>>& roll_outs);
    static void normalizeCosts(std::vector<PathCost>& trajectory_costs, const double& priority_weight, 
                        const double& transition_weight, const double& lat_weight, const double& long_weight, const double& curvature_weight);
};

#endif  /* PLANNER_HELPERS_H_ */