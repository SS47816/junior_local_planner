#ifndef STRUCT_DEFS_H_
#define STRUCT_DEFS_H_

#include <vector>
// namespace local_planner {

typedef struct  
{
    double x;
    double y;
    double heading;
    double left_width;
    double right_width;
    double speed;
    double cost;
    double time_cost;
} Waypoint;

typedef struct
{
    double x;
    double y;
    double heading;
    double width;
    double length; 
} BoxObstacle;

typedef struct  
{
    double x;
    double y;
    double yaw;
    double speed;
    double steer;
} VehicleState;

typedef struct 
{
    int front_index;
    int back_index;
    double perp_distance;
    double to_front_distance;
    double from_back_distance;
    double angle_diff;
    Waypoint perp_point;
} RelativeInfo;

typedef struct
{
    int index;
    int relative_index;
    double closest_obj_velocity;
    double distance_from_center;
    double priority_cost; //0 to 1
    double transition_cost; // 0 to 1
    double closest_obj_cost; // 0 to 1
    double cost;
    double closest_obj_distance;

    int lane_index;
    double lane_change_cost;
    double lateral_cost;
    double longitudinal_cost;
    double curvature_cost;
    bool bBlocked;
    std::vector<std::pair<int, double>> lateral_costs;
} PathCost;

typedef struct
{
    double x;
    double y;
    double vx;
    double vy;
    double radius;
    double true_radius; 
} CircleObstacle;

// } // namespace local_planner

#endif  // STRUCT_DEFS_H_