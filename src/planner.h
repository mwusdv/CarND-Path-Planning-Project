#ifndef PLANNER_H_
#define PLANNER_H_

#include <vector>
#include <string>
#include "vehicle.h"
#include "road.h"

using std::vector;
using std::string;

struct LaneInfo {
    int _lane;
    
    double _front_s;
    double _front_dist;
    double _front_speed;

    double _rear_dist;
    double _rear_speed;
};

class Planner {
public:
    Planner();
    ~Planner();

    // load map
    void load_map(const string& file_name);

    // get updated vehicle info
    void updateEgo(double x, double y, double s,
                   double d, double v, double yaw);

    void updateRoadVehicles(const vector<vector<double>>& senor_fusion);

    // preious path
    void getPreviousPath(const vector<double>& previous_path_x, const vector<double>& previous_path_y, 
                         double end_path_s, double end_path_d);

    // predict positions of road vehicles in t seconds
    vector<Vehicle> generatePredictions(double t);

    // planning the next movement
    int behaviorPlanning();

    // check a given lane
    LaneInfo checkLane(int lane);

    // evaluate running on a lane
    double evaluateLane(const LaneInfo& lane_info, double& target_s, double& target_speed);

    // choose next lane
    void chooseLane(int& target_lane, double& target_s, double& target_speed,
                    LaneInfo& target_lane_info);

    // generate trajectory for the ego vehicle,
    // given the target position
    vector<vector<double>> generateTrajectory();

    // find front vehicle of the ego vehicle on a given lane
    bool getFrontVehicle(int lane, Vehicle& front_vehicle);

    // find rear vehicle of the ego vehicle on a given lane
    bool getRearVehicle(int lane, Vehicle& rear_vehicle);

    // given front vehicle speed, what is the 
    // minimum distance beyond which ego car can drive full speed
    double fullSpeedDist(double front_speed);

    // validate a trajectory
    void validTrajectory(const vector<double>& x_points, const vector<double>& y_points);

public:
    Vehicle _ego;
    vector<Vehicle> _road_vehicles;
    Road _road;

    vector<double> _previous_path_x;
    vector<double> _previous_path_y;
    double _end_path_s;
    double _end_path_d;

    double _end_v;
    double _end_accel;

    const int NUM_TRAJECTORY_POINTS = 50;
    const double PLANNING_DIST = 30; // at most planning the next 30 meters
    const double TIME_STEP = 0.02;
    const double SPEED_LIMIT = 49.5*1600.0/3600;  // m/s
    const double ACCEL_LIMIT = 9.5;  // m/s^2
    const double JERK_LIMIT = 9.5;   // m/s^3
    const double DIST_BUFFER = 25;

};
#endif // PLANNER_H_