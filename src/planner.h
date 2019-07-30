#ifndef PLANNER_H_
#define PLANNER_H_

#include <vector>
#include <string>
#include "vehicle.h"
#include "road.h"

using std::vector;
using std::string;

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
    void getPreviousPath(const vector<double>& previous_path_x, const vector<double>& previous_path_y);

    // predict positions of road vehicles in t seconds
    vector<Vehicle> generatePredictions(double t);

    // planning the next movement
    int behaviorPlanning();

    // generate trajectory for the ego vehicle,
    // given the target position
    vector<vector<double>> generateTrajectory(int target_lane);

    // find front vehicle of the ego vehicle on a given lane
    bool getFrontVehicle(int lane, Vehicle& front_vehicle);

public:
    Vehicle _ego;
    vector<Vehicle> _road_vehicles;
    Road _road;

    vector<double> _previous_path_x;
    vector<double> _previous_path_y;

    const int NUM_PATH_POINTS = 50;
    const double TIME_STEP = 0.02;
    const double SPEED_LIMIT = 50*1600.0/3600;  // m/s
    const double ACCEL_LIMIT = 10;  // m/s^2
    const double JERK_LIMIT = 10;   // m/s^3

};
#endif // PLANNER_H_