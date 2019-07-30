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
    vector<double> behaviorPlanning();

    // generate trajectory for the ego vehicle,
    // given the target position
    vector<vector<double>> generateTrajectory(int lane, double s);

public:
    Vehicle _ego;
    vector<Vehicle> _road_vehicles;
    Road _road;

    vector<double> _previous_path_x;
    vector<double> _previous_path_y;

    const int NUM_PATH_POINTS = 50;
    const double TIME_STEP = 0.02;

};
#endif // PLANNER_H_