#ifndef ROAD_H_
#define ROAD_H_

#include <vector>
#include <string>

using std::vector;
using std::string;

class Vehicle;

class Road {
public:
    Road();
    ~Road();

    void load_map(const string& file_name);

    // is v1 in front of v2?
    bool ahead(const Vehicle& vehicle1, const Vehicle& vehicle2);

    // is v1 behind of v2?
    bool behind(const Vehicle& vehicle1, const Vehicle& vehicle2);

    double distance(const Vehicle& front_vehicle, const Vehicle& vehicle);

    double laneCenter(int lane) const;
    int getLane(double d) const;
    bool nearLandBoundary(double d) const;
    int neighborLane(double d) const;

public:
    const int LANE_WIDTH = 4;
    const int NUM_LANES = 3;
    const double MAX_S = 6945.554;

    vector<double> _map_waypoints_x;
    vector<double> _map_waypoints_y;
    vector<double> _map_waypoints_s;
    vector<double> _map_waypoints_dx;
    vector<double> _map_waypoints_dy;
};

#endif // ROAD_H_