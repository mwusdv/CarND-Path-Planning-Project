#ifndef ROAD_H_
#define ROAD_H_

#include <vector>
#include <string>

using std::vector;
using std::string;


class Road {
public:
    Road();
    ~Road();

    void load_map(const string& file_name);

    double laneCenter(int lane) const;
    int getLane(double d) const;
    bool nearLandBoundary(double d) const;
    int neighborLane(double d) const;

public:
    const int LANE_WIDTH = 4;
    const int NUM_LANES = 3;

    vector<double> _map_waypoints_x;
    vector<double> _map_waypoints_y;
    vector<double> _map_waypoints_s;
    vector<double> _map_waypoints_dx;
    vector<double> _map_waypoints_dy;
};

#endif // ROAD_H_