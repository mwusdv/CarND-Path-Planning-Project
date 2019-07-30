#include <math.h>
#include <sstream>
#include <fstream>

#include "road.h"

Road::Road() {
    // empty
}

Road::~Road() {
    // empty
}

void Road::load_map(const string& file_name) {
    std::ifstream in_map_(file_name.c_str(), std::ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        std::istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        _map_waypoints_x.push_back(x);
        _map_waypoints_y.push_back(y);
        _map_waypoints_s.push_back(s);
        _map_waypoints_dx.push_back(d_x);
        _map_waypoints_dy.push_back(d_y);
    }
}

double Road::laneCenter(int lane) const {
    return (lane + 0.5) * LANE_WIDTH;
}

int Road::getLane(double d) const {
    return d/LANE_WIDTH;
}

bool Road::nearLandBoundary(double d) const {
    int lane = getLane(d);
    double center = laneCenter(lane);
    double dist = fabs(center - d);
    double ratio = dist / (LANE_WIDTH/2);
    return (ratio > 0.75);
 }

int Road::neighborLane(double d) const {
    int lane = getLane(d);
    double center = laneCenter(lane);
    if (d < center) {
        return lane - 1;
    }
    return lane + 1;
}