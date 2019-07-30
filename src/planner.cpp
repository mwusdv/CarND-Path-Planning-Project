
#include <math.h>
#include <iostream>
#include "spline.h"

#include "planner.h"
#include "helpers.h"


using namespace std;

Planner::Planner() 
    :  _road () {
    // empty
}

Planner::~Planner() {
    // empty
}

void Planner::load_map(const string& file_name) {
    _road.load_map(file_name);
}

// get updated vehicle info
void Planner::updateEgo(double x, double y, double s,
                        double d, double v, double yaw) {
    _ego.setStatus(x, y, s, d, v, deg2rad(yaw), _road);
}

void Planner::updateRoadVehicles(const vector<vector<double>>& senor_fusion) {
    _road_vehicles.clear();
    for (size_t i = 0; i < senor_fusion.size(); ++i) {
        Vehicle vehicle(senor_fusion[i], _road);
        if (vehicle._lane >= 0) {
            _road_vehicles.push_back(vehicle);
        }
    }
}

// predict positions of road vehicles in t secons
vector<Vehicle> Planner::generatePredictions(double t) {
    vector<Vehicle> predictions;
    for (size_t i = 0; i < _road_vehicles.size(); ++i) {
        vector<Vehicle> preds = _road_vehicles[i].generatePredictions(t, _road);
        for (size_t j = 0; j < preds.size(); ++j) {
            predictions.push_back(preds[j]);
        }
    }
}

void Planner::getPreviousPath(const vector<double>& previous_path_x, const vector<double>& previous_path_y,
                              double end_path_s, double end_path_d) {
    _previous_path_x = previous_path_x;
    _previous_path_y = previous_path_y;
    _end_path_s = end_path_s;
    _end_path_d = end_path_d;
}

bool Planner::getFrontVehicle(int lane, Vehicle& front_vehicle) {
    double front_dist = _road.MAX_S*2;
    bool found = false;
    for (size_t i = 0; i < _road_vehicles.size(); ++i) {
        const Vehicle& vehicle = _road_vehicles[i];
        if (vehicle._lane == lane) {
            if (_road.ahead(vehicle, _ego)) {
                double dist = _road.distance(vehicle, _ego);
                if (dist < front_dist) {
                    front_dist = dist;
                    front_vehicle = vehicle;
                    found = true;
                }
            }

        }
    }

    return found;
}

// planning the next movement
int Planner::behaviorPlanning() {
   
    return _ego._lane;
}

    

// generate trajectory for the ego vehicle,
// given the target position
vector<vector<double>> Planner::generateTrajectory(int target_lane) {
    vector<vector<double>> trajectory(2); // [0]: x, [1]: y
   
    int prev_size = _previous_path_x.size();
    double end_s = (prev_size > 0)? _end_path_s : _ego._s;

    vector<double> ptsx;
    vector<double> ptsy;

    double ref_x = _ego._x;
    double ref_y = _ego._y;
    double ref_yaw = _ego._yaw;

    if (prev_size < 2) {
        double prev_car_x = _ego._x - cos(_ego._yaw);
        double prev_car_y = _ego._y - sin(_ego._yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(_ego._x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(_ego._y);
    } else {
        ref_x = _previous_path_x[prev_size-1];
        ref_y = _previous_path_y[prev_size-1];

        double ref_x_prev = _previous_path_x[prev_size-2];
        double ref_y_prev = _previous_path_y[prev_size-2];
        
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);

        cout << "ref_x: " << ref_x << " ref_x_prev: " << ref_x_prev << endl;
    }

    Vehicle front_vehicle;
    bool found = getFrontVehicle(target_lane, front_vehicle);
    double target_speed = SPEED_LIMIT, target_s = 30;
    if (found) {
        target_speed = front_vehicle._v;
        cout << "fron vehicle s: " << front_vehicle._s << endl;
        target_s = _road.distance(front_vehicle, _ego) + target_speed*TIME_STEP*2;
    }
    target_s += _ego._s;

    cout << "targe speed: " << target_speed  << endl;
    cout << "target s: " << target_s << " ego s: " << _ego._s << " end path s: " << _end_path_s << endl;
    cout << "end_s: " << end_s <<  endl;
    double s_gap = target_s - end_s;
    cout << "s_gap: " << s_gap << endl;

    double target_d = _road.laneCenter(target_lane);
    vector<double> next_wp0 = getXY(end_s + s_gap/3, target_d, _road._map_waypoints_s,
                                    _road._map_waypoints_x, _road._map_waypoints_y);
    vector<double> next_wp1 = getXY(end_s + s_gap*2/3, target_d, _road._map_waypoints_s,
                                    _road._map_waypoints_x, _road._map_waypoints_y);
    vector<double> next_wp2 = getXY(end_s + s_gap, target_d, _road._map_waypoints_s,
                                    _road._map_waypoints_x, _road._map_waypoints_y);
    
    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);
    
    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    cout << "ptsx: " << endl;
    showVector(ptsx);

    cout << "ptsy: " << endl;
    showVector(ptsy);

    for (size_t i = 0; i < ptsx.size(); i++) {
        // shift car reference angle to 0 degree
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
        ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
    }

    // create a spline
    tk::spline s;

    // set (x,y) points to the spline
    s.set_points(ptsx, ptsy);

    // start with the previous path points from last time
    cout << "copy previous points" << endl;
    for (int i = 0; i < _previous_path_x.size(); i++) {
        trajectory[0].push_back(_previous_path_x[i]);
        trajectory[1].push_back(_previous_path_y[i]);
    }

    // calculate how to break up spline points so that we travel at our desired reference velocity
    double target_x = target_s;
    double target_y = s(target_x);
    double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));

    double x_add_on = 0;

    // Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
    for (int i = 0; i < NUM_PATH_POINTS - _previous_path_x.size(); ++i) {
        if (_ego._v < target_speed) {
            _ego._v += min(target_speed - _ego._v, ACCEL_LIMIT * TIME_STEP);
        } else if (_ego._v > target_speed) {
            _ego._v -= min(_ego._v, ACCEL_LIMIT * TIME_STEP);
        }

        double N = target_dist / (TIME_STEP * _ego._v);  // each 0.02 seconds a new point is reached, transform miles per hour to m/s
        double x_point = x_add_on + (target_x) / N;
        double y_point = s(x_point);
        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // rotating back to normal after rotating it earlier
        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        trajectory[0].push_back(x_point);
        trajectory[1].push_back(y_point);

        cout << x_point << "," << y_point << endl;
    }

    return trajectory;
}