
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

// given front vehicle speed, what is the 
// minimum distance beyond which ego car can drive full speed
double Planner::fullSpeedDist(double front_speed) {
    if (front_speed >= SPEED_LIMIT) {
        return 1000000.0;
    } else {
        double slow_down_time = (SPEED_LIMIT - front_speed)/ACCEL_LIMIT;
        return SPEED_LIMIT*slow_down_time - 0.5*ACCEL_LIMIT*slow_down_time*slow_down_time
                + DIST_BUFFER;

    }
}

// generate trajectory for the ego vehicle,
// given the target position
vector<vector<double>> Planner::generateTrajectory(int target_lane) {
    vector<vector<double>> trajectory(2); // [0]: x, [1]: y
    // start with the previous path points from last time
    for (int i = 0; i < _previous_path_x.size(); i++) {
        trajectory[0].push_back(_previous_path_x[i]);
        trajectory[1].push_back(_previous_path_y[i]);

        //cout << "Prev " << _previous_path_x[i] << ", " << _previous_path_y[i] << endl;
    }
    
    // starting point of the new trajectory to be calculated
    int prev_size = _previous_path_x.size();
    double start_s = _ego._s;
    if (prev_size > 0) {
        start_s = _end_path_s;
        if (start_s < _ego._s) {
            start_s += _road.MAX_S;
        }
    }

    double start_x = _ego._x;
    double start_y = _ego._y;
    double start_yaw = _ego._yaw;

    // build spline points
    vector<double> ptsx;
    vector<double> ptsy;
    if (prev_size < 2) {
        double prev_car_x = _ego._x - cos(_ego._yaw);
        double prev_car_y = _ego._y - sin(_ego._yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(_ego._x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(_ego._y);
    } else {
        start_x = _previous_path_x[prev_size-1];
        start_y = _previous_path_y[prev_size-1];

        double prev_x = _previous_path_x[prev_size-2];
        double prev_y = _previous_path_y[prev_size-2];
        
        start_yaw = atan2(start_y - prev_y, start_x - prev_x);
        ptsx.push_back(prev_x);
        ptsx.push_back(start_x);

        ptsy.push_back(prev_y);
        ptsy.push_back(start_y);
    }

    // determine the target of the new trajectory, 
    // depending on the front vehicle info
    double target_speed = SPEED_LIMIT, target_s = _ego._s + PLANNING_DIST; 
    double last_dist = start_s - _ego._s; // the speed was already calulated in the last round up to last_dist
    
    Vehicle front_vehicle;
    bool found = getFrontVehicle(target_lane, front_vehicle);
    if (found) {
        double front_dist = _road.distance(front_vehicle, _ego);
        double full_speed_dist = fullSpeedDist(front_vehicle._v) + last_dist;
        cout << "front dist: " << front_dist << " full speed dist: " << full_speed_dist;
        cout << " front speed: " << front_vehicle._v  << endl;

        if (front_dist < full_speed_dist && front_dist > DIST_BUFFER ) {
            target_speed = front_vehicle._v;
        } else if (front_dist < DIST_BUFFER) {
            target_speed = min(5.0, front_vehicle._v);
        }

        // warp up
        double front_s = (front_vehicle._s < _ego._s)? front_vehicle._s + _road.MAX_S : front_vehicle._s;
        target_s = min(front_s + front_vehicle._v*TIME_STEP*prev_size, target_s);
    }

    cout << "targe speed: " << target_speed  << " ego v: " << _ego._v << endl;
    cout << "target s: " << target_s << " ego s: " << _ego._s << " end path s: " << _end_path_s << endl;
    double s_gap = target_s - start_s;

    double target_d = _road.laneCenter(target_lane);
    vector<double> next_wp0 = getXY(start_s + s_gap/2, target_d, _road._map_waypoints_s,
                                    _road._map_waypoints_x, _road._map_waypoints_y);
    vector<double> next_wp1 = getXY(start_s + s_gap, target_d, _road._map_waypoints_s,
                                    _road._map_waypoints_x, _road._map_waypoints_y);
    //vector<double> next_wp2 = getXY(start_s + s_gap, target_d, _road._map_waypoints_s,
    //                                _road._map_waypoints_x, _road._map_waypoints_y);
    
    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    //ptsx.push_back(next_wp2[0]);
    
    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    //ptsy.push_back(next_wp2[1]);

    for (size_t i = 0; i < ptsx.size(); i++) {
        // shift car reference angle to 0 degree
        double shift_x = ptsx[i] - start_x;
        double shift_y = ptsy[i] - start_y;

        ptsx[i] = (shift_x * cos(-start_yaw) - shift_y * sin(-start_yaw));
        ptsy[i] = (shift_x * sin(-start_yaw) + shift_y * cos(-start_yaw));
    }

    // create a spline
    tk::spline s;
    s.set_points(ptsx, ptsy);

    // calculate how to break up spline points so that we travel at our desired reference velocity
    double target_x = target_s - start_s;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x*target_x + target_y*target_y);

    double x = 0;
    double next_v = _ego._v;
    // Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
    cout << "prev_size: " << prev_size << endl;
    for (int i = 0; i < NUM_TRAJECTORY_POINTS - prev_size; ++i) {
        if (next_v < target_speed) {
            next_v += min(target_speed - next_v, ACCEL_LIMIT * TIME_STEP);
        } else if (next_v > target_speed) {
            next_v -= min(next_v - target_speed, ACCEL_LIMIT * TIME_STEP);
        }

        double N = target_dist / (TIME_STEP * next_v);  // each TIME_STEP a new point is reached
        x += target_x/N;
        double y = s(x);

        // rotating back to normal after rotating it earlier
        double x_point = (x * cos(start_yaw) - y * sin(start_yaw));
        double y_point = (x * sin(start_yaw) + y * cos(start_yaw));
        x_point += start_x;
        y_point += start_y;

        trajectory[0].push_back(x_point);
        trajectory[1].push_back(y_point);

        //cout << "new " << x_point << ", " << y_point << endl;
    }
    //cout << "trajectory x: " << endl;
    //showVector(trajectory[0]);

    //cout << endl << "trajectory y: " << endl;
    //showVector(trajectory[1]);
    return trajectory;
}