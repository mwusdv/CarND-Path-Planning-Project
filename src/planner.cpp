
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

bool Planner::getRearVehicle(int lane, Vehicle& rear_vehicle) {
    double rear_dist = _road.MAX_S*2;
    bool found = false;
    for (size_t i = 0; i < _road_vehicles.size(); ++i) {
        const Vehicle& vehicle = _road_vehicles[i];
        if (vehicle._lane == lane) {
            if (_road.behind(vehicle, _ego)) {
                double dist = _road.distance(_ego, vehicle);
                if (dist < rear_dist) {
                    rear_dist = dist;
                    rear_vehicle = vehicle;
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

// investigate how about running on the given lane
LaneInfo Planner::checkLane(int lane) {  
    LaneInfo lane_info;
    lane_info._lane = lane;

    // front vehicle info
    Vehicle front_vehicle;
    lane_info._front_dist = _road.MAX_S*2;
    lane_info._front_speed = SPEED_LIMIT;
    bool found = getFrontVehicle(lane, front_vehicle);
    if (found) {
        lane_info._front_s = front_vehicle._s;
        lane_info._front_dist = _road.distance(front_vehicle, _ego);
        lane_info._front_speed = front_vehicle._v;
    }

    // rear vehicle info
    Vehicle rear_vehicle;
    lane_info._rear_dist = _road.MAX_S*2;
    lane_info._rear_speed = SPEED_LIMIT;
    found = getRearVehicle(lane, rear_vehicle);
    if (found) {
        lane_info._rear_dist = _road.distance(_ego, rear_vehicle);
        lane_info._rear_speed = rear_vehicle._v;
    }

    return lane_info;
}

// evaluate running on a lane
double Planner::evaluateLane(const LaneInfo& lane_info, double& target_s, double& target_speed) {
    // if possibly collide with rear vehicle, then score is 0
    if (lane_info._lane != _ego._lane && lane_info._rear_dist < DIST_BUFFER) {
        return 0;
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

    // compute target s and target speed on the given lane
    // based on the front vehicle info
    double last_dist = start_s - _ego._s; // the speed was already calulated in the last round up to last_dist
    double full_speed_dist = fullSpeedDist(lane_info._front_speed) + last_dist;
    
    // targe speed
    target_speed = SPEED_LIMIT;
    if (lane_info._front_dist < full_speed_dist && lane_info._front_dist > DIST_BUFFER ) {
        target_speed = lane_info._front_speed;
    } else if (lane_info._front_dist < DIST_BUFFER) {
        target_speed = min(5.0, lane_info._front_speed);
    }

    // target s
    target_s = _ego._s + PLANNING_DIST;    // warp up
    double front_s = (lane_info._front_s < _ego._s)? lane_info._front_s + _road.MAX_S : lane_info._front_s;
    target_s = min(front_s + lane_info._front_speed*TIME_STEP*prev_size, target_s);

    // score: we prefer higher speed, same lane, further target s
    return target_speed - 1*abs(lane_info._lane - _ego._lane) + target_s * 0.001;
}

// choose next lane
void Planner::chooseLane(int& target_lane, double& target_s, double& target_speed,
                         LaneInfo& target_lane_info) {
    // check left, current and right lane, and keep the best one
    double best_score = -1, best_target_s, best_target_speed;
    for (int i = -1; i < 2; ++i) {
        int cur_lane = _ego._lane + i;
        if (cur_lane < 0 || cur_lane >= _road.NUM_LANES) {
            continue;
        }

        LaneInfo cur_info = checkLane(cur_lane);
        double cur_target_s, cur_target_speed;
        double score = evaluateLane(cur_info, cur_target_s, cur_target_speed);

        cout << "checking lane: " << cur_lane << endl;
        cout << "front dist: " << cur_info._front_dist << " front speed: " << cur_info._front_speed << endl;
        cout << "rear dist: " << cur_info._rear_dist << " rear speed: " << cur_info._rear_speed << endl;
        
        if (score > best_score) {
            best_score = score;
            target_lane = cur_lane;
            target_s = cur_target_s;
            target_speed = cur_target_speed;
            target_lane_info = cur_info;
        }
    }
}

// generate trajectory for the ego vehicle,
// given the target position
vector<vector<double>> Planner::generateTrajectory() {
    // choose one lane
    int target_lane;
    LaneInfo target_lane_info;
    double target_s, target_speed;
    chooseLane(target_lane, target_s, target_speed, target_lane_info);

    // start with the previous path points from last time
    vector<vector<double>> trajectory(2); // [0]: x, [1]: y
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

    cout << "target lane: " << target_lane << " ego lane: " << _ego._lane << endl;
    cout << "front dist: " << target_lane_info._front_dist << endl;
    cout << "front speed: " << target_lane_info._front_speed << " ego speed: " << _ego._v << endl;
    cout << "targe speed: " << target_speed << endl;
   
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