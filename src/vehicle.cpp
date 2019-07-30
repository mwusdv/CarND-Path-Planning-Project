#include <math.h>
#include "vehicle.h"
#include "road.h"

Vehicle::Vehicle() 
    :   _id (-1)  // -1: ego vehicle
    ,   _x  (0.0)
    ,   _y  (0.0)
    ,   _s  (0.0)
    ,   _d  (0.0)
    ,   _v  (0.0)
    ,   _yaw(0.0){
    // empty
}

// copy constructor
Vehicle::Vehicle(const Vehicle& vehicle) {
    *this = vehicle;
}

Vehicle::Vehicle(const vector<double>& sensor_data, const Road& road) {
    _id = sensor_data[0];
    double xv = sensor_data[3], yv = sensor_data[4];
    double v  = sqrt(xv*xv + yv*yv);
    double yaw = atan2(yv, xv);

    setStatus(sensor_data[1], sensor_data[2], sensor_data[5], 
              sensor_data[6], v, yaw, road);
}
    
Vehicle::~Vehicle() {
    // empty
}


// copy
const Vehicle& Vehicle::operator=(const Vehicle& vehicle) {
    _id = vehicle._id;
    _x = vehicle._x;
    _y = vehicle._y;
    _s = vehicle._s;
    _d = vehicle._d;
    _v = vehicle._v;
    _yaw = vehicle._yaw;
    _lane = vehicle._lane;

    return *this;
}

void Vehicle::setStatus(double x, double y, double s, 
                        double d, double v, double yaw,
                        const Road& road) {
    _x = x;
    _y = y;
    _s = s;
    _d = d;
    _v = v;
    _yaw = yaw;
    _lane = road.getLane(d);
}

// predict next positions
vector<Vehicle> Vehicle::generatePredictions(double t, const Road& road) {
    vector<Vehicle> predictions;

    // next position in the current lane
    Vehicle vehicle(*this);
    vehicle._s += _v*t;
    predictions.push_back(vehicle);

    // close to the lane boundary, consider lane change
    if (road.nearLandBoundary(_d)) {
        int lane = road.neighborLane(_d);
        if (lane >= 0 && lane < road.NUM_LANES) {
            vehicle._lane = lane;
            predictions.push_back(vehicle);
        }
    }

    return predictions;
}