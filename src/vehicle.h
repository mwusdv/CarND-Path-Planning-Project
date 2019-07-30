#ifndef VEHICLE_H_
#define VEHICLE_H_

#include <vector>

using std::vector;

class Road;

class Vehicle {
public:
    Vehicle();
    Vehicle(const Vehicle& vehicle);
    Vehicle(const vector<double>& sensor_data, const Road& road);
    
    ~Vehicle();

    void setStatus(double x, double y, double s, 
                   double d, double v, double yaw,
                   const Road& road);

    // predict next positions
    vector<Vehicle> generatePredictions(double t, const Road& road);

public:
    int _id;
    int _lane;
    double _x, _y;
    double _s, _d;
    double _v, _yaw;
};

#endif // VEHICLE_H_