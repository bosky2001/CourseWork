#include <vector>
#include <iostream>
#include <math.h>
#include <fstream>

using namespace std;

class Car{
public:
    //constructor class
    Car(double f, double r);
    //methods to set initial conditions and run the simulation
    void setConstants(double f, double r);
    void setInitialConditions(double x_i, double y_i, double v_i, double psi_i);
    void setInputs(double a, double steer);
    void simTracj(double t);
    void writeTofile(string fileName);
    // arrays of values at each time step
    vector<double> Xpos;              
    vector<double> Ypos;
    vector<double> speed;
    vector<double> heading;
    vector<double> time;

private:
    // to figure dynamics of model
    const float dT;
    double frontDist;
    double rearDist;
    double slip;
    double acclInput;
    double steeringAngle;
};