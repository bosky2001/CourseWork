#include <thread>
#include <iostream>
#include "car.h"

using namespace std;


int main(){
    Car car1(4.5,2.5);  // car1 initialization
    car1.setInputs(1.5,0);
    car1.setInitialConditions(50,20,20,0);
    double t1 = 1.0; // car1 simulation duration
    string file1 ="car1.txt";

    Car car2(2.5,3.5);  //car 2 initialization
    car2.setInputs(-1,0.05);
    car2.setInitialConditions(0,0,40,0.5);
    double t2 = 1.0; // car2 simulation duration
    string file2 ="car2.txt";

    thread thr1(&Car::simTracj, ref(car1),ref(t1));
    thread thr2(&Car::simTracj, ref(car2),ref(t2));

    thr1.join();
    thr2.join();

    // Car sim
}