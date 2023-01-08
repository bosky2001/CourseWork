#include "car.h"

Car:: Car(double f, double r): dT(0.01){
    setConstants(f,r);
}

void Car::setConstants(double f, double r){
    frontDist = f;
    rearDist = r;
}

void Car::setInitialConditions(double x_i, double y_i, double v_i, double psi_i){
    Xpos.push_back(x_i);
    Ypos.push_back(y_i);
    speed.push_back(v_i);
    heading.push_back(psi_i);
    time.push_back(0.0);
}

void Car::setInputs(double a, double steer){
    acclInput = a;
    steeringAngle = steer;
}

void Car::simTracj(double t){
    int numSamp = t/dT;
    for (int i=0; i<numSamp;i++){
        //using eulers methods
        double timeUpd = time[i]+dT; // simulation for time
        time.push_back(timeUpd);

        double speedUpd = speed[i]+dT*acclInput;          //simulation for speed
        speed.push_back(speedUpd);

        double slipAngle = atan(tan(steeringAngle) *rearDist/(rearDist+frontDist));        // slip angle value

        double headingUpd = heading[i] +dT*((speed[i]/rearDist)*sin(slipAngle));           //updating heading values with time
        heading.push_back(headingUpd);

        double Xupd = Xpos[i] + dT*(speed[i]*cos(heading[i]+slipAngle));                   // Updating X position with time
        Xpos.push_back(Xupd);

        double Yupd = Ypos[i] + dT*(speed[i]*sin(heading[i]+slipAngle));                   // Updating Y position with time
        Ypos.push_back(Yupd);

        cout << speed[i] <<"\n";

    }
}

void Car::writeTofile(string fileName){
    ofstream oFile;
    oFile.open(fileName);
    oFile << "t(s)" << "\t" << "X(m)" << "\t" << "Y(m)" << "\t" << "V(m/s)" << "\t" << "Psi(rad)" << "\t";
    for (int i=0; i<time.size(); i++){
        oFile << time[i] << "\t" << Xpos[i] << "\t" << Ypos[i] << "\t" << speed[i] << "\t" << heading[i] << "\t";
    }
}