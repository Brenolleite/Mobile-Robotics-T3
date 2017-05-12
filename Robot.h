#ifndef ROBOT_H
#define ROBOT_H

#define NUM_SONARS  16
#define LOG         1

#include <fstream>
#include <iostream>
#include "Simulator.h"
#include <stdlib.h>
#include <math.h>
#include<vector>

extern "C" {
   #include "extApi.h"
   #include "v_repLib.h"
}

class Robot
{
public:
    Robot(Simulator *sim, std::string name);
    void update();
    void updateSensors();
    void updatePose();
    void printPose();
    void writeGT();
    void writeSonars();
    void move(float vLeft, float vRight);
    double vRToDrive(double vLinear, double vAngular);
    double vLToDrive(double vLinear, double vAngular);
    void drive(double vLinear, double vAngular);
    void stop();
    //------Métodos Adicionados--------------------------
    void updateOdom();
    std::vector<float> getOdometry();
    void initOdometry();
    void polarErrorCalc(float poseAtual[3]); //Essa função só recebe uma pose para que a gente troque odometria por groundTruth
    void goToGoal();

private:
    const float L = 0.381;                                   // distance between wheels
    const float R = 0.0975;                                  // wheel radius
    std::string name;
    Simulator *sim;

    simxInt handle;                                        // robot handle
    simxFloat velocity[2] = {1,1};                         // wheels' speed
    simxInt sonarHandle[16];                               // handle for sonars
    simxInt motorHandle[2] = {0,0};                        // [0]-> leftMotor [1]->rightMotor
    simxInt encoderHandle[2] = {0,0};
    simxFloat encoder[2] = {0,0};
    simxFloat lastEncoder[2] = {0,0};

    /* Robot Position  */
    simxFloat robotPosition[3] = {0,0,0};                    // current robot position
    simxFloat robotOrientation[3] = {0,0,0};                 // current robot orientation
    float initialPose[3] = {0,0,0};
    simxFloat robotLastPosition[3] = {0,0,0};                // last robot position
    float sonarReadings[NUM_SONARS];

//---------------Privates Adicionados---------------------
    /* Ground Truth */
    float robotPose[3] = {0,0,0};
    float robotLastPose[3] = {0,0,0};

    /* Robot Odometry  */
    float odomPose[3] = {0,0,0};                    // current odometry based pose
    float odomLastPose[3] = {0,0,0};                // last odometry based pose

    /*Robot Control*/
    float polarError[3] = {0,0,0};
    float goal[3] = {0,0,0};
    bool atGoal = false;


};

#endif // ROBOT_H
