#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <iostream>

extern "C" {
   #include "extApi.h"
   #include "v_repLib.h"
}

class Simulator
{

public:
    Simulator(std::string ip, int portNumber);
    int connect();
    void disconnect();
    void pause();
    void resume();
    int getHandle(std::string name);
    int readProximitySensor(simxInt sensorHandle, simxUChar *state, float *coord, int init);
    int getObjectPosition(simxInt sensorHandle, float *coord);
    int getObjectOrientation(simxInt sensorHandle, float *coord);
    int getJointPosition(simxInt jointHandle, float *coord);
    int setJointTargetVelocity(simxInt jointHandle, float velocity);
    void setSynchronous();
    void startSimulation();
    void stopSimulation();
private:
    int id;
    int portNumber;
    std::string ip;

};

#endif // SIMULATOR_H
