#include "Robot.h"

Robot::Robot(Simulator *sim, std::string name) {
    this->sim = sim;
    this->name = name;
    handle = sim->getHandle(name);

    if (LOG) {
        FILE *data =  fopen("gt.txt", "wt");
        if (data!=NULL)
            fclose(data);
        data =  fopen("sonar.txt", "wt");
        if (data!=NULL)
            fclose(data);
    }

    /* Get handles of sensors and actuators */
    encoderHandle[0] = sim->getHandle("Pioneer_p3dx_leftWheel");
    encoderHandle[1] = sim->getHandle("Pioneer_p3dx_rightWheel");
    std::cout << "Left Encoder: "<< encoderHandle[0] << std::endl;
    std::cout << "Right Encoder: "<< encoderHandle[1] << std::endl;

    /* Get handles of sensors and actuators */
    motorHandle[0] = sim->getHandle("Pioneer_p3dx_leftMotor");
    motorHandle[1] = sim->getHandle("Pioneer_p3dx_rightMotor");
    std::cout << "Left motor: "<<  motorHandle[0] << std::endl;
    std::cout << "Right motor: "<<  motorHandle[1] << std::endl;

    simxChar sensorName[31];
    /* Connect to sonar sensors. Requires a handle per sensor. Sensor name: Pioneer_p3dx_ultrasonicSensorX, where
     * X is the sensor number, from 1 - 16 */
    for(int i = 0; i < 16; i++)
    {
        sprintf(sensorName,"%s%d","Pioneer_p3dx_ultrasonicSensor",i+1);
        sonarHandle[i] = sim->getHandle(sensorName);
        if (sonarHandle[i] == -1)
            std::cout <<  "Error on connecting to sensor " + i+1 << std::endl;
        else
        {
            std::cout << "Connected to sensor\n";
        }
        sim->readProximitySensor(sonarHandle[i],NULL,NULL,0);
    }

    /* Get the robot current absolute position */
    sim->getObjectPosition(handle,robotPosition);
    sim->getObjectOrientation(handle,robotOrientation);

    initialPose[0]=robotPosition[0];
    initialPose[1]=robotPosition[1];
    initialPose[2]=robotOrientation[2];

    /* Inicializa a posição da odometria*/
    odomPose[0] = robotPosition[0];
    odomPose[1] = robotPosition[1];
    odomPose[2] = robotOrientation[2];

    for(int i = 0; i < 3; i++)
    {
        robotLastPosition[i] = robotPosition[i];
    }

    /* Get the encoder data */
    sim->getJointPosition(motorHandle[0],&encoder[0]);
    sim->getJointPosition(motorHandle[1],&encoder[1]);
    std::cout << encoder[0] << std::endl;
    std::cout << encoder[1] << std::endl;

    estado = toGoal;
}

void Robot::update() {
    updateSensors();
    updatePose();
    updateOdom();
    polarErrorCalc(robotPose);
    switch(estado)
    {
        case stand:
             drive(0,0);
             break;
        case toGoal:
            goToGoal();
            break;
        case wallfollow:
            drive(0,20); // Só pra diferenciar nos testes.
            break;
        case avoidFuzzy:
            break;
    }
    checkRobotState();
}

//--------------------------------------Métodos T2----------------------------------------------------------------
void Robot::updateOdom()
{
    for(int i = 0; i < 3; i++)
    {
        odomLastPose[i] = odomPose[i];
    }
    double s,sr,sl;
    if(encoder[0]>=0  && lastEncoder[0]>=0 || encoder[0]<=0  && lastEncoder[0]<=0)
    {
        sl = R*(encoder[0]-lastEncoder[0]);
    }
    else if(encoder[0]>=0  && lastEncoder[0]<0 || encoder[0]<0  && lastEncoder[0]>=0)
    {
        sl = abs(R*(encoder[0]+lastEncoder[0]));
    }

    if(encoder[1]>=0  && lastEncoder[1]>=0 || encoder[1]<=0  && lastEncoder[1]<=0)
    {
        sr = R*(encoder[1]-lastEncoder[1]);
    }
    else if(encoder[1]>=0  && lastEncoder[1]<0 || encoder[1]<0  && lastEncoder[1]>=0)
    {
        sr = abs(R*(encoder[1]+lastEncoder[1]));
    }

    s = ((sr + sl))/2;
    odomPose[2] = ((sr - sl))/L+ odomLastPose[2]; //theta
    odomPose[0] = odomLastPose[0] + s*cos(odomPose[2]); //x
    odomPose[1] = odomLastPose[1] + s*sin(odomPose[2]); //y
    std::cout<<odomPose[0];
}

std::vector<float> Robot::getOdometry()
{
    std::vector<float> auxVector;
    auxVector[0] = odomPose[0];
    auxVector[1] = odomPose[1];
    auxVector[2] = odomPose[2];

    return auxVector;
}

void Robot::initOdometry()
{
    sim->getObjectPosition(handle,robotPosition);
    sim->getObjectOrientation(handle,robotOrientation);
    odomPose[0] = robotPosition[0];
    odomPose[1] = robotPosition[1];
    odomPose[2] = robotOrientation[2];
}

void Robot::polarErrorCalc(float poseAtual[])
{
    float alfa, beta, p, dx, dy;

    dx = goal[0] - poseAtual[0];
    dy = goal[1] - poseAtual[1];

    p = sqrt(dx*dx + dy*dy);
    alfa = - poseAtual[2] + atan2(dy,dx);
    beta = - poseAtual[2] - alfa;

    polarError[0] = p;
    polarError[1] = alfa;
    polarError[2] = beta;

    //std::cout<<"p = "<<polarError[0]<<" alfa=  "<<polarError[1]<<" beta= "<<polarError[2];
}

void Robot::goToGoal()
{
    float v, w, kRho = 50, kAlfa = 100, kBeta= -5;
    v = kRho*polarError[0];
    if(v>20)
    {
        v = 20;
    }
    w = kAlfa*polarError[1] + kBeta*polarError[2];
    //std::cout<<"Velocidade: "<<v<<"   Angular: "<<w;
    drive(v,w);
}

void Robot::setRobotState(state e)
{
    estado = e;
}

void Robot::checkRobotState()
{
    if (polarError[0] < limiar)
    {
        std::cout<<"\nI made it to the goal!!\n";
        atGoal = true;
        setRobotState(stand);
    }
    else
    {
        atGoal = false;
        if(obstaclesInWay() || estado == wallfollow)
        {
            std::cout<<"\nFollowing the wall...\n";
            setRobotState(wallfollow);
        }
        else
        {
            setRobotState(toGoal);
        }
    }
}

bool Robot::obstaclesInWay()
{
    float minValue;
    minValue = std::min(std::min(sonarReadings[2],sonarReadings[3]),std::min(sonarReadings[2],sonarReadings[3]));
    std::cout<<"\nMínimo: "<<minValue<<"\n";
    if(minValue < minSonarValue && minValue >0.05)
        return true;
    else
        return false;
}

void Robot::followTheWall()
{
    float minLeft, minRight, setPoint;
    minLeft = sonarReadings[0];
    minRight = sonarReadings[7];

}

void Robot::writePointsSonars(float position[])
{
  float x, y;
  if (LOG) {
    FILE *data =  fopen("points.txt", "at");

    if (data!=NULL){

      for (int i=0; i<8; ++i){
        if(sonarReadings[i] > 0){
          x = position[0] + (sonarReadings[i] + rSonar) * cos(robotOrientation[2] + (sonarAngles[i]*M_PI)/180);
          y = position[1] + (sonarReadings[i] + rSonar) * sin(robotOrientation[2] + (sonarAngles[i]*M_PI)/180);
          fprintf(data, "%.4f \t %.4f \n", x, y);
        }
      }
      fflush(data);
      fclose(data);
    }
  }
}

//--------------------------------------Fim Métodos T2------------------------------------------------------------

void Robot::updateSensors()
{
    /* Update sonars */
    for(int i = 0; i < NUM_SONARS; i++)
    {
        simxUChar state;       // sensor state
        simxFloat coord[3];    // detected point coordinates [only z matters]

        /* simx_opmode_streaming -> Non-blocking mode */

        /* read the proximity sensor
         * detectionState: pointer to the state of detection (0=nothing detected)
         * detectedPoint: pointer to the coordinates of the detected point (relative to the frame of reference of the sensor) */
        if (sim->readProximitySensor(sonarHandle[i],&state,coord,1)==1)
        {
            if(state > 0)
                sonarReadings[i] = coord[2];
            else
                sonarReadings[i] = -1;
        }
    }
    /* Update encoder data */
    lastEncoder[0] = encoder[0];
    lastEncoder[1] = encoder[1];

    /* Get the encoder data */
    if (sim->getJointPosition(motorHandle[0], &encoder[0]) == 1)
        std::cout << "ok left enconder"<< encoder[0] << std::endl;  // left
    if (sim->getJointPosition(motorHandle[1], &encoder[1]) == 1)
        std::cout << "ok right enconder"<< encoder[1] << std::endl;  // right

}

void Robot::updatePose()
{
    for(int i = 0; i < 3; i++)
    {
        robotLastPosition[i] = robotPosition[i];
    }

    robotLastPose[0] = robotPosition[0];
    robotLastPose[1] = robotPosition[1];
    robotLastPose[2] = robotOrientation[2];

    /* Get the robot current position and orientation */
    sim->getObjectPosition(handle,robotPosition);
    sim->getObjectOrientation(handle,robotOrientation);

    robotPose[0] = robotPosition[0];
    robotPose[1] = robotPosition[1];
    robotPose[2] = robotOrientation[2];

}

void Robot::writeGT() {
    /* write data to file */
    /* file format: robotPosition[x] robotPosition[y] robotPosition[z] robotLastPosition[x] robotLastPosition[y] robotLastPosition[z]
     *              encoder[0] encoder[1] lastEncoder[0] lastEncoder[1] */

    if (LOG) {
        FILE *data =  fopen("gt.txt", "at");
        if (data!=NULL)
        {
            for (int i=0; i<3; ++i)
                fprintf(data, "%.2f\t",robotPosition[i]);
            for (int i=0; i<3; ++i)
                fprintf(data, "%.2f\t",robotLastPosition[i]);
            for (int i=0; i<3; ++i)
                fprintf(data, "%.2f\t",robotOrientation[i]);
            for (int i=0; i<2; ++i)
                fprintf(data, "%.2f\t",encoder[i]);
            for (int i=0; i<2; ++i)
                fprintf(data, "%.2f\t",lastEncoder[i]);
            fprintf(data, "\n");
            fflush(data);
            fclose(data);
          }
          else
            std::cout << "Unable to open file";
    }
}

void Robot::writeSonars() {
    /* write data to file */
    /* file format: robotPosition[x] robotPosition[y] robotPosition[z] robotLastPosition[x] robotLastPosition[y] robotLastPosition[z]
     *              encoder[0] encoder[1] lastEncoder[0] lastEncoder[1] */
    if (LOG) {
        FILE *data =  fopen("sonar.txt", "at");
        if (data!=NULL)
        {
            if (data!=NULL)
            {
                for (int i=0; i<NUM_SONARS; ++i)
                    fprintf(data, "%.2f\t",sonarReadings[i]);
                fprintf(data, "\n");
                fflush(data);
                fclose(data);
            }
        }
    }
}

void Robot::printPose() {
    std::cout << "[" << robotPosition[0] << ", " << robotPosition[1] << ", " << robotOrientation[2] << "]" << std::endl;
}

void Robot::move(float vLeft, float vRight) {
    sim->setJointTargetVelocity(motorHandle[0], vLeft);
    sim->setJointTargetVelocity(motorHandle[1], vRight);
}

void Robot::stop() {
    sim->setJointTargetVelocity(motorHandle[0], 0);
    sim->setJointTargetVelocity(motorHandle[1], 0);
}

void Robot::drive(double vLinear, double vAngular)
{
    sim->setJointTargetVelocity(motorHandle[0], vLToDrive(vLinear,vAngular));
    sim->setJointTargetVelocity(motorHandle[1], vRToDrive(vLinear,vAngular));
}

double Robot::vRToDrive(double vLinear, double vAngular)
{
    return (((2*vLinear)+(L*vAngular))/2*R);
}

double Robot::vLToDrive(double vLinear, double vAngular)
{
    return (((2*vLinear)-(L*vAngular))/2*R);

}
