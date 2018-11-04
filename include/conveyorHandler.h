//Class specific helper
#ifndef CONVEYOR_HANDLER_H
#define CONVEYOR_HANDLER_H
//Project specific imports
#include <ros/ros.h>
#include <std_srvs/Trigger.h> // this message type is defined in the current package
#include <osrf_gear/ConveyorBeltControl.h>
#include <osrf_gear/DroneControl.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <iostream>
#include <string>
using namespace std;


class conveyorHandler{
public:
    conveyorHandler(ros::NodeHandle* nodehandle);
    void startConveyor();
    void stopConveyor();
    void callDronePickup();

protected:
    ros::NodeHandle nh_;
    
}
#endif //CONVEYOR_HANDLER_H