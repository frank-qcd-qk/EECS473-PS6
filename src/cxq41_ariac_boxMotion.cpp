#include <ros/ros.h>
#include <std_srvs/Trigger.h> // this message type is defined in the current package
#include <osrf_gear/ConveyorBeltControl.h>
#include <osrf_gear/DroneControl.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <iostream>
#include <string>
using namespace std;

//! Global Variabls:
bool g_take_new_snapshot = false;
osrf_gear::LogicalCameraImage g_cam2_data; // Global information storage holder

//! Call back for Camera2
void camera2CallBack(const osrf_gear::LogicalCameraImage& message_Holder){
    if(g_take_new_snapshot){
        ROS_INFO_STREAM("Image from Cam 1: "<<message_Holder<<endl);
        g_cam2_data = message_Holder;
        g_take_new_snapshot = false;
    }
}

//! Start Competition Callback
void start_competition(ros::NodeHandle & startCompetitionNode){
    ros::ServiceClient startup_client = startCompetitionNode.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    if(!startup_client.exists()){
        ROS_INFO("[Startup Phase]Waiting for the competition to be ready...");
        startup_client.waitForExistence();
        ROS_INFO("[Startup Phase]Competition is now ready...");
    }
    ROS_INFO("[Startup Phase]Requesting competition to start...");
    std_srvs::Trigger startup_srv;
    startup_srv.response.success = false;
    startup_client.call(startup_srv);
    while(!startup_srv.response.success){
        ROS_WARN("[Startup Phase]Not successful startup yet... Repeat Calling...");
        startup_client.call(startup_srv);
        ros::Duration(0.1).sleep();
    }
    ROS_INFO("[Startup Phase]Competition Startup Success>>>[Belt Control Phase]");
}

//! Stop Conveyor Callback
void stopConveyor(ros::NodeHandle & stopConveyorNode){
    ros::ServiceClient stop_conveyor = stopConveyorNode.serviceClient<osrf_gear::ConveyorBeltControl>("/ariac/conveyor/control");
    if(!stop_conveyor.exists()){
        ROS_INFO("[Conveyor Halt]Waiting for the conveyor stop service to be ready...");
        stop_conveyor.waitForExistence();
        ROS_INFO("[Conveyor Halt]Conveyor stop Service now ready...")
    }
    ROS_INFO("[Conveyor Halt]Stopping Conveyor...")
    osrf_gear::ConveyorBeltControl conveyor_srv_stop;
    conveyor_srv_stop.request.power = 0.0;
    conveyor_srv_stop.response.success = false;
    stop_conveyor.call(conveyor_srv_stop);
    while(!conveyor_srv_stop.response.success){
        ROS_WARN("[Conveyor Halt]Not successful stopping conveyor... Repeat Calling...");
        stop_conveyor.call(conveyor_srv_stop);
        ros::Duration(0.1).sleep();
    }
    ROS_INFO("[Conveyor Halt]Conveyor Stop Success");
}

//! Start Conveyor Callback
void startConveyor(ros::NodeHandle & startConveyorNode){
    ros::ServiceClient start_conveyor = startConveyorNode.serviceClient<osrf_gear::ConveyorBeltControl>("/ariac/conveyor/control");
    if(!stop_conveyor.exists()){
        ROS_INFO("[Conveyor Start]Waiting for the conveyor start service to be ready...");
        stop_conveyor.waitForExistence();
        ROS_INFO("[Conveyor Start]Conveyor start Service now ready...")
    }
    ROS_INFO("[Conveyor Start]Starting Conveyor...")    
    osrf_gear::ConveyorBeltControl conveyor_srv_start;
    conveyor_srv_start.request.power = 0.0;
    conveyor_srv_start.response.success = false;
    start_conveyor.call(conveyor_srv_start);
    while(!conveyor_srv_start.response.success){
        ROS_WARN("[Conveyor Start]Not successful starting conveyor... Repeat Calling...");
        start_conveyor.call(conveyor_srv_start);
        ros::Duration(0.1).sleep();
    }
    ROS_INFO("[Conveyor start]Conveyor Start Success");
}

//! Request Drone Callback
void requestDrone(ros::NodeHandle & droneNode){
    ros::ServiceClient drone_client = droneNode.serviceClient<osrf_gear::DroneControl>("/ariac/drone");
    if(!drone_client.exists()){
        ROS_INFO("[Drone call]Waiting for the drone service to be ready...");
        drone_client.waitForExistence();
        ROS_INFO("[Drone call]drone Service now ready...")
    }
    ROS_INFO("[Drone call]Calling Drone...")
    osrf_gear::DroneControl drone_srv;
    drone_srv.request.shipment_type = "Dummy Shipment";
    drone_srv.response.success = false;
    drone_client.call(drone_srv);
    while(!drone_srv.response.success){
        ROS_WARN("[Drone call]Not successful starting Drone... Repeat Calling...");
        drone_client.call(drone_srv);
        ros::Duration(0.5).sleep();        
    }
    ROS_INFO("[Drone call]Drone Call Success");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cxq41_ariac_boxMotion"); //Initialization of ros
    ros::NodeHandle n; //Creates RoS Node Handle
    //! Initialization Code:
    ros::Subscriber camera2= n.subscribe("/ariac/logical_camera_2",1,camera2CallBack); 
    
    //! Competition Startup Call
    start_competition(n);
    //! Start the conveyor immediately.
    start_conveyor();
    
    while(ros::ok()){
        while(g_cam2_data.models.size()>0){
            ROS_INFO("The current box z position is: %f",g_cam2_data.pose.position.z);
            ros::spinOnce(); //allow data from call back
            ros::Duration(0.5).sleep();
        }
        ros::spinOnce();
    }

    return 0;
}
