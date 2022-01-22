#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <tiago_iaslab_simulation/MoveTiagoAction.h>
#include <tiago_iaslab_simulation/PickPlaceAction.h>
#include <tiago_iaslab_simulation/DetectionAction.h>

#include <tiago_iaslab_simulation/posesObjs.h>

#include "tiago_iaslab_simulation/Objs.h"
#include <cstdlib>



std::vector<int> getObjID(){
    std::vector<int> obj_ids;
    ros::NodeHandle n;
	ros::ServiceClient client =
	n.serviceClient<tiago_iaslab_simulation::Objs>("/human_objects_srv");
	tiago_iaslab_simulation::Objs srv;

	srv.request.ready = true;
	srv.request.all_objs = true;
    if (client.call(srv)) { 
        obj_ids = (srv.response.ids);
    } else { 
        ROS_ERROR("Failed to call service human request");
    }  
    
    return obj_ids;
}


int main(int argc, char* argv[]){
    ros::init(argc, argv, "nodeA");
    actionlib::SimpleActionClient<tiago_iaslab_simulation::MoveTiagoAction> moveClient("moveTiagoServer", true);
    actionlib::SimpleActionClient<tiago_iaslab_simulation::DetectionAction> detectionClient("nodeB_detectionServer", true);
    actionlib::SimpleActionClient<tiago_iaslab_simulation::PickPlaceAction> pickPlaceClient("nodeC_PickPlaceServer", true);
    
    ROS_INFO("Waiting for servers.");

    moveClient.waitForServer(); 
    float posX, posY, yaw;
    
    std::vector<int> obj_ids = getObjID();

	for(int i = 0; i < obj_ids.size(); i ++){
		int obj_id = obj_ids.at(i);

		switch (obj_id){
			case 1: //BLU
				ROS_INFO("OBJECT SELECTED: BLUE HEXAGON");
				posX = 8.00;								 
				posY = -1.90;									
				yaw = -1.40;
				break;
			case 2: //GREEN
				ROS_INFO("OBJECT SELECTED: GREEN TRIANGLE");
				posX = 7.8;
				posY = -4.15;								
				yaw = M_PI_2;
				break;
			case 3: //RED
				ROS_INFO("OBJECT SELECTED: RED CUBE");
				posX = 7.50 ;
				posY = -1.85;
				yaw = -1.40;
				break;
		}

		//Define the pick position of the robot specifying the X and Y coordinates and Yaw angle.
		tiago_iaslab_simulation::MoveTiagoGoal PosGoal;  
		PosGoal.X = posX;
		PosGoal.Y = posY;
		PosGoal.yaw = yaw;

		//Define intermediate position before pick table
		tiago_iaslab_simulation::MoveTiagoGoal waypointPickPosGoal;  
		waypointPickPosGoal.X = 8.0;
		waypointPickPosGoal.Y = 0.0;
		waypointPickPosGoal.yaw = -M_PI_2;


		//Send the waypointGoal to the server
		moveClient.sendGoal(waypointPickPosGoal);
		if (!moveClient.waitForResult(ros::Duration(120.0))){
			ROS_ERROR("CANNOT MOVE TO FIRST WAYPOINT");
			return 1;
		}
		ROS_INFO("REACHED FIRST WAYPOINT");
		

		//Send the pick position goal to the server
		moveClient.sendGoal(PosGoal);;
		if(!moveClient.waitForResult(ros::Duration(120.0))){
			ROS_ERROR("CANNOT MOVE TO PICK POINT");
			return 1;
		}
		ROS_INFO("REACHED PICK POINT");

		//OBJECTS DETECTION
		detectionClient.waitForServer();
		tiago_iaslab_simulation::DetectionGoal detGoal;
		detGoal.go = 1;
		detectionClient.sendGoal(detGoal);

		if (!detectionClient.waitForResult(ros::Duration(120.0))){
			ROS_INFO("Detection did not finish before the time out.");
			return 1;
		}
		//Get the detection result, a poses' vector of the detected objects. In position 0 there is the target object pose
		tiago_iaslab_simulation::DetectionResultConstPtr result_detection = detectionClient.getResult();

		//PICK ACTION
		pickPlaceClient.waitForServer();
		int pick = 0;
		tiago_iaslab_simulation::PickPlaceGoal pickPlaceGoal;
		pickPlaceGoal.detectedObjs = result_detection->detectedObjs;
		pickPlaceGoal.ObjType = obj_id;
		pickPlaceGoal.actionType = pick;
		pickPlaceClient.sendGoal(pickPlaceGoal);

		if (!pickPlaceClient.waitForResult(ros::Duration(180.0))){
			ROS_ERROR("OBJECT NOT PICKED BEFORE TUMEOUT.");
			return 1;
		}else if (pickPlaceClient.getState() == actionlib::SimpleClientGoalState::ABORTED){
			ROS_ERROR("PICK FAILED");
			return 1;
		}

		ROS_INFO("OBJECT PICKED.");

		//GOAL POSITION IN FRONT OF PLACE TABLE
		tiago_iaslab_simulation::MoveTiagoGoal placePosGoal;
		switch (obj_id){
			case 1: //BLU
				posX = 12.50;
				break;
			case 2: //GREEN
				posX = 11.50;
				break;
			case 3: //RED
				posX = 10.50;
				break;
		}

		placePosGoal.X = posX;
		placePosGoal.Y = 0.500;         
		placePosGoal.yaw = -M_PI_2;    

		// Define waypoint for the place postion.
		tiago_iaslab_simulation::MoveTiagoGoal waypointPlacePosGoal;
		waypointPlacePosGoal.X = 9.0;
		waypointPlacePosGoal.Y = 0.5;
		waypointPlacePosGoal.yaw = 0;

		//Send the waypointGoal to the move server
		moveClient.sendGoal(waypointPlacePosGoal);
		if (!moveClient.waitForResult(ros::Duration(120.0))){
			ROS_ERROR("CANNOT MOVE TO SECOND WAYPOINT");
			return 1;
		}
		ROS_INFO("REACHED PLACE WAYPOINT");

		//Send the placeGoal to the move erver
		moveClient.sendGoal(placePosGoal);
		if (!moveClient.waitForResult(ros::Duration(120.0))){
			ROS_ERROR("CANNOT MOVE TO PLACE POINT");
			return 1;
		}
		ROS_INFO("REACHED PLACE POSITION");

		//PLACE ACTION
		int place = 1;
		pickPlaceGoal.detectedObjs = result_detection->detectedObjs;
		pickPlaceGoal.actionType = place;
		pickPlaceClient.sendGoal(pickPlaceGoal);
		if (!pickPlaceClient.waitForResult(ros::Duration(120.0))){
			ROS_INFO("OBJECT NOT PLACED BEFORE TIMEOUT");
		}else if (pickPlaceClient.getState() == actionlib::SimpleClientGoalState::ABORTED){
			ROS_ERROR("PLACE FAILED");
			return 1;
		}
		
		ROS_INFO("OBJECT PLACED SUCCESFULLY");
	}


	ROS_INFO("ALL OBJECTS PLACED SUCCESSFULLY");
	
	return 0;
}