#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <tiago_iaslab_simulation/MoveTiagoAction.h>
#include <tiago_iaslab_simulation/MoveTiagoActionFeedback.h>
#include <tiago_iaslab_simulation/PickPlaceAction.h>
#include <tiago_iaslab_simulation/DetectionAction.h>

#include <tiago_iaslab_simulation/posesObjs.h>

#include "tiago_iaslab_simulation/Objs.h"
#include <cstdlib>


/*
#   This node retrive the ID of the object to pick and define the corrsepondent position goal to send to the motionServer.
#   
*/

int getObjID(){
    int obj_id;
    ros::NodeHandle n;
	ros::ServiceClient client =
	n.serviceClient<tiago_iaslab_simulation::Objs>("/human_objects_srv");
	tiago_iaslab_simulation::Objs srv;
	srv.request.ready = true;
    if (client.call(srv)) { 
        obj_id = (int)(srv.response.ids.at(0));
        ROS_INFO("Object ID: %i", obj_id); 
        srv.request.ready = false;
    } else { 
        ROS_ERROR("Failed to call service human request");
        return 1; 
    }  
    
    return obj_id;
}


int main(int argc, char* argv[]){
    ros::init(argc, argv, "nodeA");
    actionlib::SimpleActionClient<tiago_iaslab_simulation::MoveTiagoAction> moveClient("moveTiagoServer", true);
    actionlib::SimpleActionClient<tiago_iaslab_simulation::DetectionAction> detectionClient("nodeB_detectionServer", true);
    actionlib::SimpleActionClient<tiago_iaslab_simulation::PickPlaceAction> pickPlaceClient("nodeC_PickPlaceServer", true);


    
    ROS_INFO("Waiting for servers to start.");

    moveClient.waitForServer(); 
    ROS_INFO("Action server started, sending goal.");
    float posX, posY, yaw;
    
    int obj_id = getObjID();

    switch (obj_id)
    {
        case 1: //BLU
            posX = 8.00 ;
            posY = -2.10;
            yaw = -1.57;
            break;
        case 2: //GREEN
            posX = 7.8;
            posY = -4.20;
            yaw = 1.57;
            break;
        case 3: //RED
            posX = 7.50 ;
            posY = -2.10;
            yaw = -1.57;
            break;
    }

    //Define the goal specifying the X and Y coordinates and Yaw angle.
    tiago_iaslab_simulation::MoveTiagoGoal PosGoal;  
    PosGoal.X = posX;
    PosGoal.Y = posY;
    PosGoal.yaw = yaw;

    //Send the goal to the server
    moveClient.sendGoal(PosGoal);

    //Feedback management
    /*
    int fb = 0;
    boost::shared_ptr<const tiago_iaslab_simulation::MoveTiagoActionFeedback> feedbackMsg;
    while(fb!= 3 && fb!= 4){
        feedbackMsg = ros::topic::waitForMessage<tiago_iaslab_simulation::MoveTiagoActionFeedback>("/moveTiagoServer/feedback");
        fb = feedbackMsg->feedback.feedbackID;
        switch(fb){
            case 1 :   
                ROS_INFO("Info from server: ROBOT IS MOVING ");
                break;
            case 2 :
                ROS_INFO("Info from server: ROBOT HAS REACHED FINAL POSITION");
                ROS_INFO("Info from server: COMPUTING OBSTACLES COORDINATES");
                break;
            case 3 :
                ROS_INFO("Info from server: TASK COMPLETED");
                break;
            case 4 :
                ROS_INFO("Info from server: CANNOT REACH POSITION");
                break;       
        }
    }
    */
    //Wait for the result for 120 second
    bool finished = moveClient.waitForResult(ros::Duration(120.0));
   
    if(finished){
        //Get the result 
        ROS_INFO("REACHED POSITION");

        //OBJECTS DETECTION  
        detectionClient.waitForServer();
        tiago_iaslab_simulation::DetectionGoal detGoal;
        detGoal.go = 1;
        detectionClient.sendGoal(detGoal);
        
        bool finishedDetection = detectionClient.waitForResult(ros::Duration(120.0));
   
        if(finishedDetection){
            //Get the result, a poses vector of the detected objects
            tiago_iaslab_simulation::DetectionResultConstPtr result_ = detectionClient.getResult();

            /*
            ros::NodeHandle n;
            ros::Publisher pub = n.advertise<tiago_iaslab_simulation::posesObjs>("posesMessage",1000);
            ros::Rate loop_rate(10);
            while (ros::ok())
            {
                   tiago_iaslab_simulation::posesObjs poses;
                   poses.detectedObjs = result_->detectedObjs;
                   pub.publish(poses);
                   ros::spinOnce();
                   loop_rate.sleep();
            } 
            */

            
            //PICK ACTION   
            pickPlaceClient.waitForServer();
            tiago_iaslab_simulation::PickPlaceGoal pickPlaceGoal;
            pickPlaceGoal.detectedObjs = result_->detectedObjs;
            pickPlaceGoal.type = obj_id;
            pickPlaceClient.sendGoal(pickPlaceGoal);

            bool finishedPick = pickPlaceClient.waitForResult(ros::Duration(120.0));
            if(finishedDetection){
                ROS_INFO("SUCCESFUL OBJECT PICK");
            }

        } else{
            ROS_INFO("Detection did not finish before the time out.");
        }




        
    } else{
        ROS_INFO("Movement did not finish before the time out.");
    }





    return 0;

}