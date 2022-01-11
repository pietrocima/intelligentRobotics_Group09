#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tiago_iaslab_simulation/MoveTiagoAction.h>
#include <tiago_iaslab_simulation/MoveTiagoActionFeedback.h>
#include <actionlib/client/terminal_state.h>
#include <stdlib.h>


int main(int argc, char* argv[]){
    ros::init(argc,argv,"moveTiagoClient");

    actionlib::SimpleActionClient<tiago_iaslab_simulation::MoveTiagoAction> ac("moveTiagoServer", true);
    
    ROS_INFO("Waiting for server to start.");

    ac.waitForServer(); 
    ROS_INFO("Action server started, sending goal.");
    
    //Retrieving goal from user's input
    float posX = std::stof(argv[1]);
    float posY = std::stof(argv[2]);
    float yaw = std::stof(argv[3]);

   
    //Define the goal specifying the X and Y coordinates and Yaw angle.
    tiago_iaslab_simulation::MoveTiagoGoal PosGoal;  
    PosGoal.X = posX;
    PosGoal.Y = posY;
    PosGoal.yaw = yaw;

    //Send the goal to the server
    ac.sendGoal(PosGoal);
    
    //Check and print the feedback from the server
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
    
    
    //Wait for the result for 120 second
    bool finished = ac.waitForResult(ros::Duration(120.0));
   
    if(finished){
        //Get the result and print it out
        tiago_iaslab_simulation::MoveTiagoResultConstPtr result_ = ac.getResult();
        for(int i = 0; i < result_->sequence.size(); i+=2){
            ROS_INFO("Obstacle at coordinates:  X: %f  Y: %f",result_->sequence.at(i), result_->sequence.at(i+1));
        }
    } else{
        ROS_INFO("Action did not finish before the time out.");
    }

    return 0;
}