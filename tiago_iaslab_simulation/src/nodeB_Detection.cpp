#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <tiago_iaslab_simulation/DetectionAction.h>

#include <control_msgs/PointHeadAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <tiago_iaslab_simulation/MoveTiagoActionFeedback.h>
#include <tiago_iaslab_simulation/MoveTiagoAction.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <tf/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

/*
void moveHeadDown(){
    actionlib::SimpleActionClient<control_msgs::PointHeadAction> pointHeadClient("/head_controller/point_head_action", true);

    geometry_msgs::PointStamped lookPoint;

    lookPoint.header.frame_id = "/xtion_rgb_optical_frame";
    lookPoint.point.x = 0;
    lookPoint.point.y = -0.50;
    lookPoint.point.z = 1.0;

    control_msgs::PointHeadGoal goal;
    goal.pointing_frame = "/xtion_rgb_optical_frame";
    goal.pointing_axis.x = 0.0;
    goal.pointing_axis.y = 0.0;
    goal.pointing_axis.z = 1.0;
    goal.target = lookPoint;

    pointHeadClient.waitForServer();

    pointHeadClient.sendGoal(goal);
    ROS_INFO("Waiting for result");
    pointHeadClient.waitForResult();

}
*/

void moveHeadDown(){
    
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> moveHead("/head_controller/follow_joint_trajectory", true);

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names.push_back("head_1_joint");
    goal.trajectory.joint_names.push_back("head_2_joint");

    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions.resize(2);
    goal.trajectory.points[0].positions[0] = 0.0;
    goal.trajectory.points[0].positions[1] = -0.40;
     
    goal.trajectory.points[0].time_from_start = ros::Duration(3);
    
    moveHead.waitForServer();

    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    moveHead.sendGoal(goal);

    ROS_INFO("MOVING HEAD DOWN");
    moveHead.waitForResult();
}
void moveHeadUp(){
    
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> moveHead("/head_controller/follow_joint_trajectory", true);

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names.push_back("head_1_joint");
    goal.trajectory.joint_names.push_back("head_2_joint");

    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions.resize(2);
    goal.trajectory.points[0].positions[0] = 0.0;
    goal.trajectory.points[0].positions[1] = 0.0;
     
    goal.trajectory.points[0].time_from_start = ros::Duration(3);
    
    moveHead.waitForServer();

    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    moveHead.sendGoal(goal);

    ROS_INFO("MOVING HEAD UP");
    moveHead.waitForResult();
}



class feedbackUtil{
    public:
        void CallbackFB(const tiago_iaslab_simulation::MoveTiagoActionFeedback::ConstPtr& msg);
        int fb;
};
void feedbackUtil::CallbackFB(const tiago_iaslab_simulation::MoveTiagoActionFeedback::ConstPtr& msg){
    fb = msg->feedback.feedbackID;
}



std::vector<geometry_msgs::Pose> apriltagDetection(tf::TransformListener* listen){

        //Define the destination frame of the transform from the camera frame
        const std::string destinationFrame = "base_footprint"; 

        //si assume che per ogni rilevazione si identifichi un solo oggetto da prelevare (con id <= 3) e che sarà sempre posizionato in cima al vettore di pose rilevate 
        boost::shared_ptr< const apriltag_ros::AprilTagDetectionArray> msg = ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("/tag_detections");
        int numDetections = msg->detections.size();
        listen->waitForTransform(destinationFrame,msg->header.frame_id,msg->header.stamp,ros::Duration(5.0));
        
        
        std::vector<geometry_msgs::Pose> detectObjs;
        //Fill the detectObjs vector, in 1 position there is the objcet to pick up.
        for(int i = 0; i < numDetections; i++){
            ROS_INFO("Detected ID: %i", msg->detections.at(i).id.at(0));
            
            geometry_msgs::PoseStamped obejctPoseStampDestFrame;
            geometry_msgs::PoseStamped obejctPoseStampCameraFrame;
                obejctPoseStampCameraFrame.header = msg->header;
                obejctPoseStampCameraFrame.pose = msg->detections.at(i).pose.pose.pose;

            listen->transformPose(destinationFrame, obejctPoseStampCameraFrame, obejctPoseStampDestFrame);



           // ROS_INFO("POSITION wrt CAMERA FRAME:      (%f,%f,%f)", obejctPoseStampCameraFrame.pose.position.x, obejctPoseStampCameraFrame.pose.position.y, obejctPoseStampCameraFrame.pose.position.z);
            ROS_INFO("POSITION wrt TORSO FIXED FRAME: (%f,%f,%f)", obejctPoseStampDestFrame.pose.position.x, obejctPoseStampDestFrame.pose.position.y, obejctPoseStampDestFrame.pose.position.z);
            if(detectObjs.empty()){
                detectObjs.push_back(obejctPoseStampDestFrame.pose);
            }else if( msg->detections.at(i).id.at(0) <= 3 ){
                detectObjs.insert(detectObjs.begin(),obejctPoseStampDestFrame.pose);
            }else{
                detectObjs.push_back(obejctPoseStampDestFrame.pose);
            }
        }
   
    return detectObjs;
}


class DetectionAction{

    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<tiago_iaslab_simulation::DetectionAction> as_;
        std::string action_name_;
        tiago_iaslab_simulation::DetectionFeedback feedback_;
        tiago_iaslab_simulation::DetectionResult result_;
    public:
        DetectionAction(std::string name) :
        as_(nh_,name,boost::bind(&DetectionAction::executeCB,this, _1), false),
        action_name_(name){
            as_.start();
        }
        
        ~DetectionAction(void){}

        void executeCB(const tiago_iaslab_simulation::DetectionGoalConstPtr &goal){
            tf::TransformListener listener;
            std::vector<geometry_msgs::Pose> detectedObjs;
           

            ROS_INFO("ROBOT IN FRONT OF TABLE \n");
            moveHeadDown();
            
            ROS_INFO("APRILTAG DETECTION");    
            //contain the poses of the tag, not of the object associated to the tag. 
            detectedObjs =  apriltagDetection(&listener); 
            
        
            //moveHeadUp();
            result_.detectedObjs = detectedObjs;   
            
            ROS_INFO("Detection Completed \n");
            as_.setSucceeded(result_);
        }   
};



int main(int argc, char* argv[]){
    ros::init(argc, argv, "nodeB_detectionServer");
    
    
    DetectionAction pickPlace("nodeB_detectionServer");

    ros::spin();




    /*
    tf::TransformListener listener;
    std::vector<geometry_msgs::PoseStamped> detectedObjs;
    feedbackUtil FB;
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/moveTiagoServer/feedback", 10 ,&feedbackUtil::CallbackFB, &FB);
    ros::Rate loop_rate(50);
    while(ros::ok()){
        if(FB.fb == 3) //movement completed 
           break;
        ros::spinOnce();
        loop_rate.sleep();
    }   
    
    if(FB.fb == 3){
        ROS_INFO("ROBOT IN FRONT OF TABLE \n");
        moveHeadDown();
        
        ROS_INFO("APRILTAG DETECTION");    
        detectedObjs =  apriltagDetection(&listener);    
    }

    for(int i = 0 ; i < detectedObjs.size(); i++){
       ROS_INFO("POSITION wrt TORSO FIXED FRAME: (%f,%f,%f)", detectedObjs.at(i).pose.position.x,detectedObjs.at(i).pose.position.y,detectedObjs.at(i).pose.position.z);
    }

    moveHeadUp();
    */
    return 0;
}