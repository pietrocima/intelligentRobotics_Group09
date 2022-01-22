#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <tiago_iaslab_simulation/DetectionAction.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <tf/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>


void moveHeadDown(){

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> moveHead("/head_controller/follow_joint_trajectory", true);
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> moveTorso("/torso_controller/follow_joint_trajectory", true);
    
	//MOVING TORSO UP
	control_msgs::FollowJointTrajectoryGoal goalTorso;
	goalTorso.trajectory.joint_names.push_back("torso_lift_joint");
    goalTorso.trajectory.points.resize(1);
    goalTorso.trajectory.points[0].positions.resize(1);
    goalTorso.trajectory.points[0].positions[0] = 0.20;

    goalTorso.trajectory.points[0].time_from_start = ros::Duration(3);
    
    moveTorso.waitForServer();

    goalTorso.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    moveTorso.sendGoal(goalTorso);
    moveTorso.waitForResult();

	//MOVING HEAD DOWN
    control_msgs::FollowJointTrajectoryGoal goalHead;
	goalHead.trajectory.joint_names.push_back("head_1_joint");
	goalHead.trajectory.joint_names.push_back("head_2_joint");

	goalHead.trajectory.points.resize(1);
	goalHead.trajectory.points[0].positions.resize(2);
	goalHead.trajectory.points[0].positions[0] = 0.0;
	goalHead.trajectory.points[0].positions[1] = -0.5;

	goalHead.trajectory.points[0].time_from_start = ros::Duration(3);

	moveHead.waitForServer();

	goalHead.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
	moveHead.sendGoal(goalHead);
	moveHead.waitForResult();
}

void moveHeadUp(){
	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> moveHead("/head_controller/follow_joint_trajectory", true);
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> moveTorso("/torso_controller/follow_joint_trajectory", true);
    
	//MOVING TORSO DOWN
    control_msgs::FollowJointTrajectoryGoal goalTorso;
    goalTorso.trajectory.joint_names.push_back("torso_lift_joint");
    goalTorso.trajectory.points.resize(1);
    goalTorso.trajectory.points[0].positions.resize(1);
    goalTorso.trajectory.points[0].positions[0] = 0.15;

    goalTorso.trajectory.points[0].time_from_start = ros::Duration(3);
    
    moveTorso.waitForServer();

    goalTorso.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    moveTorso.sendGoal(goalTorso);
    moveTorso.waitForResult();

	//MOVING HEAD UP
    control_msgs::FollowJointTrajectoryGoal goalHead;
	goalHead.trajectory.joint_names.push_back("head_1_joint");
	goalHead.trajectory.joint_names.push_back("head_2_joint");

	goalHead.trajectory.points.resize(1);
	goalHead.trajectory.points[0].positions.resize(2);
	goalHead.trajectory.points[0].positions[0] = 0.0;
	goalHead.trajectory.points[0].positions[1] = 0.0;

	goalHead.trajectory.points[0].time_from_start = ros::Duration(3);

	moveHead.waitForServer();

	goalHead.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
	moveHead.sendGoal(goalHead);

	
    moveHead.waitForResult();
}

//
std::vector<geometry_msgs::Pose> apriltagDetection(tf::TransformListener *listen)
{

	//Define the destination frame of the transform from the camera frame
	const std::string destinationFrame = "base_footprint";
	boost::shared_ptr<const apriltag_ros::AprilTagDetectionArray> msg;
	bool stay = true;
	while(stay){
		//we assume that for every scan, will be always find a target object (id <=3).
		msg = ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("/tag_detections");
		
		//check if at least one target object has been detected 
		for(int i = 0; i < msg->detections.size(); i++){
			if(msg->detections.at(i).id.at(0) <=3 )
				stay = false;
		}
	};
	
	int numDetections = msg->detections.size();
	listen->waitForTransform(destinationFrame, msg->header.frame_id, msg->header.stamp, ros::Duration(5.0));

	std::vector<geometry_msgs::Pose> detectObjs;
	//Fill the detectObjs vector, in position 0 there is the objcet to pick up.
	for (int i = 0; i < numDetections; i++)
	{
		ROS_INFO("Detected ID: %i", msg->detections.at(i).id.at(0));

		geometry_msgs::PoseStamped obejctPoseStampDestFrame;
		geometry_msgs::PoseStamped obejctPoseStampCameraFrame;
		obejctPoseStampCameraFrame.header = msg->header;
		obejctPoseStampCameraFrame.pose = msg->detections.at(i).pose.pose.pose;

		listen->transformPose(destinationFrame, obejctPoseStampCameraFrame, obejctPoseStampDestFrame);

		ROS_INFO("POSITION wrt TORSO FIXED FRAME: (%f,%f,%f)", obejctPoseStampDestFrame.pose.position.x, obejctPoseStampDestFrame.pose.position.y, obejctPoseStampDestFrame.pose.position.z);
		
		//Place target pose at the start of the vector
		if (detectObjs.empty()){
			detectObjs.push_back(obejctPoseStampDestFrame.pose);
		}
		else if (msg->detections.at(i).id.at(0) <= 3){
			detectObjs.insert(detectObjs.begin(), obejctPoseStampDestFrame.pose);
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
			result_.detectedObjs.clear();

			tf::TransformListener listener;
            std::vector<geometry_msgs::Pose> detectedObjs;
            
            ROS_INFO("ROBOT IN FRONT OF TABLE \n");
			ROS_INFO("MOVING HEAD DOWN");
			moveHeadDown();
            sleep(2);  
            
			ROS_INFO("APRILTAG DETECTION");    
            detectedObjs =  apriltagDetection(&listener);

			ROS_INFO("MOVING HEAD UP");
			moveHeadUp();
            result_.detectedObjs = detectedObjs;   
            
            ROS_INFO("Detection Completed \n");
            as_.setSucceeded(result_);
        }   
};



int main(int argc, char* argv[]){
    ros::init(argc, argv, "nodeB_detectionServer");
    
    
    DetectionAction pickPlace("nodeB_detectionServer");

    ros::spin();

    return 0;
}