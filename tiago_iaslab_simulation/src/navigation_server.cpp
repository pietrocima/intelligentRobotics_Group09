#include <ros/ros.h>
#include <math.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>
#include <tiago_iaslab_simulation/MoveTiagoAction.h>
#include <actionlib/client/terminal_state.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>



std::vector<double> obstaclePosition(tf::TransformListener* listen){

            //output vector that contain sequence of detected obj's (x,y) coord  
            std::vector<double> obsPos; 

            //Extract a read of the laserScan.
            boost::shared_ptr<const sensor_msgs::LaserScan> laserMsg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan");

            ros::Time timePCl = ros::Time::now();

            //Transform LaserScan into PointCloud
            laser_geometry::LaserProjection projector_;
            sensor_msgs::PointCloud cloud_laserFrame;
            projector_.projectLaser(*laserMsg,cloud_laserFrame);
            
            //Transform the point from the laser frame to the map frame 
            sensor_msgs::PointCloud cloud_mapFrame;
            
            listen->waitForTransform("base_laser_link","map",cloud_laserFrame.header.stamp,ros::Duration(5.0));
            listen->transformPointCloud("map",cloud_laserFrame,cloud_mapFrame);
                    
            //Compute the obstacle positions in the map frame.
            //The first and last 22 measurements of the laser have to be discarded since they do not return useful measurements to find obstacles.
        
            int startCoord = 25;                                     //We start at 25 since we consider a window of 3 points
            int endCoord = cloud_laserFrame.points.size()-22;
       
            std::vector<int> boolObs;                               //Binary vector: 1 = possible obsatcle 0 = no obstacle (3 points are aligned)
            
            float precision = 0.055;

            //Computing the difference between angular coefficent of two couple of consecutive points to state if they are in a line or not. 
            
            for(int i = startCoord; i < endCoord ; i ++ ){ 
               
                float X_diff1 = cloud_laserFrame.points.at(i-2).x - cloud_laserFrame.points.at(i-1).x;
                float Y_diff1 = cloud_laserFrame.points.at(i-2).y - cloud_laserFrame.points.at(i-1).y;

                float X_diff2 = cloud_laserFrame.points.at(i-1).x - cloud_laserFrame.points.at(i).x;
                float Y_diff2 = cloud_laserFrame.points.at(i-1).y - cloud_laserFrame.points.at(i).y;

                float angular_coeff_difference = Y_diff1/X_diff1 - Y_diff2/X_diff2;

                if (fabs(angular_coeff_difference) > precision){
                    boolObs.push_back(1);
                }else{
                    boolObs.push_back(0);
                }
            }

            //Check the length of sequences of 1 in boolObs. If the sequence is longer than obstacleWidth compute the coordinates 
            //of the obstacle's center. 

            int obstacleWidth = 10;     //Dimension that works for detecting the cylinders.
            
            for(int i =0; i < boolObs.size(); i++){

                if( (i < boolObs.size()) && boolObs.at(i) == 1){
                    int startObs = i;
                    int endObs;

                    while( i < boolObs.size() && boolObs.at(i) == 1 ){  
                        endObs = i;
                        i++;
                    }
                    
                    if (endObs - startObs > obstacleWidth){
                        float X_start_obs = cloud_mapFrame.points.at(startObs+25).x;    
                        float Y_start_obs = cloud_mapFrame.points.at(startObs+25).y;    
                        float X_end_obs = cloud_mapFrame.points.at(endObs+21).x;
                        float Y_end_obs = cloud_mapFrame.points.at(endObs+21).y;

                        float mid_x_obs = (X_start_obs + X_end_obs)/2;
                        float mid_y_obs = (Y_start_obs + Y_end_obs)/2;
                        
                        obsPos.push_back(mid_x_obs);
                        obsPos.push_back(mid_y_obs);
                    }
                }  
            }
        
        return obsPos;
}



class MoveTiagoAction{

protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<tiago_iaslab_simulation::MoveTiagoAction> as_;
    std::string action_name_;
    tiago_iaslab_simulation::MoveTiagoFeedback feedback_;
    tiago_iaslab_simulation::MoveTiagoResult result_;
public:
    MoveTiagoAction(std::string name) :
     as_(nh_,name,boost::bind(&MoveTiagoAction::executeCB,this, _1), false),
      action_name_(name){
        as_.start();
    }
    
    ~MoveTiagoAction(void){}

    void executeCB(const tiago_iaslab_simulation::MoveTiagoGoalConstPtr &goal){

        result_.sequence.clear();

        //Define the listener for transforming the point from a frame to another.
        tf::TransformListener listener; 
        
        tf2::Quaternion orientation;
        orientation.setRPY(0,0,goal->yaw);  //insert roll, pitch and yaw from the input.

        //create the client to move the base of the robot.
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_in("move_base", true);
        ac_in.waitForServer();

        //Set the final pose to reach 
        move_base_msgs::MoveBaseGoal goalMove;
        goalMove.target_pose.header.frame_id = "map";
        goalMove.target_pose.header.stamp = ros::Time::now();

        //Define the coordinate X and Y of the final position
        goalMove.target_pose.pose.position.x = goal->X;
        goalMove.target_pose.pose.position.y = goal->Y;
        //Define the orientation of the final position
        goalMove.target_pose.pose.orientation.w = orientation.getW();        
        goalMove.target_pose.pose.orientation.x = orientation.getX();
        goalMove.target_pose.pose.orientation.y = orientation.getY();
        goalMove.target_pose.pose.orientation.z = orientation.getZ();
        
        

        ROS_INFO("Sending goal to move: X:%f  Y:%f", goal->X, goal->Y);
        ac_in.sendGoal(goalMove);

        //Wait for the server to decide if the position is reachable. If it is, set the feedback to:"Robot is moving"
        sleep(2);       
        if(ac_in.getState() == actionlib::SimpleClientGoalState::ACTIVE){
            feedback_.feedbackID= 1;        //1 = Robot is moving 
            as_.publishFeedback(feedback_);
        }

        ROS_INFO("Waiting for result");
        ac_in.waitForResult();

        
        if(ac_in.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("The base has moved to X: %f, Y: %f",goal->X, goal->Y );
           
            feedback_.feedbackID= 2;        //2 = Robot has reached final position and is computing obstacles cooridnates 
            as_.publishFeedback(feedback_); 
            
			//OBTSACLE DETECTION
            result_.sequence = obstaclePosition(&listener);

            feedback_.feedbackID= 3;        //3 = task completed
            as_.publishFeedback(feedback_);
            
            ROS_INFO("Task Completed \n");
            as_.setSucceeded(result_);
        }else{
            feedback_.feedbackID = 4;       //4 = cannot reach position
            as_.publishFeedback(feedback_);
            ROS_INFO("Task Aborted \n");
            as_.setAborted();
        }      
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "moveTiagoServer");
    
    MoveTiagoAction tiago("moveTiagoServer");
    ros::spin();

    return 0;
}



