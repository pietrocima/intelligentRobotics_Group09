#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tiago_iaslab_simulation/PickPlaceAction.h>

#include <shape_msgs/Mesh.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>



void closeGripper(){

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> moveGripper("/gripper_controller/follow_joint_trajectory", true);

    control_msgs::FollowJointTrajectoryGoal goal;

    goal.trajectory.joint_names.push_back("gripper_left_finger_joint");
    goal.trajectory.joint_names.push_back("gripper_right_finger_joint");

    goal.trajectory.points.resize(1);
    //first
    goal.trajectory.points[0].positions.resize(2);
    goal.trajectory.points[0].positions[0] = 0.03;
    goal.trajectory.points[0].positions[1] = 0.03;
     
    goal.trajectory.points[0].time_from_start = ros::Duration(0.5);
    

    moveGripper.waitForServer();
    
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    moveGripper.sendGoal(goal);

   // ROS_INFO("Waiting for result");
    moveGripper.waitForResult();

}

void openGripper(){
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> moveGripper("/gripper_controller/follow_joint_trajectory", true);

    control_msgs::FollowJointTrajectoryGoal goal;

    goal.trajectory.joint_names.push_back("gripper_left_finger_joint");
    goal.trajectory.joint_names.push_back("gripper_right_finger_joint");

    goal.trajectory.points.resize(1);
    //first
    goal.trajectory.points[0].positions.resize(2);
    goal.trajectory.points[0].positions[0] = 0.04;
    goal.trajectory.points[0].positions[1] = 0.04;
     
    goal.trajectory.points[0].time_from_start = ros::Duration(0.5);
    

    moveGripper.waitForServer();
    
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    moveGripper.sendGoal(goal);

    //ROS_INFO("Waiting for result");
    moveGripper.waitForResult();
}


void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  /* Add both finger joints of tiago gripper. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "gripper_left_finger_joint";
  posture.joint_names[1] = "gripper_right_finger_joint";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = 0.04;
  posture.points[0].time_from_start = ros::Duration(0.5);
  
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{

  /* Add both finger joints of tiago gripper. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "gripper_left_finger_joint";
  posture.joint_names[1] = "gripper_right_finger_joint";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  posture.points[0].time_from_start = ros::Duration(0.5);

}

void pick(moveit::planning_interface::MoveGroupInterface& move_group,std::vector<geometry_msgs::Pose> poses_vec, int type)
{
  
  std::string referenceFrame = move_group.getPlanningFrame();
  //move arm to safe position
  
  std::map<std::string, double> safe_position;
  std::vector<std::string> torso_arm_joint_names = move_group.getJoints();
  move_group.setStartStateToCurrentState();
   
  
  safe_position["arm_1_joint"] = 0.07;
  safe_position["arm_2_joint"] = 0.01;
  safe_position["arm_3_joint"] = -3.14;
  safe_position["arm_4_joint"] = 1.55;
  safe_position["arm_5_joint"] = -0.20;
  safe_position["arm_6_joint"] = 0.01;
  safe_position["arm_7_joint"] = 0.00;

  ROS_INFO("GOING TO SAFE POSITION");
  
  for (int k = 0; k < torso_arm_joint_names.size(); ++k)
      if ( safe_position.count(torso_arm_joint_names.at(k)) > 0 )
       {
         move_group.setJointValueTarget(torso_arm_joint_names[k], safe_position[torso_arm_joint_names[k]]);
       }
	   
	   moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	   bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   
      if ( !success )
        throw std::runtime_error("No plan found");
   
      move_group.move();

  //-------------- Grasp has to be defined for each type, then it is fixed/constant (now it has random numbers)
  // Create a vector of grasps to be attempted, currently only creating single grasp.
  // This is essentially useful when using a grasp generator to generate and test multiple grasps.
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);
  ROS_INFO("SETTING GRASP, PRE GRASP, POST GRASP ");
  // Setting grasp pose. With the pose of teh object to grasp
  
  grasps[0].grasp_pose.header.frame_id = referenceFrame;  
  tf2::Quaternion orientation, rot; 
  
  //orientation.setW(poses_vec.at(0).orientation.w); 
  //orientation.setX(poses_vec.at(0).orientation.x); 
  //orientation.setY(poses_vec.at(0).orientation.y); 
  //orientation.setZ(poses_vec.at(0).orientation.z); 
  
  //rot.setEulerZYX(0, M_PI_2, M_PI_2); 
  //grasps[0].grasp_pose.pose.orientation = tf2::toMsg(rot*orientation); 
    
  orientation.setRPY(M_PI_2, 0, M_PI_4);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  
  grasps[0].grasp_pose.pose.position = poses_vec.at(0).position;
  

  // Setting pre-grasp approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].pre_grasp_approach.direction.header.frame_id = referenceFrame;
  /* Direction is set as positive x axis */
  grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].post_grasp_retreat.direction.header.frame_id = referenceFrame;
  /* Direction is set as positive z axis */
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;
  //------------------------------------------------------------------------------------------------------------------------------------------------------------


  // Setting posture of eef before grasp
  // +++++++++++++++++++++++++++++++++++
  openGripper(grasps[0].pre_grasp_posture);

  // Setting posture of eef during grasp
  // +++++++++++++++++++++++++++++++++++
  closedGripper(grasps[0].grasp_posture);
 

  // Set support surface as table1.
  move_group.setSupportSurfaceName("table1");
  // Call pick to pick up the object using the grasps given
  ROS_INFO("PICKING");
  move_group.pick("target", grasps);
  
  
  
  
}

void place(moveit::planning_interface::MoveGroupInterface& group, int type)
{
	  //--------------------Grasp poses same as for pick	
	  // Create a vector of placings to be attempted, currently only creating single place location.
	  std::vector<moveit_msgs::PlaceLocation> place_location;
	  place_location.resize(1);

	  // Setting place location pose
	  // +++++++++++++++++++++++++++
	  place_location[0].place_pose.header.frame_id = "torso_fixed_link";
	  tf2::Quaternion orientation;
	  orientation.setRPY(0, 0, M_PI / 2);
	  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

	  /* While placing it is the exact location of the center of the object. */
	  place_location[0].place_pose.pose.position.x = 0;
	  place_location[0].place_pose.pose.position.y = 0.5;
	  place_location[0].place_pose.pose.position.z = 0.5;

	  // Setting pre-place approach
	  // ++++++++++++++++++++++++++
	  /* Defined with respect to frame_id */
	  place_location[0].pre_place_approach.direction.header.frame_id = "torso_fixed_link";
	  /* Direction is set as negative z axis */
	  place_location[0].pre_place_approach.direction.vector.z = -1.0;
	  place_location[0].pre_place_approach.min_distance = 0.095;
	  place_location[0].pre_place_approach.desired_distance = 0.115;

	  // Setting post-grasp retreat
	  // ++++++++++++++++++++++++++
	  /* Defined with respect to frame_id */
	  place_location[0].post_place_retreat.direction.header.frame_id = "torso_fixed_link";
	  /* Direction is set as negative y axis */
	  place_location[0].post_place_retreat.direction.vector.y = -1.0;
	  place_location[0].post_place_retreat.min_distance = 0.1;
	  place_location[0].post_place_retreat.desired_distance = 0.25;
	  //------------------------------------------------------------------------------------------------------------------------------------------------------------


	  // Setting posture of eef after placing object
	  // +++++++++++++++++++++++++++++++++++++++++++
	  /* Similar to the pick case */
	  openGripper(place_location[0].post_place_posture);

	  // Set support surface as table2.
	  group.setSupportSurfaceName("table2");
	  // Call place to place the object using the place locations given.
	  group.place("target", place_location);
	  // END_SUB_TUTORIAL
}



std::vector<moveit_msgs::CollisionObject> addCollisionObjects(moveit::planning_interface::MoveGroupInterface& move_group,moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, std::vector<geometry_msgs::Pose> poses_vec, int type, int step)
{
  std::string referenceFrame = move_group.getPoseReferenceFrame();
  // Create vector of collision objects
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  
  
  if (step == 0){
      collision_objects.resize(poses_vec.size());  //contain the detcted obj
      ROS_INFO("STEP == 0");
    
      //define target object to pick 
      shape_msgs::Mesh t_mesh;
      shapes::ShapeMsg t_mesh_msg; 
      shapes::Mesh* t;
      float rescaleFactor = 0.5;
      Eigen::Vector3d rescale(rescaleFactor,rescaleFactor,rescaleFactor);

      float targetHeight;
      switch(type){
        case 1:
          t = shapes::createMeshFromResource("package://tiago_iaslab_simulation/meshes/hexagon.dae",rescale); 
          targetHeight = 0.20 * rescaleFactor;
          break;
        case 2:
          t = shapes::createMeshFromResource("package://tiago_iaslab_simulation/meshes/triangle_centered.stl",rescale); 
         // targetHeight = 0.12 * rescaleFactor;
          break;
        case 3:
          t = shapes::createMeshFromResource("package://tiago_iaslab_simulation/meshes/hexagon.dae"); //CUBO
          targetHeight = 0.05;
          break;
      }
     
      shapes::constructMsgFromShape(t, t_mesh_msg);
      t_mesh = boost::get<shape_msgs::Mesh>(t_mesh_msg);
      ROS_INFO("Target mesh loaded");
      
      moveit_msgs::CollisionObject co;  
      co.header.frame_id = referenceFrame;
      co.id="target";
      co.meshes.resize(1);
      co.mesh_poses.resize(1);
      co.meshes[0] = t_mesh;  
      co.mesh_poses[0].position.x = poses_vec.at(0).position.x;   
      co.mesh_poses[0].position.y = poses_vec.at(0).position.y;   
      co.mesh_poses[0].position.z = (poses_vec.at(0).position.z);   //pose is wrt the apriltag, this mean that the Z mus be lowered of half of the height of teh obj.
      co.mesh_poses[0].orientation = poses_vec.at(0).orientation;
      
      collision_objects[0].header.frame_id = referenceFrame;
      collision_objects[0] = co; 
    
      // Define mesh model for obsatcle collision object.
        
      shapes::Mesh* m = shapes::createMeshFromResource("package://tiago_iaslab_simulation/meshes/hexagon.dae"); 
      ROS_INFO("Hexagon mesh loaded");
      float hexagonHeight = 0.20; // height 20cm.
     
      shape_msgs::Mesh mesh;
      shapes::ShapeMsg mesh_msg;  
      shapes::constructMsgFromShape(m, mesh_msg);
      mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
      
      for (int i = 1; i < poses_vec.size(); i++){
        
        // Define a collision object ROS message.
        moveit_msgs::CollisionObject co;
        co.header.frame_id = referenceFrame;

        // The id of the object is used to identify it.
        co.id = "obstacle" + std::to_string(i);
        co.meshes.resize(1);
        co.mesh_poses.resize(1);
        co.meshes[0] = mesh; 
        co.mesh_poses[0].position.x = poses_vec.at(i).position.x;  //pose is wrt to the apriltag, teh Z must be lowered of half the height of teh object.
        co.mesh_poses[0].position.y = poses_vec.at(i).position.y;  //pose is wrt to the apriltag, teh Z must be lowered of half the height of teh object.
        co.mesh_poses[0].position.z = (poses_vec.at(i).position.z-(hexagonHeight/2));  //pose is wrt to the apriltag, teh Z must be lowered of half the height of teh object.
        co.mesh_poses[0].orientation = poses_vec.at(i).orientation;



        collision_objects[i] = co;
        ROS_INFO( "ADD OBSTACLE %i", i );
        ROS_INFO( "Obs pose Z: %f", (poses_vec.at(i).position.z-(hexagonHeight/2)));
    }
    //table for picking
      moveit_msgs::CollisionObject table;

      table.id = "table1";
      table.header.frame_id = "map";

      // Define the primitive and its dimensions. 
      table.primitives.resize(1);
      table.primitives[0].type = table.primitives[0].BOX;
      table.primitives[0].dimensions.resize(3);
      table.primitives[0].dimensions[0] = 0.917;
      table.primitives[0].dimensions[1] = 0.917;
      table.primitives[0].dimensions[2] = 0.04;

      /* Define the pose of the table. */
      table.primitive_poses.resize(1);
      table.primitive_poses[0].position.x = 7.80;
      table.primitive_poses[0].position.y = -2.95;
      table.primitive_poses[0].position.z = 0.76; 

      collision_objects.push_back(table);
  }

	if (step == 1){
		
	  //table for placing
	  
		collision_objects[1].id = "table2";
		collision_objects[1].header.frame_id = "map";

		// Define the primitive and its dimensions. 
		collision_objects[1].primitives.resize(1);
		collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].CYLINDER;
		collision_objects[1].primitives[0].dimensions.resize(2);
		collision_objects[1].primitives[0].dimensions[0] = 0.69;
		collision_objects[1].primitives[0].dimensions[1] = 0.21;
	

		//Define the pose of the table. 
		collision_objects[1].primitive_poses.resize(1);
		
		
		switch(type){
		  case 1:
			collision_objects[1].primitive_poses[0].position.x = 12.50;
			collision_objects[1].primitive_poses[0].position.y = -0.30;
			collision_objects[1].primitive_poses[0].position.z = collision_objects[1].primitives[0].dimensions[2]/2; 
		  
		  case 2:
			collision_objects[1].primitive_poses[0].position.x = 11.50;
			collision_objects[1].primitive_poses[0].position.y = -0.30;
			collision_objects[1].primitive_poses[0].position.z = collision_objects[1].primitives[0].dimensions[2]/2; 	
			
		  case 3:
			collision_objects[1].primitive_poses[0].position.x = 10.50;
			collision_objects[1].primitive_poses[0].position.y = -0.30;
			collision_objects[1].primitive_poses[0].position.z = collision_objects[1].primitives[0].dimensions[2]/2; 	  

		}	
		
		collision_objects.resize(2);
	}
   
  planning_scene_interface.applyCollisionObjects(collision_objects);
  
  return collision_objects;
}


void moveArm(moveit::planning_interface::MoveGroupInterface& group, moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, std::vector<geometry_msgs::Pose> poses_vec){
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    
    geometry_msgs::PoseStamped safePose;
    tf2::Quaternion orientationSafepose; 

    safePose.header.frame_id = group.getPlanningFrame();
    safePose.pose.position.x = 0.31;
    safePose.pose.position.y = 0.17;
    safePose.pose.position.z = 0.63;

    //orientationSafepose.setRPY(0,0,0);
    orientationSafepose.setX(-0.0641);
    orientationSafepose.setY(-0.7217);
    orientationSafepose.setZ(0.0056);
    orientationSafepose.setW(0.6891);
 
    safePose.pose.orientation = tf2::toMsg(orientationSafepose);
    group.setPoseTarget(safePose);
    ROS_INFO("GOING SAFE POSITION");
    group.setStartStateToCurrentState();
    
    bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if ( !success ){
      ROS_INFO("PLAN NOT FOUND");
      throw std::runtime_error("No plan found");
    }
    group.move();
    sleep(2);
    
    //PRE PICK POSE 
    geometry_msgs::PoseStamped prePickPose;
    tf2::Quaternion orientationPrePick;
    prePickPose.header.frame_id = group.getPlanningFrame();
    prePickPose.pose.position.x = poses_vec.at(0).position.x - 0.30;
    prePickPose.pose.position.y = poses_vec.at(0).position.y;
    prePickPose.pose.position.z = 0.83;
    
    orientationPrePick.setX(0.7085);
    orientationPrePick.setY(0.0058);
    orientationPrePick.setZ(0.0065);
    orientationPrePick.setW(0.7056);
    prePickPose.pose.orientation = tf2::toMsg(orientationPrePick);

    group.setPoseTarget(prePickPose);
    group.setStartStateToCurrentState();
    ROS_INFO("GOING PRE PICK POSITION");
    
    success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if ( !success )
      throw std::runtime_error("No plan found");
  
    group.move();

    sleep(2);

    //GOAL POSE TARGET
    geometry_msgs::PoseStamped goalPose;
    tf2::Quaternion orientation;

    goalPose.header.frame_id = group.getPlanningFrame();
    goalPose.pose.position.x = poses_vec.at(0).position.x - 0.15;
    goalPose.pose.position.y = poses_vec.at(0).position.y;
    goalPose.pose.position.z = 0.83;

    //orientation.setRPY(-0.011,1.57,0.037);
    orientation.setX(0.7085);
    orientation.setY(0.0058);
    orientation.setZ(0.0065);
    orientation.setW(0.7056); 
    goalPose.pose.orientation = tf2::toMsg(orientation);

    group.setPoseTarget(goalPose);  
    group.setStartStateToCurrentState();
    ROS_INFO("GOING PICK POSITION");
    
    success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if ( !success )
      throw std::runtime_error("No plan found");
  
    group.move();
    sleep(2);

    //PICK 
    group.attachObject("target");
    std::vector<std::string>removeID;
    removeID.push_back("target");
    planning_scene_interface.removeCollisionObjects(removeID);
    ROS_INFO_STREAM("removed obj"<<removeID.at(0));
    
    

    //closeGripper();
    sleep(2);




//-------------------------------------------------------------------------------

    ros::ServiceClient client = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("attach_object");
    gazebo_ros_link_attacher::Attach srv;
    srv.request.model_name_1 = 'target'
    srv.request.link_name_1 = 'link'
    srv.request.model_name_2 = 'arm_7_link'
    srv.request.link_name_2 = 'link'

    if (client.call(srv))
    {
      if (srv.response.ok)
      ROS_INFO("attachment successful between target and arm_link_7");
    }
    else
    {
    ROS_ERROR("Failed to call service Attach");
    return 1;
     }

//---------------------------------------------------------------------------------




    group.setPoseTarget(prePickPose);
    group.setStartStateToCurrentState();
    ROS_INFO("GOING PREPICK POSITION AFTER PICK");
    
    success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if ( !success )
      throw std::runtime_error("No plan found");
  
    group.move();
    sleep(2);
    
    group.setPoseTarget(safePose);
    group.setStartStateToCurrentState();
    ROS_INFO("GOING SAFE POSITION AFTER PICK");
    
    success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if ( !success )
      throw std::runtime_error("No plan found");
  
    group.move();

}


class PickPlaceAction{

    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<tiago_iaslab_simulation::PickPlaceAction> as_;
        std::string action_name_;
        tiago_iaslab_simulation::PickPlaceFeedback feedback_;
        tiago_iaslab_simulation::PickPlaceResult result_;
    public:
        PickPlaceAction(std::string name) :
        as_(nh_,name,boost::bind(&PickPlaceAction::executeCB,this, _1), false),
        action_name_(name){
            as_.start();
        }
        
        ~PickPlaceAction(void){}

        void executeCB(const tiago_iaslab_simulation::PickPlaceGoalConstPtr &goal){
            //goal is a vec<geometry_msgs::PoseStamped> that contain teh pose of teh detected objects
            std::vector<geometry_msgs::Pose> poses_vec = goal->detectedObjs;
            int objType = goal->type;

            ROS_INFO("GOAL RECEIVED");

            ros::WallDuration(1.0).sleep();
            moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
            moveit::planning_interface::MoveGroupInterface group("arm");
              
            ROS_INFO("move group Reference frame: %s", group.getPlanningFrame().c_str());
            ROS_INFO("move group Pose Reference frame: %s", group.getPoseReferenceFrame().c_str());
            ROS_INFO_STREAM("End effector link: "<< group.getEndEffectorLink());
                   

            group.setPlanningTime(45.0);

            std::vector<moveit_msgs::CollisionObject> objects_pick = addCollisionObjects(group, planning_scene_interface, poses_vec, objType, 0);

            // Wait a bit for ROS things to initialize
            ros::WallDuration(1.0).sleep();

            //pick(group, poses_vec, objType);
            moveArm(group,planning_scene_interface, poses_vec);


            

            
            ros::WallDuration(1.0).sleep();
  
        }
};





int main(int argc, char* argv[]){
    
    ros::init(argc, argv, "nodeC_PickPlaceServer");

    PickPlaceAction pickPlace("nodeC_PickPlaceServer");

    ros::MultiThreadedSpinner spinner(4);

    spinner.spin();
    //ros::spin();

    return 0;
}