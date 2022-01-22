#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tiago_iaslab_simulation/PickPlaceAction.h>

#include <gazebo_ros_link_attacher/Attach.h>
#include <gazebo_ros_link_attacher/AttachRequest.h>
#include <gazebo_ros_link_attacher/AttachResponse.h>

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
 
    goal.trajectory.points[0].positions.resize(2);
    goal.trajectory.points[0].positions[0] = 0.03;
    goal.trajectory.points[0].positions[1] = 0.03;
     
    goal.trajectory.points[0].time_from_start = ros::Duration(0.5);
    

    moveGripper.waitForServer();
    
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    moveGripper.sendGoal(goal);

    moveGripper.waitForResult();
	sleep(1);

}

void openGripper(){
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> moveGripper("/gripper_controller/follow_joint_trajectory", true);

    control_msgs::FollowJointTrajectoryGoal goal;

    goal.trajectory.joint_names.push_back("gripper_left_finger_joint");
    goal.trajectory.joint_names.push_back("gripper_right_finger_joint");

    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions.resize(2);
    goal.trajectory.points[0].positions[0] = 0.04;
    goal.trajectory.points[0].positions[1] = 0.04;
     
    goal.trajectory.points[0].time_from_start = ros::Duration(0.5);
    

    moveGripper.waitForServer();
    
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    moveGripper.sendGoal(goal);
;
    moveGripper.waitForResult();
	sleep(1);
}

bool moveGivenJoints(std::vector<std::string> joint_names, std::map<std::string, double> joint_values, moveit::planning_interface::MoveGroupInterface& group, moveit::planning_interface::MoveGroupInterface::Plan my_plan){

	//Take in input a target pose in the joint space for the arm and move it. If it does not find a plan return false and the action is aborted.
	bool success;
	group.setStartStateToCurrentState();
	for (int k = 0; k < joint_names.size(); ++k){
		if (joint_values.count(joint_names.at(k)) > 0){
			group.setJointValueTarget(joint_names[k], joint_values[joint_names[k]]);
		}
	}

	success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	if (!success){
		ROS_ERROR("PLAN NOT FOUND");
		return false;
	}
	group.move();
	sleep(1);
	
	return true;
}

bool moveGivenPose(geometry_msgs::PoseStamped pose, moveit::planning_interface::MoveGroupInterface &group, moveit::planning_interface::MoveGroupInterface::Plan my_plan){
	
	//Take in input a target pose in the cartesian space for the arm and move it. If it does not find a plan return false and the action is aborted.
	bool success;
	group.setPoseTarget(pose);
	group.setStartStateToCurrentState();

	success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	if (!success){
		ROS_ERROR("PLAN NOT FOUND");
		return false;
	}
	group.move();
	sleep(1);

	return true;
}

void attachObjectGazebo(std::string modelName1,std::string linkName1,std::string modelName2,std::string linkName2){
      ros::NodeHandle n;
      ros::ServiceClient client = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");

      gazebo_ros_link_attacher::Attach srv;
      srv.request.model_name_1 = modelName1;
      srv.request.link_name_1 = linkName1;
      srv.request.model_name_2 = modelName2;   
      srv.request.link_name_2 = linkName2;
      if(client.call(srv)){
        if(srv.response.ok){
          ROS_INFO("TARGET ATTACHED");
        }else{
          ROS_INFO("TARGET NOT ATTACHED");
        }
      }else{
        ROS_ERROR("FAILED TO CALL DETACH SERVER");
      }
}

void detachObjectGazebo(std::string modelName1,std::string linkName1,std::string modelName2,std::string linkName2){
      ros::NodeHandle n;
      ros::ServiceClient client = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");

      gazebo_ros_link_attacher::Attach srv;
      srv.request.model_name_1 = modelName1;
      srv.request.link_name_1 = linkName1;
      srv.request.model_name_2 = modelName2;   
      srv.request.link_name_2 = linkName2;
      if(client.call(srv)){
        if(srv.response.ok){
          ROS_INFO("TARGET DETACHED");
        }else{
          ROS_INFO("TARGET NOT DETACHED");
        }
      }else{
        ROS_ERROR("FAILED TO CALL DETACH SERVER");
      }
}

void addCollisionObjects(moveit::planning_interface::MoveGroupInterface& move_group,moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, std::vector<geometry_msgs::Pose> poses_vec, int type, int step){
	
	std::string referenceFrame = move_group.getPoseReferenceFrame();
	// Create vector of collision objects
	std::vector<moveit_msgs::CollisionObject> collision_objects;

	//PICK ACTION HAS TO BE PERFORMED
	if (step == 0){
		collision_objects.resize(poses_vec.size());

		//define target object to pick
		shape_msgs::Mesh t_mesh;
		shapes::ShapeMsg t_mesh_msg;
		shapes::Mesh *t;
		float rescaleFactor = 0.5;
		Eigen::Vector3d rescaleTergets(rescaleFactor, rescaleFactor, rescaleFactor);

		float targetHeight = 0;
		switch (type){
			case 1:
				t = shapes::createMeshFromResource("package://tiago_iaslab_simulation/meshes/hexagon.dae", rescaleTergets);
				targetHeight = 0.20 * rescaleFactor;
				break;
			case 2:
				t = shapes::createMeshFromResource("package://tiago_iaslab_simulation/meshes/triangle_centered.stl", rescaleTergets);
				break;
			case 3:
				targetHeight = 0.05;
				collision_objects[0].id = "target";
				collision_objects[0].header.frame_id = move_group.getPlanningFrame();
				// Define the primitive and its dimensions.
				collision_objects[0].primitives.resize(1);
				collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
				collision_objects[0].primitives[0].dimensions.resize(3);
				collision_objects[0].primitives[0].dimensions[0] = targetHeight;
				collision_objects[0].primitives[0].dimensions[1] = targetHeight;
				collision_objects[0].primitives[0].dimensions[2] = targetHeight;

				// Define the cube's pose.
				collision_objects[0].primitive_poses.resize(1);
				collision_objects[0].primitive_poses[0].position.x = poses_vec.at(0).position.x;
				collision_objects[0].primitive_poses[0].position.y = poses_vec.at(0).position.y;
				collision_objects[0].primitive_poses[0].position.z = poses_vec.at(0).position.z - (targetHeight / 2);

				collision_objects[0].primitive_poses[0].orientation = poses_vec.at(0).orientation;

				break;
		}

		if (type == 2 || type == 1){
			shapes::constructMsgFromShape(t, t_mesh_msg);
			t_mesh = boost::get<shape_msgs::Mesh>(t_mesh_msg);
			ROS_INFO("Target mesh loaded");

			collision_objects[0].header.frame_id = referenceFrame;
			collision_objects[0].id = "target";
			collision_objects[0].meshes.resize(1);
			collision_objects[0].mesh_poses.resize(1);
			collision_objects[0].meshes[0] = t_mesh;

			//The detected pose is the Apriltag tag pose, this mean that the Z of the object must be lowered of half of the height of the obj to be placed correctly.
			collision_objects[0].mesh_poses[0].position.x = poses_vec.at(0).position.x;
			collision_objects[0].mesh_poses[0].position.y = poses_vec.at(0).position.y;
			collision_objects[0].mesh_poses[0].position.z = (poses_vec.at(0).position.z) - (targetHeight / 2); 
			collision_objects[0].mesh_poses[0].orientation = poses_vec.at(0).orientation;
		}

		// Define mesh model for obsatcle collision object.
		// Rescale the obstacles such that are a little bigger than the true ones.
		rescaleFactor = 1.3;
		Eigen::Vector3d rescaleObstacles(rescaleFactor, rescaleFactor, rescaleFactor);
		shapes::Mesh *m = shapes::createMeshFromResource("package://tiago_iaslab_simulation/meshes/hexagon.dae", rescaleObstacles);
		ROS_INFO("Hexagon mesh loaded");
		float hexagonHeight = 0.20; 

		shape_msgs::Mesh mesh;
		shapes::ShapeMsg mesh_msg;
		shapes::constructMsgFromShape(m, mesh_msg);
		mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

		for (int i = 1; i < poses_vec.size(); i++){
			// Define a collision object ROS message.
			collision_objects[i].header.frame_id = referenceFrame;

			// The id of the object is used to identify it.
			collision_objects[i].id = "obstacle" + std::to_string(i);
			collision_objects[i].meshes.resize(1);
			collision_objects[i].mesh_poses.resize(1);
			collision_objects[i].meshes[0] = mesh;
			collision_objects[i].mesh_poses[0].position.x = poses_vec.at(i).position.x;							
			collision_objects[i].mesh_poses[0].position.y = poses_vec.at(i).position.y;							
			collision_objects[i].mesh_poses[0].position.z = (poses_vec.at(i).position.z - (hexagonHeight / 2)); 
			collision_objects[i].mesh_poses[0].orientation = poses_vec.at(i).orientation;
		}
		
		// Table for picking
		moveit_msgs::CollisionObject table;
		table.id = "table1";
		table.header.frame_id = "map";

		// Define the primitive and its dimensions.
		table.primitives.resize(1);
		table.primitives[0].type = table.primitives[0].BOX;
		table.primitives[0].dimensions.resize(3);
		table.primitives[0].dimensions[0] = 0.97;
		table.primitives[0].dimensions[1] = 0.97;
		table.primitives[0].dimensions[2] = 0.045;

		// Define the pose of the table. 
		table.primitive_poses.resize(1);
		table.primitive_poses[0].position.x = 7.77;		
		table.primitive_poses[0].position.y = -2.95;	
		table.primitive_poses[0].position.z = 0.76;

		collision_objects.push_back(table);
	}

 	//PLACE ACTION HAS TO BE PERFORMED
	if (step == 1){
		//Add the place cylinder
		collision_objects.resize(1);

		collision_objects[0].header.frame_id = "map";
		collision_objects[0].primitives.resize(1);
		collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].CYLINDER;
		collision_objects[0].primitives[0].dimensions.resize(2);
		collision_objects[0].primitives[0].dimensions[0] = 0.72; // bigger then real ones
		collision_objects[0].primitives[0].dimensions[1] = 0.29;
		collision_objects[0].primitive_poses.resize(1);

		float posX, posY = -0.35, posZ = collision_objects[0].primitives[0].dimensions[0] / 2;
		switch (type){
			case 1:
				collision_objects[0].id = "placeCylinderBlue";
				posX = 12.58;
				break;
			case 2:
				collision_objects[0].id = "placeCylinderGreen";
				posX = 11.58;
				break;
			case 3:
				collision_objects[0].id = "placeCylinderRed";
				posX = 10.58;
				break;
		}
		collision_objects[0].primitive_poses[0].position.x = posX;
		collision_objects[0].primitive_poses[0].position.y = posY;
		collision_objects[0].primitive_poses[0].position.z = posZ;
	}

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

bool moveArmPick(moveit::planning_interface::MoveGroupInterface& group, moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, std::vector<geometry_msgs::Pose> poses_vec, int obj_id){
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success;
	std::vector<std::string> torso_arm_joint_names = group.getJoints();
	std::string modelNameObj,linkNameObj;                                   //Used to attach the target object to tiago's hand in Gazebo
    std::string modelNameTiago = "tiago", linkNameTiago = "arm_7_link";

	geometry_msgs::PoseStamped prePickPose, pickPose, intermediatePose;
	tf2::Quaternion orientationPrePick, rotation, targetOrientation, orientationInterpose;


	//DEFINE THE PREPICK AND PICK POSE FOR THE DIFFERENT OBJECTS
	switch (obj_id){
		case 1:
			modelNameObj = "Hexagon";
			linkNameObj = "Hexagon_link";

			//PREPICK POSE
			rotation.setRPY(M_PI_2, 0, 0);

			targetOrientation.setW(poses_vec.at(0).orientation.w);
			targetOrientation.setX(poses_vec.at(0).orientation.x);
			targetOrientation.setY(poses_vec.at(0).orientation.y);
			targetOrientation.setZ(poses_vec.at(0).orientation.z);

			orientationPrePick = targetOrientation * rotation;

			prePickPose.header.frame_id = group.getPlanningFrame();
			prePickPose.pose.position.x = poses_vec.at(0).position.x - 0.23; 
			prePickPose.pose.position.y = poses_vec.at(0).position.y - 0.035;
			prePickPose.pose.position.z = planning_scene_interface.getObjects().at("target").mesh_poses.at(0).position.z + 0.15;
			prePickPose.pose.orientation = tf2::toMsg(orientationPrePick);

			//PICK POSE
			pickPose.header.frame_id = group.getPlanningFrame();
			pickPose.pose.position.x = prePickPose.pose.position.x; 
			pickPose.pose.position.y = prePickPose.pose.position.y;
			pickPose.pose.position.z = prePickPose.pose.position.z - 0.15; 

			pickPose.pose.orientation = prePickPose.pose.orientation;


			break;
		case 2:
			modelNameObj = "Triangle";
			linkNameObj = "Triangle_link";
			
			//PREPICK POSE
			rotation.setRPY(-M_PI, M_PI_2, 0);

			targetOrientation.setW(poses_vec.at(0).orientation.w);
			targetOrientation.setX(poses_vec.at(0).orientation.x);
			targetOrientation.setY(poses_vec.at(0).orientation.y);
			targetOrientation.setZ(poses_vec.at(0).orientation.z);

			orientationPrePick = targetOrientation * rotation;

			prePickPose.header.frame_id = group.getPlanningFrame();
			prePickPose.pose.position.x = poses_vec.at(0).position.x - 0.10;
			prePickPose.pose.position.y = poses_vec.at(0).position.y + 0.07;
			prePickPose.pose.position.z = poses_vec.at(0).position.z + 0.22;

			prePickPose.pose.orientation = tf2::toMsg(orientationPrePick);

			//PICK POSE
			pickPose.header.frame_id = group.getPlanningFrame();
			pickPose.pose.position.x = prePickPose.pose.position.x;
			pickPose.pose.position.y = prePickPose.pose.position.y;
			pickPose.pose.position.z = prePickPose.pose.position.z - 0.05;

			pickPose.pose.orientation = prePickPose.pose.orientation;

			//INTERMDIATE POSE
			intermediatePose.header.frame_id = group.getPlanningFrame();
			intermediatePose.pose.position.x = 0.51;
			intermediatePose.pose.position.y = 0.22;
			intermediatePose.pose.position.z = 1.0976;

			intermediatePose.pose.orientation.x = 0.0694;
			intermediatePose.pose.orientation.y =-0.0011;
			intermediatePose.pose.orientation.z = 0.7182;
			intermediatePose.pose.orientation.w = 0.0380;


			break;
		case 3:
			modelNameObj = "cube";
			linkNameObj = "cube_link";

			//PREPICK POSE
			rotation.setRPY(M_PI_2, 0, 0);

			targetOrientation.setW(poses_vec.at(0).orientation.w);
			targetOrientation.setX(poses_vec.at(0).orientation.x);
			targetOrientation.setY(poses_vec.at(0).orientation.y);
			targetOrientation.setZ(poses_vec.at(0).orientation.z);

			orientationPrePick = targetOrientation * rotation;

			prePickPose.header.frame_id = group.getPlanningFrame();
			prePickPose.pose.position.x = poses_vec.at(0).position.x - 0.20;
			prePickPose.pose.position.y = poses_vec.at(0).position.y + 0.01; 
			prePickPose.pose.position.z = poses_vec.at(0).position.z + 0.10;

			prePickPose.pose.orientation = tf2::toMsg(orientationPrePick);

			//PICK POSE
			pickPose.header.frame_id = group.getPlanningFrame();
			pickPose.pose.position.x = prePickPose.pose.position.x;
			pickPose.pose.position.y = prePickPose.pose.position.y;
			pickPose.pose.position.z = prePickPose.pose.position.z - 0.10;

			pickPose.pose.orientation = prePickPose.pose.orientation;

			break;
	}


	
	//INITIAL CONFIGURATION
	std::map<std::string, double> initial_position;

	initial_position["arm_1_joint"] = 0.51;
	initial_position["arm_2_joint"] = -0.75;
	initial_position["arm_3_joint"] = -1.69;
	initial_position["arm_4_joint"] = 2.21;
	initial_position["arm_5_joint"] = -0.71;
	initial_position["arm_6_joint"] = 1.04;
	initial_position["arm_7_joint"] = -0.83;

	ROS_INFO("GOING TO INITIAL CONFIGURATION");
	if (!(moveGivenJoints(torso_arm_joint_names, initial_position, group, my_plan)))
		return false;

	//PRE-PICK POSE
	ROS_INFO("GOING PRE PICK POSITION");
	if (!(moveGivenPose(prePickPose, group, my_plan)))
		return false;

	//PICK POSE
	ROS_INFO("GOING PICK POSITION");
	if (!(moveGivenPose(pickPose, group, my_plan)))
		return false;

	//PICK
	std::vector<std::string> noCollisionLinks{"gripper_right_finger_link", "gripper_left_finger_link"};
	group.attachObject("target", group.getEndEffectorLink(), noCollisionLinks);
	std::vector<std::string> removeID{"target"};
	planning_scene_interface.removeCollisionObjects(removeID);

	attachObjectGazebo(modelNameTiago, linkNameTiago, modelNameObj, linkNameObj);

	closeGripper();

	//PRE PICK POSE
	ROS_INFO("GOING PREPICK POSITION AFTER PICK");
	if (!(moveGivenPose(prePickPose, group, my_plan)))
		return false;


	//INTERMEDIATE POSE
	ROS_INFO("GOING INTERMEDIATE POSITION");
	switch (obj_id)	{
		case 2:
			if (!(moveGivenPose(intermediatePose, group, my_plan)))
				return false;
			break;
		default:
			if (!(moveGivenJoints(torso_arm_joint_names, initial_position, group, my_plan)))
				return false;
			break;
	}

	//SAFE POSE
	std::map<std::string, double> safe_position;

	safe_position["arm_1_joint"] = 0.20;  
	safe_position["arm_2_joint"] = -1.50;
	safe_position["arm_3_joint"] = -0.19; 
	safe_position["arm_4_joint"] = 1.82;
	safe_position["arm_5_joint"] = -1.46;
	safe_position["arm_6_joint"] = 1.37;
	safe_position["arm_7_joint"] = 0.00;

	ROS_INFO("GOING TO SAFE POSITION");
	if (!(moveGivenJoints(torso_arm_joint_names, safe_position, group, my_plan)))
		return false;
	

	//Remove all collision object from the world;
	planning_scene_interface.removeCollisionObjects(planning_scene_interface.getKnownObjectNames());
	return true;
}

bool moveArmPlace(moveit::planning_interface::MoveGroupInterface& group, moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, int obj_id){
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	bool success;
    std::string modelNameObj,linkNameObj;                                   //Used to attach the target object to tiago's hand in Gazebo
    std::string modelNameTiago = "tiago", linkNameTiago = "arm_7_link";

	std::map<std::string, double> pre_place_configuration, place_configuration;
	std::vector<std::string> torso_arm_joint_names = group.getJoints();
	
	//Define place arm configuration for the different objects 
	switch (obj_id){
		case 1:
			modelNameObj = "Hexagon";
     		linkNameObj = "Hexagon_link";
			
			//Define place arm configuration for the hexagon
			place_configuration["arm_1_joint"] = 0.40;	
			place_configuration["arm_2_joint"] = -0.16;	
			place_configuration["arm_3_joint"] = -1.52;	
			place_configuration["arm_4_joint"] = 1.71;	
			place_configuration["arm_5_joint"] = -0.21;	
			place_configuration["arm_6_joint"] = 0.00;
			place_configuration["arm_7_joint"] = 0.00; 
			break;
		case 2:
			modelNameObj = "Triangle";
      		linkNameObj = "Triangle_link";

			//Define place arm configuration for the triangle
			place_configuration["arm_1_joint"] = 0.56;		
			place_configuration["arm_2_joint"] = 0.16;
			place_configuration["arm_3_joint"] = -1.70;
			place_configuration["arm_4_joint"] = 1.44;		
			place_configuration["arm_5_joint"] = 0.27;
			place_configuration["arm_6_joint"] = -0.94;
			place_configuration["arm_7_joint"] = -0.33;
			break;
		case 3:
			modelNameObj = "cube";
      		linkNameObj = "cube_link";

			//Define place arm configuration for the cube
			place_configuration["arm_1_joint"] = 0.40;	
			place_configuration["arm_2_joint"] = -0.16; 
			place_configuration["arm_3_joint"] = -1.52; 
			place_configuration["arm_4_joint"] = 1.71;	
			place_configuration["arm_5_joint"] = -0.21; 
			place_configuration["arm_6_joint"] = 0.00;	
			place_configuration["arm_7_joint"] = 0.00;	
			break;
	}

	//PRE-PLACE POSE

	pre_place_configuration["arm_1_joint"] = 0.40;				
	pre_place_configuration["arm_2_joint"] = -0.16; 
	pre_place_configuration["arm_3_joint"] = -1.82;
	pre_place_configuration["arm_4_joint"] = 1.71;					
	pre_place_configuration["arm_5_joint"] = -0.21;
	pre_place_configuration["arm_6_joint"] = -0.22; 
	pre_place_configuration["arm_7_joint"] = 0.00; 

	ROS_INFO("GOING TO PRE PLACE POSITION");
	if (!(moveGivenJoints(torso_arm_joint_names, pre_place_configuration, group, my_plan)))
		return false;
	

	//PLACE POSE
	ROS_INFO("GOING TO PLACE POSITION");
	if (!(moveGivenJoints(torso_arm_joint_names, place_configuration, group, my_plan)))
		return false;

	//PLACE
	openGripper();
	group.detachObject("target");
	detachObjectGazebo(modelNameTiago, linkNameTiago, modelNameObj, linkNameObj);

	//PREPALCE
	ROS_INFO("GOING TO PRE-PLACE POSITION");
	if (!(moveGivenJoints(torso_arm_joint_names, pre_place_configuration, group, my_plan)))
		return false;


	//SAFE POSE
	std::map<std::string, double> safe_position;

	safe_position["arm_1_joint"] = 0.20;  
	safe_position["arm_2_joint"] = -1.50; 
	safe_position["arm_3_joint"] = -0.19; 
	safe_position["arm_4_joint"] = 1.82;
	safe_position["arm_5_joint"] = -1.46;
	safe_position["arm_6_joint"] = 1.37;
	safe_position["arm_7_joint"] = 0.00;

	ROS_INFO("GOING TO SAFE POSITION");
	if (!(moveGivenJoints(torso_arm_joint_names, safe_position, group, my_plan)))
		return false;


	planning_scene_interface.removeCollisionObjects(planning_scene_interface.getKnownObjectNames());
	return true;
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
			
			result_.actionCompleted = false;

            //Goal is a vec<geometry_msgs::Pose> that contain the poses of the detected objects, in position 0 there is teh pose of teh target object
            std::vector<geometry_msgs::Pose> poses_vec = goal->detectedObjs;
            int objType = goal->ObjType;
            int actionType = goal->actionType; 

            ROS_INFO("GOAL RECEIVED");

            moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
            moveit::planning_interface::MoveGroupInterface group("arm");
                   
            group.setPlanningTime(45.0);

            addCollisionObjects(group, planning_scene_interface, poses_vec, objType, actionType);
    
            if(actionType == 0){ //Perform pick;
				if( moveArmPick(group, planning_scene_interface, poses_vec, objType)){
					result_.actionCompleted = true;
					as_.setSucceeded(result_);
				}else{
					ROS_ERROR("Task Aborted \n");
					as_.setAborted();
				}
            }else{				//Perform place
				if( moveArmPlace(group, planning_scene_interface, objType)){
					result_.actionCompleted = true;
					as_.setSucceeded(result_);
				}else{
					ROS_ERROR("Task Aborted \n");
					as_.setAborted();
				}
            }            
        }
};



int main(int argc, char* argv[]){
    
    ros::init(argc, argv, "nodeC_PickPlaceServer");

    PickPlaceAction pickPlace("nodeC_PickPlaceServer");

    ros::MultiThreadedSpinner spinner(4);

    spinner.spin();

    return 0;
}