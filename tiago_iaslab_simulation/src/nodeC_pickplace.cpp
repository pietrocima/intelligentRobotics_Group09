#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <shape_msgs/Mesh.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <control_msgs/PointHeadAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <moveit_msgs/CollisionObject.h>

#include <tiago_iaslab_simulation/MoveTiagoAction.h>
#include <tiago_iaslab_simulation/posesObjs.h>


class feedbackUtil{
    public:
        void CallbackFB(const tiago_iaslab_simulation::posesObjsConstPtr& msg);
        std::vector<geometry_msgs::Pose> poses;    
};
void feedbackUtil::CallbackFB(const tiago_iaslab_simulation::posesObjsConstPtr& msg){
        poses = msg->detectedObjs; 
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
      co.meshes.resize(1);
      co.mesh_poses.resize(1);
      co.meshes[0] = t_mesh;  
      co.mesh_poses[0].position.x = poses_vec.at(0).position.x;   
      co.mesh_poses[0].position.y = poses_vec.at(0).position.y;   
      co.mesh_poses[0].position.z = (poses_vec.at(0).position.z);   //pose is wrt the apriltag, this mean that the Z mus be lowered of half of the height of teh obj.
      co.mesh_poses[0].orientation = poses_vec.at(0).orientation;
      
      collision_objects[0].header.frame_id = referenceFrame;
      collision_objects[0].id = "target";
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
      table.primitives[0].dimensions[0] = 0.914;
      table.primitives[0].dimensions[1] = 0.914;
      table.primitives[0].dimensions[2] = 0.04;

      /* Define the pose of the table. */
      table.primitive_poses.resize(1);
      table.primitive_poses[0].position.x = 7.80;
      table.primitive_poses[0].position.y = -2.95;
      table.primitive_poses[0].position.z = 0.76; 

      collision_objects.push_back(table);


    //---------------------------------------------------------------------------
      moveit_msgs::CollisionObject check;

      check.id = "check";
      check.header.frame_id = referenceFrame;

      // Define the primitive and its dimensions. 
      check.primitives.resize(1);
      check.primitives[0].type = collision_objects[1].primitives[0].BOX;
      check.primitives[0].dimensions.resize(3);
      check.primitives[0].dimensions[0] = 0.05;
      check.primitives[0].dimensions[1] = 0.05;
      check.primitives[0].dimensions[2] = 0.05;

      /* Define the pose of the table. */
      check.primitive_poses.resize(1);
      check.primitive_poses[0].position = poses_vec.at(0).position;

      tf2::Quaternion orientation, rot; 
  
      orientation.setW(poses_vec.at(0).orientation.w); 
      orientation.setX(poses_vec.at(0).orientation.x); 
      orientation.setY(poses_vec.at(0).orientation.y); 
      orientation.setZ(poses_vec.at(0).orientation.z); 
  
      rot.setEulerZYX(0, M_PI / 2, M_PI / 2); 

      check.primitive_poses[0].orientation = tf2::toMsg(rot*orientation);


      collision_objects.push_back(check);
  }

	if (step == 1){
		
	  //table for placing
	  
		collision_objects[1].id = "table2";
		collision_objects[1].header.frame_id = "torso_fixed_link";

		// Define the primitive and its dimensions. 
		collision_objects[1].primitives.resize(1);
		collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
		collision_objects[1].primitives[0].dimensions.resize(3);
		collision_objects[1].primitives[0].dimensions[0] = 0.4;
		collision_objects[1].primitives[0].dimensions[1] = 0.2;
		collision_objects[1].primitives[0].dimensions[2] = 0.4;

		//Define the pose of the table. 
		collision_objects[1].primitive_poses.resize(1);
		
		//---------------------------------------------------------------------------------------pose to be defined as above
		collision_objects[1].primitive_poses[0] = poses_vec.at(1);		
		//------------------------------------------------------------------------------------------------------------------
		
		
		collision_objects.resize(2);
	}
   
  planning_scene_interface.applyCollisionObjects(collision_objects);
  
  return collision_objects;
}


void moveArm(moveit::planning_interface::MoveGroupInterface& group){

    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.28;
    target_pose1.position.y = 0.00;
    target_pose1.position.z = 0.80;
    group.setPoseTarget(target_pose1);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if(success){
      group.move();
    }
}





int main(int argc, char* argv[]){
    
    ros::init(argc, argv, "nodeC_pickplace");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    

   // ros::Subscriber sub = nh.subscribe("/moveTiagoServer/feedback", 10 ,&feedbackUtil::CallbackFB, &FB);
   // ros::Rate loop_rate(50);
    
    boost::shared_ptr<const tiago_iaslab_simulation::posesObjs> posesMsg = ros::topic::waitForMessage<tiago_iaslab_simulation::posesObjs>("/posesMessage");

    std::vector<geometry_msgs::Pose> poses_vec = posesMsg->detectedObjs;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group("arm_torso");
              
    ROS_INFO("move group Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("move group Pose Reference frame: %s", group.getPoseReferenceFrame().c_str());
            
                   

    group.setPlanningTime(45.0);

    std::vector<moveit_msgs::CollisionObject> objects_pick = addCollisionObjects(group, planning_scene_interface, poses_vec, 2, 0);


           

    // Wait a bit for ROS things to initialize
    ros::WallDuration(1.0).sleep();

    //pick(group, poses_vec, objType);
    moveArm(group);



    /*
    while(ros::ok()){
        if
        ros::spinOnce();
        loop_rate.sleep();
    }   
    
    if(FB.fb == 3){
        ROS_INFO("ROBOT IN FRONT OF TABLE \n");
        
        
    
    }
    */

}