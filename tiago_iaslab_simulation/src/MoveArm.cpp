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


//POSE ARM_TOOL_LINK FOR VERTICAL APPROACH TO AN OBJECT.
//(0.60, -0.08, 1.03)    QUATERNION(x 0.58 ,y 0.44 ,z -0.45 ,w 0.50)
//
//
//



void closeGripper(){

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> moveGripper("/gripper_controller/follow_joint_trajectory", true);

    control_msgs::FollowJointTrajectoryGoal goal;

    goal.trajectory.joint_names.push_back("gripper_left_finger_joint");
    goal.trajectory.joint_names.push_back("gripper_right_finger_joint");

    goal.trajectory.points.resize(1);
    //first
    goal.trajectory.points[0].positions.resize(2);
    goal.trajectory.points[0].positions[0] = 0.02;
    goal.trajectory.points[0].positions[1] = 0.02;
     
    goal.trajectory.points[0].time_from_start = ros::Duration(0.5);
    

    moveGripper.waitForServer();
    
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    moveGripper.sendGoal(goal);

    ROS_INFO("Waiting for result");
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

    ROS_INFO("Waiting for result");
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


void addTable(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface){
    std::vector<moveit_msgs::CollisionObject> collision_objects;

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
    table.primitive_poses[0].position.x = 1;
    table.primitive_poses[0].position.y = 0;
    table.primitive_poses[0].position.z = 0.76; 

    collision_objects.push_back(table);
    //define target object to pick 
      shape_msgs::Mesh t_mesh;
      shapes::ShapeMsg t_mesh_msg; 
      shapes::Mesh* t;
      float rescaleFactor = 0.5;
      Eigen::Vector3d rescale(rescaleFactor,rescaleFactor,rescaleFactor);

      float targetHeight;
      t = shapes::createMeshFromResource("package://tiago_iaslab_simulation/meshes/triangle_centered.stl",rescale); 
      // targetHeight = 0.12 * rescaleFactor;
     
      shapes::constructMsgFromShape(t, t_mesh_msg);
      t_mesh = boost::get<shape_msgs::Mesh>(t_mesh_msg);
      ROS_INFO("Target mesh loaded");
      
      moveit_msgs::CollisionObject co;  
      co.header.frame_id = "base_footprint";
      co.id = "target";
      co.meshes.resize(1);
      co.mesh_poses.resize(1);
      co.meshes[0] = t_mesh;  
      co.mesh_poses[0].position.x = 0.68;   
      co.mesh_poses[0].position.y = 0;   
      co.mesh_poses[0].position.z = 0.79;   //pose is wrt the apriltag, this mean that the Z mus be lowered of half of the height of teh obj.
    

      
      collision_objects.push_back(co);



    planning_scene_interface.applyCollisionObjects(collision_objects);


}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
  std::vector<moveit_msgs::Grasp> grasps;
  std::string referenceFrame = move_group.getPlanningFrame();
  grasps.resize(1);
  ROS_INFO("SETTING GRASP, PRE GRASP, POST GRASP ");
  // Setting grasp pose. With the pose of teh object to grasp
  grasps[0].grasp_pose.header.frame_id = referenceFrame; 
  //posotion
  grasps[0].grasp_pose.pose.position.x =0.68;
  grasps[0].grasp_pose.pose.position.y = 0;
  grasps[0].grasp_pose.pose.position.z = 0.79;
  //orientation
  grasps[0].grasp_pose.pose.orientation.w =0.048 ;
  grasps[0].grasp_pose.pose.orientation.x = 0.082;
  grasps[0].grasp_pose.pose.orientation.y = 0.56;
  grasps[0].grasp_pose.pose.orientation.z = -0.07;


  //PRE GRASP
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

int main(int argc, char* argv[]){
    
    ros::init(argc, argv, "move_arm");
    
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

    addTable(planning_scene_interface);
    
    moveit::planning_interface::MoveGroupInterface movegroup("arm");

    ROS_INFO("End effector link: %s", movegroup.getEndEffectorLink().c_str());

    //pick(movegroup);



    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    
    
    tf2::Quaternion orientation;
    //
   // orientation.setRPY(-1.57, 0, 1.57);
    orientation.setRPY(M_PI_2,0,M_PI_4);
    
    //orientation.setW(0.66);
    //orientation.setX(0.31);
    //orientation.setY(0.17);
    //orientation.setZ(0.64);
   
    
    
   // orientation.setRPY(-0.11,1.57,0.0037);

    //posa triangolo (0.75,0.042,0.75) Quat(w 0.048, z -0.07, y 0.56, x 0.82)
    geometry_msgs::PoseStamped target_pose1;
    target_pose1.header.frame_id="base_footprint";
    target_pose1.pose.orientation =tf2::toMsg(orientation);
    target_pose1.pose.position.x = 0.52;
    target_pose1.pose.position.y = -0.16;
    target_pose1.pose.position.z = 0.82;
    movegroup.setPoseTarget(target_pose1);


    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (movegroup.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if ( !success )
        throw std::runtime_error("No plan found");
    
    moveit::planning_interface::MoveItErrorCode e = movegroup.move();
    if (!bool(e))
      throw std::runtime_error("Error executing plan");


    closeGripper();
    movegroup.attachObject("target");
    std::vector<std::string> removeID;
    removeID.push_back("target");
    planning_scene_interface.removeCollisionObjects(removeID);





}

