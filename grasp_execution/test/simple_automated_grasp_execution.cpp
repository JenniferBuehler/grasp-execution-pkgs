#include <grasp_execution_msgs/GraspAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <arm_components_name_manager/ArmComponentsNameManager.h>
#include <convenience_ros_functions/ROSFunctions.h>
#include <convenience_ros_functions/RobotInfo.h>

#include "SimpleGraspGenerator.h"
#include <object_msgs/Object.h>
#include <object_msgs/ObjectInfo.h>
#include <object_msgs_tools/ObjectFunctions.h>
#include <grasp_execution_reach_test/MoveItPlanner.h>

#include <control_msgs/FollowJointTrajectoryAction.h>


using object_msgs_tools::ObjectFunctions;
using grasp_execution_reach_test::MoveItPlanner;
using convenience_ros_functions::RobotInfo;

/**
 * \param timeout_wait_object wait this amount of seconds maximum until cube is there.
 *      This is useful if it has been spawned at the same time the node has been launched.
 */
bool getObjectPose(const std::string& object_name, const std::string& REQUEST_OBJECTS_SERVICE, const float timeout_wait_object, geometry_msgs::PoseStamped& pose)
{
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<object_msgs::ObjectInfo>(REQUEST_OBJECTS_SERVICE);
	object_msgs::ObjectInfo srv;
	srv.request.name = object_name;
    srv.request.get_geometry=false;

    ros::Time startTime = ros::Time::now();
    float timeWaited = 0;
    bool success=false;
    while (timeWaited < timeout_wait_object)
    {
        if (client.call(srv) && srv.response.success)
        {
            // ROS_INFO("getObjectInfo result:");
            // std::cout<<srv.response<<std::endl;
            success = true;
            break;
        }
        ros::Time currTime = ros::Time::now();
        timeWaited = (currTime - startTime).toSec();
    }
    if (!success)
    {
        ROS_ERROR("Failed to call service %s, error code: %i",REQUEST_OBJECTS_SERVICE.c_str(),srv.response.error_code);
        return false;
    }
    return ObjectFunctions::getObjectPose(srv.response.object,pose);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grasp_action_client");

    ros::NodeHandle priv("~");
    ros::NodeHandle pub("");

    /////////// Read parameters  ///////////////////
    if (!priv.hasParam("object_name"))
    {
        ROS_ERROR("Object name required!");
        return 0;
    } 
    std::string OBJECT_NAME;
	priv.param<std::string>("object_name", OBJECT_NAME, OBJECT_NAME);


    std::string ROBOT_NAMESPACE;
    if (!priv.hasParam("robot_namespace"))
    {
        ROS_ERROR("Node requires private parameter 'robot_namespace'");
        return 1;
    }
	priv.param<std::string>("robot_namespace", ROBOT_NAMESPACE, ROBOT_NAMESPACE);

    arm_components_name_manager::ArmComponentsNameManager jointsManager(ROBOT_NAMESPACE, false);
    double maxWait=5;
    ROS_INFO("Waiting for joint info parameters to be loaded...");
    jointsManager.waitToLoadParameters(1,maxWait); 
    ROS_INFO("Parameters loaded.");

    std::vector<std::string> gripperJoints = jointsManager.getGripperJoints();
    for (int i=0; i<gripperJoints.size(); ++i) ROS_INFO_STREAM("Gripper "<<i<<": "<<gripperJoints[i]);
    // std::vector<std::string> armJoints = joints.getArmJoints();

    std::string arm_base_link = jointsManager.getArmLinks().front();
    std::string effector_link = jointsManager.getEffectorLink();
 
    bool GRASPING=true;
	priv.param<bool>("grasping_action", GRASPING, GRASPING);
    
    double OPEN_ANGLES=0.05;
	priv.param<double>("open_angles", OPEN_ANGLES, OPEN_ANGLES);
    double CLOSE_ANGLES=0.7;
	priv.param<double>("close_angles", CLOSE_ANGLES, CLOSE_ANGLES);


    double POSE_ABOVE;
	priv.param<double>("pose_above_object", POSE_ABOVE, POSE_ABOVE);
    double POSE_X;
	priv.param<double>("x_from_object", POSE_X, POSE_X);
    double POSE_Y;
	priv.param<double>("y_from_object", POSE_Y, POSE_Y);
    
    double EFF_POS_TOL;
	priv.param<double>("effector_pos_tolerance", EFF_POS_TOL, EFF_POS_TOL);
    double EFF_ORI_TOL;
	priv.param<double>("effector_ori_tolerance", EFF_ORI_TOL, EFF_ORI_TOL);
    double JOINT_ANGLE_TOL;
	priv.param<double>("joint_angle_tolerance", JOINT_ANGLE_TOL, JOINT_ANGLE_TOL);
   

	std::string REQUEST_OBJECTS_SERVICE="world/request_object";
	priv.param<std::string>("request_object_service", REQUEST_OBJECTS_SERVICE, REQUEST_OBJECTS_SERVICE);
    
    std::string JOINT_STATES_TOPIC = "/joint_states";
	priv.param<std::string>("joint_states_topic", JOINT_STATES_TOPIC, JOINT_STATES_TOPIC);

    std::string GRASP_ACTION_NAME = "/grasp_action";
	priv.param<std::string>("grasp_action_name", GRASP_ACTION_NAME, GRASP_ACTION_NAME);
    
    std::string JOINT_TRAJECTORY_ACTION_NAME = "/joint_trajectory_action";
	priv.param<std::string>("joint_trajectory_action_name", JOINT_TRAJECTORY_ACTION_NAME, JOINT_TRAJECTORY_ACTION_NAME);

	std::string MOVEIT_MOTION_PLAN_SERVICE="/plan_kinematic_path";
	priv.param<std::string>("moveit_motion_plan_service", MOVEIT_MOTION_PLAN_SERVICE, MOVEIT_MOTION_PLAN_SERVICE);

	std::string MOVEIT_STATE_VALIDITY_SERVICE="/check_state_validity";
	priv.param<std::string>("moveit_state_validity_service", MOVEIT_STATE_VALIDITY_SERVICE, MOVEIT_STATE_VALIDITY_SERVICE);

    double ARM_REACH_SPAN = 2;
	priv.param<double>("arm_reach_span", ARM_REACH_SPAN, ARM_REACH_SPAN);
    std::string PLANNING_GROUP="Arm";
	priv.param<std::string>("planning_group", PLANNING_GROUP, PLANNING_GROUP);
 
    /////////// Get object pose  ///////////////////
        
    geometry_msgs::PoseStamped obj_pose;
    float maxWaitObject=2; 
    if (!getObjectPose(OBJECT_NAME, REQUEST_OBJECTS_SERVICE, maxWaitObject, obj_pose))
    {
        ROS_ERROR("Could not get pose of object %s", OBJECT_NAME.c_str());
        return 0;
    }
    
    /////////// Generate grasp  ///////////////////

    std::string object_frame_id = OBJECT_NAME;
    
    manipulation_msgs::Grasp mgrasp;
    bool genGraspSuccess = grasp_execution::SimpleGraspGenerator::generateSimpleGraspFromTop(
        gripperJoints,
        arm_base_link,
        "TestGrasp",
        OBJECT_NAME,        
        obj_pose,
        object_frame_id,
        POSE_ABOVE, POSE_X, POSE_Y,
        OPEN_ANGLES, CLOSE_ANGLES,
        mgrasp);

    if (!genGraspSuccess)
    {
        ROS_ERROR("Could not generate grasp");
        return 0;
    }

    // ROS_INFO_STREAM("generated manipulation_msgs::Grasp: "<<std::endl<<mgrasp);

    grasp_execution_msgs::GraspGoal graspGoal;
    bool isGrasp = true;
    grasp_execution::SimpleGraspGenerator::generateSimpleGraspGoal(effector_link,
        mgrasp,0, isGrasp, graspGoal);

    grasp_execution::SimpleGraspGenerator::useCustomTolerances(EFF_POS_TOL,
        EFF_ORI_TOL, JOINT_ANGLE_TOL, graspGoal);
    
    ROS_INFO_STREAM("generated grasp_execution_msgs::Grasp: "<<std::endl<<graspGoal);



    /////////// Do trajectory planning to reach  ///////////////////
    
    MoveItPlanner trajectoryPlanner(pub, 
	        MOVEIT_MOTION_PLAN_SERVICE,
	        MOVEIT_STATE_VALIDITY_SERVICE);
   
    // build planning constraints:
    // XXX TODO parameterize!
    float plan_eff_pos_tol = EFF_POS_TOL;
    float plan_eff_ori_tol = EFF_ORI_TOL;
    int type = 0; // 0 = only position, 1 = pos and ori, 2 = only ori
    moveit_msgs::Constraints goal_constraints = trajectoryPlanner.getPoseConstraint(effector_link,
        mgrasp.grasp_pose, plan_eff_pos_tol, plan_eff_ori_tol, type); 
    

    // get the current arm base pose in the object frame. This is needed
    // to generate MoveIt! workspace. This can be in any frame, it can also
    // be the object frame since the robot is not moving.
    convenience_ros_functions::ROSFunctions::initSingleton();
    geometry_msgs::PoseStamped currBasePose;
    int transRet=convenience_ros_functions::ROSFunctions::Singleton()->getTransform(
            object_frame_id, arm_base_link,
            currBasePose.pose,
            ros::Time(0),2,true);
    if (transRet!=0) {
        ROS_ERROR("Could not get current effector tf transform in object frame.");
        return 0;
    }
    currBasePose.header.stamp=ros::Time::now();
    currBasePose.header.frame_id=object_frame_id;
    ROS_INFO_STREAM("Effector currBasePose pose: "<<currBasePose);
 
    // request joint trajectory
    RobotInfo robotInfo;
    sensor_msgs::JointState currArmJointState = robotInfo.getCurrentJointState(JOINT_STATES_TOPIC, pub);
    jointsManager.extractFromJointState(currArmJointState,0,currArmJointState);
    ROS_INFO_STREAM("Current arm joint state: "<<currArmJointState);

    moveit_msgs::RobotTrajectory robotTrajectory;
    moveit_msgs::MoveItErrorCodes moveitRet = trajectoryPlanner.requestTrajectory(
        currBasePose,
        ARM_REACH_SPAN,
        PLANNING_GROUP,
        goal_constraints,
        NULL,
        currArmJointState,
        robotTrajectory);

    if (moveitRet.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
        ROS_ERROR("Could not plan joint trajectory");
        return 0;
    }

    /////////// Execute joint trajectory  ///////////////////

    ROS_INFO("Now constructing joint trajectory goal");

    // send a goal to the action
    control_msgs::FollowJointTrajectoryGoal jtGoal;
    jtGoal.trajectory = robotTrajectory.joint_trajectory;

    // create the action client
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> jtac(JOINT_TRAJECTORY_ACTION_NAME, true);
    ROS_INFO_STREAM("Waiting for action server to start: "<< JOINT_TRAJECTORY_ACTION_NAME);
    // wait for the action server to start
    jtac.waitForServer();  // will wait for infinite time
    ROS_INFO("Joint trajectory action server has started. Now sending goal");
    jtac.sendGoal(jtGoal);

    //wait for the action to return
    bool finished_before_timeout = jtac.waitForResult(ros::Duration(15.0));
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = jtac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
    {
        ROS_INFO("Action did not finish before the time out.");
        return 0;
    }
    
    /////////// Send grasp execution action request  ///////////////////

    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<grasp_execution_msgs::GraspAction> gac(GRASP_ACTION_NAME, true);

    ROS_INFO_STREAM("Waiting for action server to start: "<< GRASP_ACTION_NAME);
    // wait for the action server to start
    gac.waitForServer(); //will wait for infinite time
    ROS_INFO("Action server started. Now sending goal");
    gac.sendGoal(graspGoal);

    //wait for the action to return
    finished_before_timeout = gac.waitForResult(ros::Duration(15.0));
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = gac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
    {
        ROS_INFO("Action did not finish before the time out.");
        return 0;
    }
    return 0;
}
