#include <grasp_execution_msgs/GraspAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <arm_components_name_manager/ArmComponentsNameManager.h>
#include <convenience_ros_functions/ROSFunctions.h>
#include "SimpleGraspGenerator.h"
#include <object_msgs/Object.h>
#include <object_msgs/ObjectInfo.h>
#include <object_msgs_tools/ObjectFunctions.h>

#include <grasp_execution_reach_test/MoveItPlanner.h>

using object_msgs_tools::ObjectFunctions;
using grasp_execution_reach_test::MoveItPlanner;

/**
 * \param timeout_wait_object wait this amount of seconds maximum until cube is there.
 *      This is useful if it has been spawned at the same time the node has been launched.
 */
bool getObjectPose(const std::string& object_name, const std::string& REQUEST_OBJECTS_TOPIC, const float timeout_wait_object, geometry_msgs::PoseStamped& pose)
{
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<object_msgs::ObjectInfo>(REQUEST_OBJECTS_TOPIC);
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
        ROS_ERROR("Failed to call service %s, error code: %i",REQUEST_OBJECTS_TOPIC.c_str(),srv.response.error_code);
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

    std::string arm_base_frame = jointsManager.getArmLinks().front();
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
   

	std::string REQUEST_OBJECTS_TOPIC="world/request_object";
	priv.param<std::string>("request_object_service_topic", REQUEST_OBJECTS_TOPIC, REQUEST_OBJECTS_TOPIC);

    std::string GRASP_ACTION_TOPIC = "/grasp_action";
	priv.param<std::string>("grasp_action_topic", GRASP_ACTION_TOPIC, GRASP_ACTION_TOPIC);

	std::string MOVEIT_MOTION_PLAN_SERVICE="/plan_kinematic_path";
	priv.param<std::string>("moveit_motion_plan_service", MOVEIT_MOTION_PLAN_SERVICE, MOVEIT_MOTION_PLAN_SERVICE);

	std::string MOVEIT_STATE_VALIDITY_SERVICE="/check_state_validity";
	priv.param<std::string>("moveit_state_validity_service", MOVEIT_STATE_VALIDITY_SERVICE, MOVEIT_STATE_VALIDITY_SERVICE);
 
    /////////// Get object pose  ///////////////////
        
    geometry_msgs::PoseStamped obj_pose;
    float maxWaitObject=2; 
    if (!getObjectPose(OBJECT_NAME, REQUEST_OBJECTS_TOPIC, maxWaitObject, obj_pose))
    {
        ROS_ERROR("Could not get pose of object %s", OBJECT_NAME.c_str());
        return 0;
    }
    
    /////////// Generate grasp  ///////////////////

    std::string object_frame_id = OBJECT_NAME;
    
    manipulation_msgs::Grasp mgrasp;
    bool genGraspSuccess = grasp_execution::SimpleGraspGenerator::generateSimpleGraspFromTop(
        gripperJoints,
        arm_base_frame,
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

/*    moveit_msgs::MoveItErrorCodes moveitRet = trajectoryPlanner.requestTrajectory(
        const geometry_msgs::PoseStamped& robot_pose,
        const geometry_msgs::PoseStamped& target_pose,
        float armReachSpan, const std::string& planning_group,
        const moveit_msgs::Constraints& goal_constraints,
        const moveit_msgs::Constraints * pathConstraints,
        const sensor_msgs::JointState& startState,
        moveit_msgs::RobotTrajectory& resultTraj);
*/

    /////////// Execute joint trajectory  ///////////////////


    
    /////////// Send grasp execution action request  ///////////////////

    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<grasp_execution_msgs::GraspAction> ac(GRASP_ACTION_TOPIC, true);

    ROS_INFO("Waiting for action server to start: %s", GRASP_ACTION_TOPIC.c_str());
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time
    ROS_INFO("Action server started.");


    ROS_INFO("Now sending goal");
    ac.sendGoal(graspGoal);

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(15.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");

    return 0;
}
