#include <grasp_execution/SimpleAutomatedGraspExecution.h>
#include <convenience_ros_functions/ROSFunctions.h>
#include <convenience_math_functions/MathFunctions.h>

#include <object_msgs/Object.h>
#include <object_msgs/ObjectInfo.h>
#include <object_msgs_tools/ObjectFunctions.h>

using grasp_execution::SimpleAutomatedGraspExecution;

using object_msgs_tools::ObjectFunctions;
using moveit_planning_helper::MoveItPlanner;
using convenience_ros_functions::RobotInfo;
using convenience_math_functions::MathFunctions;
using arm_components_name_manager::ArmComponentsNameManager;


SimpleAutomatedGraspExecution::SimpleAutomatedGraspExecution():
    initialized(false),
    jointsManager(NULL),
    trajectoryPlanner(NULL),
    graspActionClient(NULL),
    jointTrajectoryActionClient(NULL),
    graspHandler(NULL)
{
    convenience_ros_functions::ROSFunctions::initSingleton();
}
SimpleAutomatedGraspExecution::~SimpleAutomatedGraspExecution()
{
    if (jointsManager) delete jointsManager;
    if (trajectoryPlanner) delete trajectoryPlanner;
    if (graspActionClient) delete graspActionClient;
    if (jointTrajectoryActionClient) delete jointTrajectoryActionClient;
    if (graspHandler) delete graspHandler;    
}

bool SimpleAutomatedGraspExecution::init()
{
    ros::NodeHandle priv("~");

    /////////// Read parameters  ///////////////////
    std::string ROBOT_NAMESPACE;
    if (!priv.hasParam("robot_namespace"))
    {
        ROS_ERROR("Node requires private parameter 'robot_namespace'");
        return false;
    }
    priv.param<std::string>("robot_namespace", ROBOT_NAMESPACE, ROBOT_NAMESPACE);

    jointsManager = new ArmComponentsNameManager(ROBOT_NAMESPACE, false);
    double maxWait=5;
    ROS_INFO("Waiting for joint info parameters to be loaded...");
    jointsManager->waitToLoadParameters(1,maxWait); 
    ROS_INFO("Parameters loaded.");

    OPEN_ANGLES=0.05;
    priv.param<double>("open_angles", OPEN_ANGLES, OPEN_ANGLES);
    CLOSE_ANGLES=0.7;
    priv.param<double>("close_angles", CLOSE_ANGLES, CLOSE_ANGLES);
  
    EFF_POS_TOL=0.02;
    EFF_ORI_TOL=0.1;
    JOINT_ANGLE_TOL=0.1;
    PLAN_TOLERANCE_FACTOR=0.1;
    priv.param<double>("effector_pos_tolerance", EFF_POS_TOL, EFF_POS_TOL);
    priv.param<double>("effector_ori_tolerance", EFF_ORI_TOL, EFF_ORI_TOL);
    priv.param<double>("joint_angle_tolerance", JOINT_ANGLE_TOL, JOINT_ANGLE_TOL);
    priv.param<double>("plan_tolerance_factor", PLAN_TOLERANCE_FACTOR, PLAN_TOLERANCE_FACTOR);

    REQUEST_OBJECTS_SERVICE="world/request_object";
    priv.param<std::string>("request_object_service", REQUEST_OBJECTS_SERVICE, REQUEST_OBJECTS_SERVICE);
    
    JOINT_STATES_TOPIC = "/joint_states";
    priv.param<std::string>("joint_states_topic", JOINT_STATES_TOPIC, JOINT_STATES_TOPIC);

    GRASP_ACTION_NAME = "/grasp_action";
    priv.param<std::string>("grasp_action_name", GRASP_ACTION_NAME, GRASP_ACTION_NAME);
    
    JOINT_TRAJECTORY_ACTION_NAME = "/joint_trajectory_action";
    priv.param<std::string>("joint_trajectory_action_name", JOINT_TRAJECTORY_ACTION_NAME, JOINT_TRAJECTORY_ACTION_NAME);

    MOVEIT_MOTION_PLAN_SERVICE="/plan_kinematic_path";
    priv.param<std::string>("moveit_motion_plan_service", MOVEIT_MOTION_PLAN_SERVICE, MOVEIT_MOTION_PLAN_SERVICE);

    MOVEIT_STATE_VALIDITY_SERVICE="/check_state_validity";
    priv.param<std::string>("moveit_state_validity_service", MOVEIT_STATE_VALIDITY_SERVICE, MOVEIT_STATE_VALIDITY_SERVICE);

    MOVEIT_GET_PLANNING_SCENE_SERVICE="/get_planning_scene";
    priv.param<std::string>("moveit_get_planning_scene_service", MOVEIT_GET_PLANNING_SCENE_SERVICE, MOVEIT_GET_PLANNING_SCENE_SERVICE);
    
    MOVEIT_SET_PLANNING_SCENE_TOPIC="/planning_scene";
    priv.param<std::string>("moveit_get_planning_scene_topic", MOVEIT_SET_PLANNING_SCENE_TOPIC, MOVEIT_SET_PLANNING_SCENE_TOPIC);

    ARM_REACH_SPAN = 2;
    priv.param<double>("arm_reach_span", ARM_REACH_SPAN, ARM_REACH_SPAN);
    PLANNING_GROUP="Arm";
    priv.param<std::string>("planning_group", PLANNING_GROUP, PLANNING_GROUP);

    trajectoryPlanner = new MoveItPlanner(node, 
        MOVEIT_MOTION_PLAN_SERVICE,
        MOVEIT_STATE_VALIDITY_SERVICE);

    //create the action client
    // true causes the client to spin its own thread
    graspActionClient = new actionlib::SimpleActionClient<grasp_execution_msgs::GraspAction> (GRASP_ACTION_NAME, true);
    ROS_INFO_STREAM("Waiting for grasp action client to start: "<< GRASP_ACTION_NAME);
    // wait for the action server to start
    graspActionClient->waitForServer(); //will wait for infinite time
    ROS_INFO("Grasp action client started.");

    // create the action client
    jointTrajectoryActionClient = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(JOINT_TRAJECTORY_ACTION_NAME, true);
    ROS_INFO_STREAM("Waiting for joint trajectory action client to start: "<< JOINT_TRAJECTORY_ACTION_NAME);
    // wait for the action server to start
    jointTrajectoryActionClient->waitForServer();  // will wait for infinite time
    ROS_INFO("Joint trajectory action client has started.");

    std::vector<std::string> gripperLinkNames=jointsManager->getGripperLinks();
    std::string palmLinkName = jointsManager->getPalmLink();
    gripperLinkNames.push_back(palmLinkName);
    
    graspHandler = new moveit_object_handling::GraspedObjectHandlerMoveIt(node,
        gripperLinkNames, MOVEIT_GET_PLANNING_SCENE_SERVICE,MOVEIT_SET_PLANNING_SCENE_TOPIC);

    graspHandler->waitForSubscribers();

    initialized = initImpl();
    return initialized;
}

bool SimpleAutomatedGraspExecution::getObjectPose(const std::string& object_name, const float timeout_wait_object, geometry_msgs::PoseStamped& pose)
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


sensor_msgs::JointState SimpleAutomatedGraspExecution::getHomeState(bool capToPI)
{
    std::vector<float> arm_init = jointsManager->getArmJointsInitPose();
    if (capToPI)
    {
        MathFunctions::capToPI(arm_init); 
    }
    sensor_msgs::JointState ret;
    jointsManager->copyToJointState(ret, 0, &arm_init, 0, true);
    ret.velocity.resize(arm_init.size(),0);
    return ret;
}

bool SimpleAutomatedGraspExecution::setFingersToCurr(const sensor_msgs::JointState& currState, sensor_msgs::JointState& targetState)
{
    std::vector<std::string> gripperJoints = jointsManager->getGripperJoints();
    std::vector<int> idxCurr;
    if (!jointsManager->getJointIndices(currState.name, idxCurr,2))
    {
        ROS_ERROR("Current state does not have finger joint names.");
        return false;
    }
        
    if (idxCurr.size() != gripperJoints.size())
    {
        ROS_ERROR_STREAM("Consistency in current state: number of joint indices ("<<idxCurr.size()<<
            ") smaller than expected ("<<gripperJoints.size()<<")");
        return false;
    }

    std::vector<int> idxTarget;
    if (!jointsManager->getJointIndices(targetState.name, idxTarget,2))
    {
        ROS_ERROR("Current state does not have finger joint names.");
        return false;
    }
        
    if (idxTarget.size() != gripperJoints.size())
    {
        ROS_ERROR_STREAM("Consistency in target state: number of joint indices ("<<idxTarget.size()<<
            ") smaller than expected ("<<gripperJoints.size()<<")");
        return false;
    }


    for (int i=0; i < gripperJoints.size(); ++i)
    {
        targetState.position[idxTarget[i]]=currState.position[idxCurr[i]];
    }
    return true;
}

int SimpleAutomatedGraspExecution::planAndExecuteMotion(
    const std::string& fixed_frame_id,
    const std::string& arm_base_link,
    moveit_msgs::Constraints& reachConstraints,
    const float arm_reach_span,
    const std::string& planning_group)
{
    // get the current arm base pose.
    geometry_msgs::PoseStamped currBasePose;
    int transRet=convenience_ros_functions::ROSFunctions::Singleton()->getTransform(
            fixed_frame_id, arm_base_link,
            currBasePose.pose,
            ros::Time(0),2,true);
    if (transRet!=0) {
        ROS_ERROR("Could not get current effector tf transform in object frame.");
        return -1;
    }
    currBasePose.header.stamp=ros::Time::now();
    currBasePose.header.frame_id=fixed_frame_id;
    // ROS_INFO_STREAM("Effector currBasePose pose: "<<currBasePose);
 
    // request joint trajectory
    sensor_msgs::JointState currArmJointState = robotInfo.getCurrentJointState(JOINT_STATES_TOPIC, node);
    // get only the arm joints of the joint state:
    jointsManager->extractFromJointState(currArmJointState,0,currArmJointState);
    // ROS_INFO_STREAM("Current arm joint state: "<<currArmJointState);

    ROS_INFO("Planning trajectory...");
    moveit_msgs::RobotTrajectory robotTrajectory;
    moveit_msgs::MoveItErrorCodes moveitRet = trajectoryPlanner->requestTrajectory(
        currBasePose,
        arm_reach_span,
        planning_group,
        reachConstraints,
        NULL,
        currArmJointState,
        robotTrajectory);

    if (moveitRet.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
        ROS_ERROR("Could not plan joint trajectory");
        return -2;
    }


    // ROS_INFO("############  Resulting joint trajectory ################");
    // ROS_INFO_STREAM(robotTrajectory.joint_trajectory);

    /////////// Execute joint trajectory  ///////////////////

    //ROS_INFO("Now constructing joint trajectory goal");

    // send a goal to the action
    control_msgs::FollowJointTrajectoryGoal jtGoal;
    jtGoal.trajectory = robotTrajectory.joint_trajectory;

    ROS_INFO("Now sending joint trajectory goal");
    jointTrajectoryActionClient->sendGoal(jtGoal);

    // wait for the action to return
    bool finished_before_timeout = jointTrajectoryActionClient->waitForResult(ros::Duration(15.0));
    actionlib::SimpleClientGoalState state = jointTrajectoryActionClient->getState();
    if (state!=actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        if (finished_before_timeout)
        {
            ROS_ERROR("Could not execute joint trajectory action: %s",state.toString().c_str());
        }
        else
        {
            ROS_ERROR("Joint trajectory action did not finish before the time out.");
        }
        return -3;
    }
    ROS_INFO("Joint trajectory action finished: %s",state.toString().c_str());
    return 0;
}


bool SimpleAutomatedGraspExecution::graspPlan(const std::string& object_name, bool doGrasp, grasp_execution_msgs::GraspGoal& graspGoal)
{
    return getGrasp(object_name, doGrasp, graspGoal);
}

bool SimpleAutomatedGraspExecution::reach(const std::string& object_name, const grasp_execution_msgs::GraspGoal& graspGoal)
{
    std::string effector_link = jointsManager->getEffectorLink();
    std::string arm_base_link = jointsManager->getArmLinks().front();
    std::string object_frame_id = object_name;
  
    // build planning constraints:
    float plan_eff_pos_tol = EFF_POS_TOL * PLAN_TOLERANCE_FACTOR;
    float plan_eff_ori_tol = EFF_ORI_TOL * PLAN_TOLERANCE_FACTOR;
    int type = 1; // 0 = only position, 1 = pos and ori, 2 = only ori
    moveit_msgs::Constraints reachConstraints = trajectoryPlanner->getPoseConstraint(effector_link,
        graspGoal.grasp.grasp.grasp_pose, plan_eff_pos_tol, plan_eff_ori_tol, type); 

    int motionRet = planAndExecuteMotion(
        object_frame_id,
        arm_base_link,
        reachConstraints,
        ARM_REACH_SPAN,
        PLANNING_GROUP);
    if (motionRet !=0)
    {
        ROS_ERROR_STREAM("Could not plan/execution motion, return code "<<motionRet);
        return false;
    }
    return true;
}
    
bool SimpleAutomatedGraspExecution::grasp(const std::string& object_name, const grasp_execution_msgs::GraspGoal& graspGoal)
{
    graspActionClient->sendGoal(graspGoal);

    //wait for the action to return
    bool finished_before_timeout = graspActionClient->waitForResult(ros::Duration(15.0));
    actionlib::SimpleClientGoalState state = graspActionClient->getState();
    if (state!=actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        if (finished_before_timeout)
        {
            ROS_ERROR("Could not execute grasp action: %s",state.toString().c_str());
        }
        else
        {
            ROS_ERROR("Grasp action did not finish before the time out.");
        }
        return false;
    }
    ROS_INFO("Grasp action finished: %s",state.toString().c_str());

    // attach object to MoveIt! robot
    std::string palmLinkName = jointsManager->getPalmLink();
    graspHandler->attachObjectToRobot(object_name,palmLinkName);
    return true;
}

bool SimpleAutomatedGraspExecution::unGrasp(const std::string& object_name, const grasp_execution_msgs::GraspGoal& graspGoal)
{
    graspActionClient->sendGoal(graspGoal);

    //wait for the action to return
    bool finished_before_timeout = graspActionClient->waitForResult(ros::Duration(15.0));
    actionlib::SimpleClientGoalState state = graspActionClient->getState();
    if (state!=actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        if (finished_before_timeout)
        {
            ROS_ERROR("Could not execute grasp action: %s",state.toString().c_str());
        }
        else
        {
            ROS_ERROR("Grasp action did not finish before the time out.");
        }
        return false;
    }
    ROS_INFO("Grasp action finished: %s",state.toString().c_str());

    // dettach object to MoveIt! robot
    graspHandler->detachObjectFromRobot(object_name);
    return true;
}


bool SimpleAutomatedGraspExecution::homeArm()
{
    sensor_msgs::JointState homeState=getHomeState(true);
    sensor_msgs::JointState currJointState = robotInfo.getCurrentJointState(JOINT_STATES_TOPIC, node);
    if (!setFingersToCurr(currJointState, homeState))
    {
        ROS_ERROR("Could not set the target finger states");
        return false;
    }

    float JOINT_PLAN_TOL=JOINT_ANGLE_TOL;
    moveit_msgs::Constraints homeConstraints = trajectoryPlanner->getJointConstraint(homeState,JOINT_PLAN_TOL); 
    
    std::string arm_base_link = jointsManager->getArmLinks().front();

    ROS_INFO_STREAM("Planning for constraints: " << homeConstraints);

    int homeMotionRet = planAndExecuteMotion(
        arm_base_link,
        arm_base_link,
        homeConstraints,
        ARM_REACH_SPAN,
        PLANNING_GROUP);
    if (homeMotionRet !=0)
    {
        ROS_ERROR_STREAM("Could not plan/execution motion to HOME, return code "<<homeMotionRet);
        return false;
    }
    return true;
}


bool SimpleAutomatedGraspExecution::graspHomeAndUngrasp(const std::string& object_name)
{
    grasp_execution_msgs::GraspGoal graspGoal;
    ROS_INFO_STREAM("###### Grasp Planning #######");
    if (!graspPlan(object_name, true, graspGoal))
    {
        ROS_ERROR_STREAM("Could not plan the grasp for "<<object_name);
        return false;
    }
    ROS_INFO_STREAM("###### Reaching #######");
    if (!reach(object_name, graspGoal))
    {
        ROS_ERROR_STREAM("Could not reach to "<<object_name);
        return false;
    }
    
    ROS_INFO_STREAM("###### Grasping #######");
    if (!grasp(object_name, graspGoal))
    {
        ROS_ERROR_STREAM("Could not grasp "<<object_name);
        return false;
    }
    
    ROS_INFO_STREAM("###### Homing arm #######");
    if (!homeArm())
    {
        ROS_ERROR_STREAM("Could not home the arm after grasping "<<object_name);
        return false;
    }
    ROS_INFO_STREAM("###### Un-Grasp Planning #######");
    grasp_execution_msgs::GraspGoal ungraspGoal;
    if (!graspPlan(object_name, false, ungraspGoal))
    {
        ROS_ERROR_STREAM("Could not plan the grasp for "<<object_name);
        return false;
    }
 
    ROS_INFO_STREAM("###### Un-Grasping #######");
    if (!unGrasp(object_name, ungraspGoal))
    {
        ROS_ERROR_STREAM("Could not un-grasp "<<object_name);
        return false;
    }
    return true;
}
