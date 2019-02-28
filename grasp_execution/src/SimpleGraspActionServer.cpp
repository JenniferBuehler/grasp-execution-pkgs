#include <grasp_execution/SimpleGraspActionServer.h>
#include "GraspStateHelper.h"

using grasp_execution::SimpleGraspActionServer;

SimpleGraspActionServer::SimpleGraspActionServer(ros::NodeHandle& _node,
    const std::string& _grasp_action_topic,
    const std::string& _grasp_control_action_topic,
    GraspEligibilityCheckerPtr& _eligibility_checker):
    GraspActionServer(_node,_grasp_action_topic, _eligibility_checker),
    graspControlClient(_node, _grasp_control_action_topic)
{
}

SimpleGraspActionServer::~SimpleGraspActionServer()
{
}

bool SimpleGraspActionServer::initImpl(){
    ROS_INFO("Initialising GraspServer");
    ROS_INFO("Waiting a while for grasp control action server to start...");
    bool graspControllerOnline=graspControlClient.waitForServer(ros::Duration(2));
    if (!graspControllerOnline)
        ROS_WARN("Grasp control action server not online, this might lead to goals not being accepted if it doesn't start soon");
    return true;
}

void SimpleGraspActionServer::shutdownImpl(){
    ROS_INFO("Shutting down GraspServer");
}

void SimpleGraspActionServer::actionCallbackImpl(const ActionGoalHandleT& goal)
{
    const moveit_msgs::Grasp& grasp = goal.getGoal()->grasp.grasp;
    if (goal.getGoal()->is_grasp){

        sensor_msgs::JointState graspPosture;
        int ret = getStateFromTrajectory(grasp.grasp_posture, graspPosture);
        if (ret != 0)
        {
          ROS_ERROR_STREAM("Could not get JointState from JointTrajectory, "
           << "return code " << ret << ". Won't execute grasp action.");
          return;
        }
        this->startGraspExecution(graspPosture);
    }else{ 
        sensor_msgs::JointState graspPosture;
        int ret = getStateFromTrajectory(grasp.pre_grasp_posture, graspPosture);
        if (ret != 0)
        {
          ROS_ERROR_STREAM("Could not get JointState from JointTrajectory, "
           << "return code " << ret << ". Won't execute grasp action.");
          return;
        }
        this->startGraspExecution(graspPosture);
    }
}

void SimpleGraspActionServer::actionCancelCallbackImpl(ActionGoalHandleT& goal)
{
    ROS_INFO_STREAM("SimpleGraspActionServer: Received cancel goal");
    graspControlClient.cancelGoal();
}

bool SimpleGraspActionServer::executionEligiblePreCheck(const GoalT& goal)
{
    if (!graspControlClient.isServerConnected()){
        ROS_ERROR("Grasp control action client not connected, can't accept grasp goal.");
        return false;
    }
    if ((goal.gripper_approach_trajectory.points.size() > 0)
        || (goal.gripper_retreat_trajectory.points.size() > 0))
    {
        ROS_ERROR("SimpleGraspActionServer does not support any gripper translation in the Grasp.action message.");
        return false; 
    }
    return true;
}

void SimpleGraspActionServer::startGraspExecution(const sensor_msgs::JointState& graspPosture){
    grasp_execution_msgs::GraspControlGoal gripGoal;
    gripGoal.target_joint_state = graspPosture;
    gripGoal.use_trajectory=false;
    gripGoal.closing=true;

    ROS_INFO("Sending grasp control goal.");
    graspControlClient.sendGoal(gripGoal,
        boost::bind(&SimpleGraspActionServer::graspControlDoneCallback, this, _1, _2),
        boost::bind(&SimpleGraspActionServer::graspControlActiveCallback, this), 
        boost::bind(&SimpleGraspActionServer::graspControlFeedbackCallback, this, _1));
}


void SimpleGraspActionServer::graspControlDoneCallback(const actionlib::SimpleClientGoalState& state, const GraspControlResultConstPtrT& result){
    //ROS_INFO("GraspControlGoal returned with status: %s.",state.toString().c_str());
    ResultT graspResult;
    if ((state!=actionlib::SimpleClientGoalState::SUCCEEDED) || !result->success) {
        ROS_ERROR_STREAM("Action failed.");
        graspResult.success=false;
        graspResult.execution_time=ros::Duration(0);
    }else{
        graspResult.success=true;
        graspResult.execution_time=ros::Duration(0);
        graspResult.execution_time=ros::Duration(this->timeRunning());
    }
    this->currentActionDone(graspResult,state);        
}

void SimpleGraspActionServer::graspControlFeedbackCallback(const GraspControlFeedbackConstPtrT& feedback){
    // ROS_INFO("GraspServer::graspControlFeedbackCallback");
    // ROS_INFO_STREAM(feedback);
}

void SimpleGraspActionServer::graspControlActiveCallback(){
    //ROS_INFO("GraspServer::graspControlActiveCallback");
}
