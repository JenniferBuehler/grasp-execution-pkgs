#ifndef GRASP_EXECUTION_GRASPACTIONSERVER_H
#define GRASP_EXECUTION_GRASPACTIONSERVER_H

#include <actionlib/client/simple_action_client.h>

#include <grasp_execution_msgs/GraspAction.h>
#include <grasp_execution_msgs/GraspControlAction.h>
#include <grasp_execution/GraspEligibilityChecker.h>

#include <architecture_binding/SharedPtr.h>
#include <convenience_ros_functions/TypedSubscriber.h>
#include <convenience_ros_functions/ActionServer.h>

namespace grasp_execution
{

/**
 * Action server that can be used for grasping and un-grasping.
 * The action message accepted is grasp_execution_msgs/Grasp.action. Please also refer
 * to this message for documentation about the inputs / outputs.
 *
 * This server can provide different implementations for handling the grasp action
 * by being derived.
 *
 * \author Jennifer Buehler
 * \date March 2016
 */
class GraspActionServer: public convenience_ros_functions::ActionServer<grasp_execution_msgs::GraspAction>
{
protected:
    typedef convenience_ros_functions::ActionServer<grasp_execution_msgs::GraspAction> ActionServerT;

    typedef actionlib::SimpleActionClient<grasp_execution_msgs::GraspControlAction> GraspControlActionClientT;
    typedef grasp_execution_msgs::GraspControlFeedbackConstPtr GraspControlFeedbackConstPtrT;
    typedef grasp_execution_msgs::GraspControlResultConstPtr GraspControlResultConstPtrT;
        
    /*typedef grasp_execution_msgs::GraspGoal GraspGoalT;
    typedef grasp_execution_msgs::GraspGoalPtr GraspGoalPtrT;
    typedef grasp_execution_msgs::GraspGoalConstPtr GraspGoalConstPtrT;
      
    //typedef typename GraspControlActionClientT::Result GraspResult;  
    typedef grasp_execution_msgs::GraspResult GraspResult;
*/
public:
    typedef architecture_binding::shared_ptr<grasp_execution::GraspEligibilityChecker>::type GraspEligibilityCheckerPtr;

    /**
     */
    GraspActionServer(ros::NodeHandle& _node,
        const std::string& _grasp_action_topic,
        const std::string& _grasp_control_action_topic,
        GraspEligibilityCheckerPtr& _eligibility_checker):
        ActionServerT(_node,_grasp_action_topic),
        eligibilityChecker(_eligibility_checker),
        graspControlClient(_node, _grasp_control_action_topic),
        jointStateSub(_node) 
    {
    }

    virtual ~GraspActionServer()
    {
        shutdown();
    }

protected:

    virtual bool initImpl(){
        ROS_INFO("Initialising GraspServer");
        ROS_INFO("Waiting a while for gripper action server to start...");
        bool gripOnline=graspControlClient.waitForServer(ros::Duration(2));
        if (!gripOnline) ROS_WARN("Gripper control action server not online, this might lead to goals not being accepted if it doesn't start soon");
        return true;
    }

    virtual void shutdownImpl(){
        ROS_INFO("Shutting down GraspServer");
    }


    virtual bool canAccept(const ActionGoalHandleT& goal)
    {
        return executionEligible(goal) > 0;
    }

    /**
     * Receive a new goal: subclasses implementation. No need to set the goal to accepted, rejected, etc.
     * This function should just be used to initialize interal fields based on the values of the
     * goal handle, and to kick off the execution of the action. Method canAccept() should have been
     * used before to rule out that this action can be started.
     *
     * Once the action is done, call method currentActionDone() to finalize the execution of
     * this goal.
     */
    virtual void actionCallbackImpl(const ActionGoalHandleT& goal)
    {
        const manipulation_msgs::Grasp& grasp = goal.getGoal()->grasp.grasp;
        if (goal.getGoal()->is_grasp){
            const sensor_msgs::JointState& graspPosture=grasp.grasp_posture;
            this->startGraspExecution(graspPosture);
        }else{ 
            const sensor_msgs::JointState& graspPosture=grasp.pre_grasp_posture;    
            this->startGraspExecution(graspPosture);
        }
    }

    /**
     * Receive a cancel instruction: subclasses implementation.
     * Only needs to be implemented if any special actions are required
     * upon canceling the action. 
     */
    virtual void actionCancelCallbackImpl(ActionGoalHandleT& goal)
    {
        ROS_INFO_STREAM("GraspActionServer: Received cancel goal");
        graspControlClient.cancelGoal();
    }


    /**
     * \retval 1 action eligible
     * \retval -1 grasp control server not connected, cannot execute action
     * \retval -2 action not eligible because arm not in required state
     */
    int executionEligible(const ActionGoalHandleT& goal) {
        if (!graspControlClient.isServerConnected()){
            ROS_ERROR("Gripper action client not connected, can't accept grasp goal.");
            return -1;
        }
        GoalConstPtrT graspGoal = goal.getGoal();
        bool eligible = eligibilityChecker->executionEligible(*graspGoal);
        return eligible ? 1 : -2;
    }

private:
    virtual void startGraspExecution(const sensor_msgs::JointState& graspPosture){
        grasp_execution_msgs::GraspControlGoal gripGoal;
        gripGoal.target_joint_state = graspPosture;
        gripGoal.use_trajectory=false;
        gripGoal.closing=true;

        ROS_INFO("Sending gripper goal.");
        graspControlClient.sendGoal(gripGoal,
            boost::bind(&GraspActionServer::gripControlDoneCallback, this, _1, _2),
            boost::bind(&GraspActionServer::gripControlActiveCallback, this), 
            boost::bind(&GraspActionServer::gripControlFeedbackCallback, this, _1));
    }


    void gripControlDoneCallback(const actionlib::SimpleClientGoalState& state, const GraspControlResultConstPtrT& result){
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

    void gripControlFeedbackCallback(const GraspControlFeedbackConstPtrT& feedback){
        //ROS_INFO("GraspServer::gripControlFeedbackCallback");
        ROS_INFO_STREAM(feedback);
    }

    void gripControlActiveCallback(){
        //ROS_INFO("GraspServer::gripControlActiveCallback");
    }

    GraspEligibilityCheckerPtr eligibilityChecker;
    GraspControlActionClientT graspControlClient;
    typedef convenience_ros_functions::TypedSubscriber<sensor_msgs::JointState> JointStateSubscriber;
    JointStateSubscriber jointStateSub;
};

}

#endif   // GRASP_EXECUTION_GRASPACTIONSERVER_H
