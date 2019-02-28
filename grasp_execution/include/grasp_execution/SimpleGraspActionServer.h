#ifndef GRASP_EXECUTION_SIMPLEGRASPACTIONSERVER_H
#define GRASP_EXECUTION_SIMPLEGRASPACTIONSERVER_H

#include <grasp_execution/GraspActionServer.h>

#include <actionlib/client/simple_action_client.h>
#include <grasp_execution_msgs/GraspControlAction.h>

namespace grasp_execution
{

/**
 * This implementation of GraspActionServer simply forwards the target JointState
 * as a grasp_execution_msgs/GraspControl.action. It does not support gripper translation
 * (approach / retreat) and is therefore a very simple implementation. All it does
 * is set the gripper to the moveit_msgs::Grasp::grasp_posture for a grasp,
 * and to moveit_msgs::Grasp::pre_grasp_posture for an ungrasp.
 *
 * \author Jennifer Buehler
 * \date March 2016
 */
class SimpleGraspActionServer: public GraspActionServer
{
protected:
    typedef actionlib::SimpleActionClient<grasp_execution_msgs::GraspControlAction> GraspControlActionClientT;
    typedef grasp_execution_msgs::GraspControlFeedbackConstPtr GraspControlFeedbackConstPtrT;
    typedef grasp_execution_msgs::GraspControlResultConstPtr GraspControlResultConstPtrT;

public:

    /**
     */
    SimpleGraspActionServer(ros::NodeHandle& _node,
        const std::string& _grasp_action_topic,
        const std::string& _grasp_control_action_topic,
        GraspEligibilityCheckerPtr& _eligibility_checker);

    virtual ~SimpleGraspActionServer();

private:

    virtual bool initImpl();

    virtual void shutdownImpl();

    virtual void actionCallbackImpl(const ActionGoalHandleT& goal);

    virtual void actionCancelCallbackImpl(ActionGoalHandleT& goal);

    virtual bool executionEligiblePreCheck(const GoalT& goal);

    virtual void startGraspExecution(const sensor_msgs::JointState& graspPosture);

    void graspControlDoneCallback(const actionlib::SimpleClientGoalState& state, const GraspControlResultConstPtrT& result);

    void graspControlFeedbackCallback(const GraspControlFeedbackConstPtrT& feedback);

    void graspControlActiveCallback();

    GraspControlActionClientT graspControlClient;
};

}

#endif   // GRASP_EXECUTION_SIMPLEGRASPACTIONSERVER_H
