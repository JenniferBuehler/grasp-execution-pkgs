#ifndef GRASP_EXECUTION_GRASPACTIONSERVER_H
#define GRASP_EXECUTION_GRASPACTIONSERVER_H

#include <grasp_execution_msgs/GraspAction.h>
#include <grasp_execution/GraspEligibilityChecker.h>
#include <baselib_binding/SharedPtr.h>
#include <convenience_ros_functions/ActionServer.h>

namespace grasp_execution
{

/**
 * \brief Action server that can be used for grasping and un-grasping.
 * The action message accepted is grasp_execution_msgs/Grasp.action. Please also refer
 * to this message for documentation about the inputs / outputs.
 *
 * This server can provide different implementations for handling the grasp action
 * by being derived. The base class acts as common interface and does eligibility
 * checking of an arriving action with a GraspEligibilityChecker. 
 *
 * \author Jennifer Buehler
 * \date March 2016
 */
class GraspActionServer:
    public convenience_ros_functions::ActionServer<grasp_execution_msgs::GraspAction>
{
protected:
    typedef convenience_ros_functions::ActionServer<grasp_execution_msgs::GraspAction> ActionServerT;
       
public:
    typedef baselib_binding::shared_ptr<grasp_execution::GraspEligibilityChecker>::type GraspEligibilityCheckerPtr;

    /**
     * \param _grasp_action_topic name/topic of this action server.
     * \param _eligibility_checker the GraspEligibilityChecker which should be used.
     */
    GraspActionServer(ros::NodeHandle& _node,
        const std::string& _grasp_action_topic,
        GraspEligibilityCheckerPtr& _eligibility_checker);

    virtual ~GraspActionServer() {}

protected:
    /**
     * See superclass. This may be implemented by subclasses if required,
     * and does nothing by default.
     */
    virtual bool initImpl() {} 

    /**
     * See superclass. This may be implemented by subclasses if required,
     * and does nothing by default.
     */
    virtual void shutdownImpl() {} 


    /**
     * See superclass. This remains pure virtual and has to be implemented
     * by subclasses.
     */
    virtual void actionCallbackImpl(const ActionGoalHandleT& goal) = 0;

    /**
     * See superclass. This may be implemented by subclasses if required,
     * and does nothing by default.
     */
    virtual void actionCancelCallbackImpl(ActionGoalHandleT& goal) {}

    /**
     * See superclass. While subclasses may re-implement this, it
     * is not recommended. Use method executionEligiblePreCheck()
     * and executionEligiblePostCheck() instead,
     * which is called from here.
     */
    virtual bool canAccept(const ActionGoalHandleT& goal); 

    /**
     * Checks additional requirements, aside from eligibility 
     * determined by GraspEligibilityChecker, which determine whether the action
     * is eligiblie. This covers all additional checks which need to be
     * performed by subclasses. It is called *before* the checks of the
     * GraspEligibilityChecker. See also executionEligiblePostCheck().
     */
    virtual bool executionEligiblePreCheck(const GoalT& goal)
    {
        return true;
    }
   
    /**
     * Like executionEligiblePreCheck(), but is called *after* the checks
     * of the GraspEligibilityChecker.
     */ 
    virtual bool executionEligiblePostCheck(const GoalT& goal)
    {
        return true;
    }

    GraspEligibilityCheckerPtr eligibilityChecker;
};

}

#endif   // GRASP_EXECUTION_GRASPACTIONSERVER_H
