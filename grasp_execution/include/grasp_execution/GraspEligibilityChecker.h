#ifndef GRASP_EXECUTION_GRASPELIGIBILITYCHECKER_H
#define GRASP_EXECUTION_GRASPELIGIBILITYCHECKER_H

#include <grasp_execution_msgs/GraspAction.h>
#include <convenience_ros_functions/TypedSubscriber.h>

namespace grasp_execution
{

/**
 * Class that can be used to check for eligibility of grasping and un-grasping actions.
 * 
 * The action message to be checked is of type grasp_execution_msgs/Grasp.action. Please also refer
 * to this message for documentation about the inputs / outputs.
 *
 * This class provides the option to subscribe to joint states in order to check eligibility criteria.
 *
 * Different implementations for handling the eligibility checks 
 * can be achieved by deriving this class.
 * The default implementation in this base class only checks for the end effector pose and
 * ignores the joint states.
 *
 * \author Jennifer Buehler
 * \date March 2016
 */
class GraspEligibilityChecker
{
protected:
	typedef grasp_execution_msgs::GraspGoal GraspActionGoalT;
public:

	/**
	 * \param _effectorPosAccuracy as \e _effectorOriAccuracy: default **tolerance** (/accuracy)
     *      for the current end effector pose (as opposed to the target pose) to accept a grasp/ungrasp action.
     *      This is the default if nothing is specified in the message.
	 */
	GraspEligibilityChecker(ros::NodeHandle& node,
        const std::string& joint_states_topic,
		const float& _effectorPosAccuracy,
		const float& _effectorOriAccuracy);

	virtual ~GraspEligibilityChecker(); 


    /**
     * Returns true if the implementaiton also checks for correct joint states
     * as precondition to the grasp/ungrasp.
     */
    virtual bool checksJointStates();

    /**
     * \retval true action eligible
     * \retval false action not eligible because arm not in required state
     */
	virtual bool executionEligible(const GraspActionGoalT& graspAction);

protected:
    

	/**
	 * \param graspPose the pose the end-effector (frame effectorLinkFrame) has to be at to perform the grasp
	 */
	bool graspExecutionEligible(const std::string& effectorLinkFrame,
            const geometry_msgs::PoseStamped& graspPose, 
			const float& _effectorPosAccuracy,
            const float& _effectorOriAccuracy);
private:
    
    void connectSubscriber(const std::string& joint_states_topic);

 	// default tolerance for the effector target pose and actual effector pose to be
	// similar enough to accept a grasp action.
	float effectorPosAccuracy;

 	// default tolerance for the effector target orientation and actual effector orientation to be
	// similar enough to accept a grasp action (in rad).
	float effectorOriAccuracy;

	typedef convenience_ros_functions::TypedSubscriber<sensor_msgs::JointState> JointStateSubscriberT;
	JointStateSubscriberT joint_states_sub;

};

}

#endif   // GRASP_EXECUTION_GRASPELIGIBILITYCHECKER_H
