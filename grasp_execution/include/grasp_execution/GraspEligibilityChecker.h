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
	typedef grasp_execution_msgs::GraspGoal GraspGoalT;
public:

	/**
	 * \param _effectorPosAccuracy as \e _effectorOriAccuracy: default **tolerance** (/accuracy)
     *      for the current end effector pose (as opposed to the target pose) to accept a grasp/ungrasp action.
     *      This is the default if nothing is specified in the message.
     * \param _jointAnglesAccuracy default tolerance for the joints (in rad) when checking against joint state
     *      targets.
	 */
	GraspEligibilityChecker(ros::NodeHandle& node,
		const float& _effectorPosAccuracy,
		const float& _effectorOriAccuracy,
        const float& _jointAnglesAccuracy);

	virtual ~GraspEligibilityChecker();


    /**
     * Returns true if the implementaiton also checks for correct joint states
     * as precondition to the grasp/ungrasp. If it returns false, only the
     * end effector pose is checked to be as expected.
     */
    virtual bool checksJointStates();

    /**
     * Connects subscriber to receive most recent joint states. This only
     * makes sense if checkJointStates() returns true, and if the Grasp actions
     * sent to not already contain the most recent joint state (i.e. whichever
     * node generates the grasp action requests, does leave the field
     * Grasp::curr_joint_state uninitialized).
     */
    void connectSubscriber(const std::string& joint_states_topic);

    /**
     * \retval true action eligible
     * \retval false action not eligible because arm not in required state
     */
	virtual bool executionEligible(const GraspGoalT& graspAction);

protected:

	/**
     * Uses /tf to determine eligibility. The pose of \e effectorLinkFrame (in /tf)
     * is compared to \e graspPose.
	 * \param graspPose the expected pose the end-effector (frame effectorLinkFrame)
     *      has to be at to perform the grasp
	 */
	bool graspExecutionEligible(const std::string& effectorLinkFrame,
            const geometry_msgs::PoseStamped& graspPose,
			const float& _effectorPosAccuracy,
            const float& _effectorOriAccuracy);

    /**
     * Checks for consistency in the grasp goal. In particular, checks
     * whether the JointTrajectory (grasp_trajectory) relating to the grasp
     * conforms to the fields in the moveit_msgs::Grasp, i.e. the
     * last trajectory point has to be the same as the
     * moveit_msgs::Grasp::(pre_)grasp_posture.
     * \param useJointAnglesAccuracy the tolerance to use for comparing joint states
     */
    bool goalJointStatesConsistent(const GraspGoalT& graspGoal, const float useJointAnglesAccuracy) const;

 	// default tolerance for the effector target pose and actual effector pose to be
	// similar enough to accept a grasp action.
	float effectorPosAccuracy;

 	// default tolerance for the effector target orientation and actual effector orientation to be
	// similar enough to accept a grasp action (in rad).
	float effectorOriAccuracy;

    // default tolerance for the joints (in rad) when checking against joint state targets
    float jointAnglesAccuracy;

	typedef convenience_ros_functions::TypedSubscriber<sensor_msgs::JointState> JointStateSubscriberT;
	JointStateSubscriberT joint_states_sub;

};

}

#endif   // GRASP_EXECUTION_GRASPELIGIBILITYCHECKER_H
