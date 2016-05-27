/**
 */
#ifndef GRASP_EXECUTION_SIMPLEGRASPCONTROLSERVER_H
#define GRASP_EXECUTION_SIMPLEGRASPCONTROLSERVER_H

#include <ros/ros.h>

#include <grasp_execution_msgs/GraspControlAction.h>
#include <sensor_msgs/JointState.h>

#include <baselib_binding/Thread.h>
#include <arm_components_name_manager/ArmComponentsNameManager.h>
#include <arm_components_name_manager/ArmJointStateSubscriber.h>

#include <convenience_ros_functions/ActionServer.h>


namespace grasp_execution
{

/**
 * \brief Accepts grasp_execution_msgs/GraspControl action to open or close the grippers/fingers.
 * 
 * This simple implementation publishes the goal joint state as a sensor_msgs::JointState to a
 * topic used to control joints, in order to close/open the hand.
 * The grasp is considered finished when the grippers are at the goal position, or if they
 * haven't moved for a certain time.
 *
 * \author Jennifer Buehler
 * \date March 2016
 */
class SimpleGraspControlServer:
    public convenience_ros_functions::ActionServer<grasp_execution_msgs::GraspControlAction>
{
	protected:
/*	typedef actionlib::ActionServer<grasp_execution_msgs::GraspControlAction> GraspControlActionServerT;
	typedef GraspControlActionServerT::GoalHandle GoalHandle;
*/
    typedef convenience_ros_functions::ActionServer<grasp_execution_msgs::GraspControlAction> GraspControlActionServerT;
	typedef GraspControlActionServerT::ActionGoalHandleT ActionGoalHandleT;
    typedef GraspControlActionServerT::ResultT ResultT;	
	
    public:
	/**
     * \param gripper_joint_names names of the joints which are involved in the grasping action.
     *      These are the joints which are checked against the goal state - or if they
     *      are not moving for a while, are considered to be grapsing (see \e noMoveTolerance)
	 * \param goalTolerance when the grippers are this close (in rad) to their target,
     *      we assume the grasp as finished.
	 * \param noMoveTolerance if a gripper does not move this amount (in rad) within checkStateFreq,
     *      it is considered to have met resistance (touched object).
     * \param noMoveStillCnt Number of times at which a joint has not moved (at interval \e checkStateFreq
     *       and not moved more than \e noMoveTolerance) at which point it is considered "still".
	 */
	SimpleGraspControlServer(
		ros::NodeHandle &n, 
		std::string& action_topic_name,
		std::string& joint_states_topic,
		std::string& joint_control_topic,
        const arm_components_name_manager::ArmComponentsNameManager& _joints_manager,
		float goalTolerance,
        float noMoveTolerance,
        int noMoveStillCnt,
		float checkStateFreq);

	~SimpleGraspControlServer();

    protected:
    
	bool initImpl();
	void shutdownImpl();


    virtual bool canAccept(const ActionGoalHandleT& goal);
    virtual void actionCallbackImpl(const ActionGoalHandleT& goal);
    virtual void actionCancelCallbackImpl(ActionGoalHandleT& goal);


	
	private: 
    typedef baselib_binding::unique_lock<baselib_binding::mutex>::type unique_lock;	
	

	/**
     * Finalises the execution, including calling parent classes currentActionDone(). 
 	 */
	void setExecutionFinished(bool success);

    /**
     * Checks whether the grippers are not moving and updates \e no_move_stat
     * accordingly.
     * Precondition: move_stat and no_move_stat must be initialized to the
     * size of the number of gripper joints, with all 0 values.
     * \retval -1  error doing the check
     * \retval 0 check successful, but goal conditions not reached
     * \retval 1 check successful and goal conditions reached
     */
    int updateGrippersCheck();
    /**
     * Loop for calling updateGrippersCheck at a rate of \e updateRate
     */
    static void updateGrippersCheckLoop(SimpleGraspControlServer * _this, float updateRate);

    void cancelGripperCheckThread();

    /*
	 *  tolerance (in radian) at which a goal pose of the grippers is considered reached
     */
	float GOAL_TOLERANCE;

    /*
     *  As soon as a gripper joint moves less than this amount of rads 
     *  since the last update of its position, it
	 *  is considered to not have moved. This is checked at a rate of
     *  \e gripper_angles_check_freq.
     */
	float NO_MOVE_TOLERANCE;
   
    /**
     * Number of times at which a joint has not moved (at interval
     * \e gripper_angles_check_freq and not moved more than \e NO_MOVE_TOLERANCE)
     * at which point it is considered "still".
     */ 
    int NO_MOVE_STILL_CNT;
        
    std::vector<std::string> gripper_joint_names;

    /*
	 * last gripper state that was saved. Gripper states are observed at a rate of gripper_angles_check_freq.
     */
	std::vector<float> last_gripper_angles;

    /*
	 *  time stamp of last_gripper_angles
     */
	ros::Time time_last_gripper_angles;
	
    /*
	 *  rate at which the gripper states are checked
     */
	float gripper_angles_check_freq;

    /*
     *  thread to run the loop updateGrippersCheckLoop() at the rate of
     *  gripper_angles_check_freq
     */
    baselib_binding::thread * gripper_check_thread;

    /*
	 *  number of times the grippers have not moved since the last check (since last_gripper_angles)
     *  these are given in the same order as gripper_joint_names.
     *  This field is updated by updateGrippersCheck which is called at rate gripper_angles_check_freq.
     */
    std::vector<int> no_move_stat;

    /**
     * Number of times all of the grippers together haven't moved.
     */
    int no_move_stat_all;
    
    /*
     *  number of times that movement was recorded for grippers since the goal was accepted.
     *  This field is updated by updateGrippersCheck which is called at rate gripper_angles_check_freq.
     */
    std::vector<int> move_stat;
	
    /*
	 * structure that subsribes to joint state messages and keeps the Jaco specific structures
     */
    const arm_components_name_manager::ArmComponentsNameManager joints_manager;

    /*
     * Subscribes to joint states in order to always have the most recent information.
     */
    arm_components_name_manager::ArmJointStateSubscriber joint_state_subscriber;    

    /*
	 *  target gripper joints state. Protected
     *  by goal_lock.
     */
	std::vector<float> target_gripper_angles;
    baselib_binding::recursive_mutex target_gripper_angles_mutex;

	ros::Publisher joint_control_pub;

	bool initialized;

};
}  // namespace

#endif // GRASP_EXECUTION_SIMPLEGRASPCONTROLSERVER_H
