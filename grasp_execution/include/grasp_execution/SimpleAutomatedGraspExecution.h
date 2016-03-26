#ifndef GRASP_EXECUTION_SIMPLEAUTOMATEDGRASPEXECUTION_H
#define GRASP_EXECUTION_SIMPLEAUTOMATEDGRASPEXECUTION_H

#include <grasp_execution_msgs/GraspAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <arm_components_name_manager/ArmComponentsNameManager.h>
#include <convenience_ros_functions/RobotInfo.h>
#include <moveit_planning_helper/MoveItPlanner.h>
#include <moveit_object_handling/GraspedObjectHandler.h>

#include <control_msgs/FollowJointTrajectoryAction.h>

namespace grasp_execution
{

/**
 * Simple class to test automated grasping. Reads parameters from ROS parameter
 * server (within the private node namespace), according to the template
 * ``rosed grasp_execution SimpleAutomatedGraspTemplate.yaml``.
 * This class also provides some convenience functions and can be derived to
 * help a quick-start to test automated grasping.
 *
 * \author Jennifer Buehler
 * \date March 2016
 */
class SimpleAutomatedGraspExecution
{
public:
    SimpleAutomatedGraspExecution();
    virtual ~SimpleAutomatedGraspExecution();

    bool init();
    /**
     * Plans a grasp for this object
     * \param doGrasp if true, plan a grasp. If false, plan for an un-grasp
     */
    bool graspPlan(const std::string& object_name, bool doGrasp, grasp_execution_msgs::GraspGoal& graspGoal);
    /**
     * Does motion planning and execution to reach to the object before grasping it
     */
    bool reach(const std::string& object_name, const grasp_execution_msgs::GraspGoal& graspGoal);
    /**
     * Grasps the object
     */
    bool grasp(const std::string& object_name, const grasp_execution_msgs::GraspGoal& graspGoal);
    /**
     * Un-grasps the object
     */
    bool unGrasp(const std::string& object_name, const grasp_execution_msgs::GraspGoal& graspGoal);
    /**
     * Homes the arm
     */
    bool homeArm();

    /**
     * Test method which grasps the object, homes the arm, and then ungrasps
     * the object.
     */
    bool graspHomeAndUngrasp(const std::string& object_name);

protected:
    virtual bool initImpl()=0;
    /**
     * Get the grasp for this object
     * \param isGrasp generate it as a grasp if true, or as ungrasp if false.
     */
    virtual bool getGrasp(const std::string& object_name, bool isGrasp, grasp_execution_msgs::GraspGoal& graspGoal)=0;

    /**
     * \param timeout_wait_object wait this amount of seconds maximum until cube is there.
     *      This is useful if it has been spawned at the same time the node has been launched.
     */
    bool getObjectPose(const std::string& object_name, const float timeout_wait_object, geometry_msgs::PoseStamped& pose); 

    sensor_msgs::JointState getHomeState(bool capToPI); 

    /**
     * \param currState current state of arm. Must include gripper joints.
     * \param targetState target state of arm. Must include gripper joints,
     *      values will be overwritten with the ones in \e currState
     */
    bool setFingersToCurr(const sensor_msgs::JointState& currState, sensor_msgs::JointState& targetState); 

    /**
     * \param arm_base_link the link name of the base of the arm
     * \param fixed_frame_id A fixed frame to be used to determine the pose fo the \e arm_base_link from /tf
     *      transforms. This is used to generate MoveIt! workspace. This can be in any frame, it can also
     *      be the object frame since the robot is not moving.
     * \retval 0 success
     * \retval -1 can't find transform from object to arm base
     * \retval -2 planning failed
     * \retval -3 execution failed
     */
    int planAndExecuteMotion(
        const std::string& fixed_frame_id,
        const std::string& arm_base_link,
        moveit_msgs::Constraints& reachConstraints,
        const float arm_reach_span,
        const std::string& planning_group); 

    bool initialized;
    arm_components_name_manager::ArmComponentsNameManager * jointsManager;
    moveit_planning_helper::MoveItPlanner * trajectoryPlanner;
    moveit_object_handling::GraspedObjectHandlerMoveIt * graspHandler;
    convenience_ros_functions::RobotInfo robotInfo;
    ros::NodeHandle node;
        
    actionlib::SimpleActionClient<grasp_execution_msgs::GraspAction> * graspActionClient;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> * jointTrajectoryActionClient;
 

    ///// all variables read from ROS parameters
    
    double OPEN_ANGLES;
    double CLOSE_ANGLES;
    double EFF_POS_TOL;
    double EFF_ORI_TOL;
    double JOINT_ANGLE_TOL;
    double PLAN_TOLERANCE_FACTOR;
	std::string REQUEST_OBJECTS_SERVICE;
    std::string JOINT_STATES_TOPIC;
    std::string GRASP_ACTION_NAME;
    std::string JOINT_TRAJECTORY_ACTION_NAME;
	std::string MOVEIT_MOTION_PLAN_SERVICE;
	std::string MOVEIT_STATE_VALIDITY_SERVICE;
    std::string MOVEIT_GET_PLANNING_SCENE_SERVICE;
    std::string MOVEIT_SET_PLANNING_SCENE_TOPIC;
    double ARM_REACH_SPAN;
    std::string PLANNING_GROUP;
};

}  // namespace

#endif   // GRASP_EXECUTION_SIMPLEAUTOMATEDGRASPEXECUTION_H
