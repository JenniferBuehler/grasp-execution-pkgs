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
 * server, according to the template ``rosed grasp_execution SimpleAutomatedGraspTemplate.yaml``.
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

protected:
    virtual bool initImpl()=0;

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
     * \retval 0 success
     * \retval -1 can't find transform from object to arm base
     * \retval -2 planning failed
     * \retval -3 execution failed
     */
    int planAndExecuteMotion(
        const std::string& object_frame_id,
        const std::string& arm_base_link,
        moveit_msgs::Constraints& reachConstraints,
        const float arm_reach_span,
        const std::string& planning_group); 

    arm_components_name_manager::ArmComponentsNameManager * jointsManager;
    moveit_planning_helper::MoveItPlanner * trajectoryPlanner;
    ros::NodeHandle node;
        
    actionlib::SimpleActionClient<grasp_execution_msgs::GraspAction> * graspActionClient;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> * jointTrajectoryActionClient;
 
    ///// all fields read from ROS parameters
    
    std::string OBJECT_NAME;
    bool GRASPING;
    double OPEN_ANGLES;
    double CLOSE_ANGLES;
    double EFF_POS_TOL;
    double EFF_ORI_TOL;
    double JOINT_ANGLE_TOL;
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


/**
 * Generates a simple grasp from top of the object and then executes it.
 * In addition to the ROS parameters of base class, also takes following
 * ROS Parameters:
 *
 * ``
 *   # end effector to be positioned this much above object (z-direction)
 *   pose_above_object: 0.17
 *   # end effector to be positioned relative to object (x-direction)
 *   x_from_object: 0.0
 *
 *   # end effector to be positioned relative to object (y-direction)
 *   y_from_object: -0.02
 * ``
 */
class SimpleAutomatedGraspFromTop: public SimpleAutomatedGraspExecution
{
public:
    SimpleAutomatedGraspFromTop();
    virtual ~SimpleAutomatedGraspFromTop();
    bool exeTest();

protected:
    virtual bool initImpl(); 

private:
    bool generateGrasp(grasp_execution_msgs::GraspGoal& graspGoal);    
    
    double POSE_ABOVE;
    double POSE_X;
    double POSE_Y;
};

}  // namespace

#endif   // GRASP_EXECUTION_SIMPLEAUTOMATEDGRASPEXECUTION_H
