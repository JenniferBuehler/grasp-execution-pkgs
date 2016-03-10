#ifndef GRASP_EXECUTION_REACH_TEST_MOVEITPLANNER_H
#define GRASP_EXECUTION_REACH_TEST_MOVEITPLANNER_H

#include <ros/ros.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/GetCartesianPath.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/PlanningScene.h>

namespace grasp_execution_reach_test
{

/**
 * Collection of convenience functions to help plan motion trajectories with MoveIt!.
 * This class already maintains the necessary server clients and sends the requests.
 *
 * \author Jennifer Buehler
 * \date March 2016
 */
class MoveItPlanner
{
public:
    /**
     * \param _moveit_motion_plan_service motion plan service of moveit (to plan a kinematic path)
     * \param _moveit_check_state_validity_service state validity checking service of moveit
     */
    MoveItPlanner(ros::NodeHandle& n,
		const std::string& _moveit_motion_plan_service="/plan_kinematic_path",
		const std::string& _moveit_check_state_validity_service="/check_state_validity");

    ~MoveItPlanner();


	/**
	 * Builds goal constraint for the link to be at this pose. 
	 * \param type if 0, only position is considered. If 1, position and orientation
     *      are considered, and if 2 then only orientation is considered.
	 */
	static moveit_msgs::Constraints getPoseConstraint(const std::string& link_name,
        const geometry_msgs::PoseStamped& pose, double tolerance_pos, double tolerance_angle, int type);


    /**
     * Builds a constraint at which \e link_name has the given orientation and tolerances.
     */
	static moveit_msgs::OrientationConstraint getOrientationConstraint(const std::string& link_name,
        const geometry_msgs::QuaternionStamped& quat, 
		const float& x_tolerance, 
		const float& y_tolerance,
		const float& z_tolerance); 

	/**
	 * Builds the joint constraint such that it corresponds to the passed joint_state
	 */
	static moveit_msgs::Constraints getJointConstraint(const std::string& link_name,
        const sensor_msgs::JointState& joint_state, const float& joint_tolerance);

	/**
     * Builds a position constraint for \e link_name with a sphere around the \e target_pose 
	 * \param arm_reach_span the maximum span the arm can reach (radius of the reaching sphere).
     *      Can be an overestimation of it, but should not underestimate.
	 */
	static moveit_msgs::PositionConstraint getSpherePoseConstraint(
			const std::string &link_name, 
			const geometry_msgs::PoseStamped &target_pose, 
			float arm_reach_span);
	
	/**
     * Builds a position constraint for \e link_name with a box at \e box_origin of the given
     * x/y/z dimensions.
	 */
	static moveit_msgs::PositionConstraint getBoxConstraint(const std::string &link_name,
        const geometry_msgs::PoseStamped& box_origin, const double& x, const double& y, const double& z);

   
    /**
     * \param arm_base_pose the pose of the base of the arm. This is required to generate the MoveIt! workspace.
     * \param arm_reach_span maximum distance from frame in \e arm_base_pose to the end effector when it is stretched out at its furthest.
     *      This value will be used to further extend the workspace so that the full space of the arm reachability is covered.
     * \param path_constraints path constraints for motion planning, or NULL if none are to be used.
     * \param planning_group name of the planning group
     * \param goal_constraints the goal constraints for planning
     * \param start_state the start joint state to plan from. Only joints in the planning group
     *      may be in the joint state!
     */
    moveit_msgs::MoveItErrorCodes requestTrajectory(
        const geometry_msgs::PoseStamped& arm_base_pose,
        float arm_reach_span,
        const std::string& planning_group, 
        const moveit_msgs::Constraints& goal_constraints,
        const moveit_msgs::Constraints * path_constraints,  
        const sensor_msgs::JointState& start_state,
        moveit_msgs::RobotTrajectory& result_traj); 

    /**
     * Like requestTrajectory(), but does so for mobile robots with a virtual joint.
     * \param robot_pose the robot pose to plan from. This is required to generate the
     *      MoveIt! workspace and the robot's path to navigate.
     *      See also \e target_pose.
     * \param target_pose the target pose which the robot should be able to reach.
     *      This is only used for creating the workspace, so
     *      it can be the pose furthest from \e robot_pose which the robot can
     *      navigate to, or reach the arm out to.
     *      The workspace will be generated as a box between \e robot_pose and
     *      \e target_pose, which will be fruther extended
     *      by \e arm_reach_span
     * \param arm_reach_span maximum distance from frame in \e start_pose to the end
     *      effector when it is stretched out at its furthest.
     *      This value will be used to further extend the workspace so that the
     *      full space of the arm reachability is covered.
     * \param path_constraints path constraints for motion planning, or NULL if none are to be used.
     * \param planning_group name of the planning group
     * \param goal_constraints the goal constraints for planning
     * \param start_state the start joint state to plan from. Only joints in the planning group
     *      may be in the joint state!
     * \param virtual_joint_name the name of the virtual joint
     * \param target_frame_virtual_joint MoveIt! likes to have the virtual joint state in the
     *      frame which it was specified in its configuration. Specify this frame here, e.g. "odom".
     */
    moveit_msgs::MoveItErrorCodes requestTrajectoryForMobileRobot(
        const geometry_msgs::PoseStamped& start_pose, 
        const geometry_msgs::PoseStamped& target_pose, 
        float arm_reach_span, const std::string& planning_group, 
        const moveit_msgs::Constraints& goal_constraints,
        const moveit_msgs::Constraints * path_constraints,  
        const sensor_msgs::JointState& start_state,
        const std::string& virtual_joint_name,
        const std::string& target_frame_virtual_joint,
        moveit_msgs::RobotTrajectory& result_traj); 

    /**
     * \param mdjs the MultiDOFJointState of the robot, if it is a mobile robot
     *      which has a virtual joint. NULL if this is not to be used.
     * \param group_name name of the planning group
     * \param goal_constraints the goal constraints for planning
     * \param start_state the start joint state to plan from. Only joints in the planning group
     *      may be in the joint state!
     */
    moveit_msgs::MoveItErrorCodes requestTrajectory(const std::string& group_name, 
        const std::vector<moveit_msgs::Constraints>& goal_constraints, 
        const sensor_msgs::JointState& from_state, 
        const moveit_msgs::WorkspaceParameters& wspace,
        const sensor_msgs::MultiDOFJointState *mdjs,
        const moveit_msgs::Constraints* path_constraints,
        moveit_msgs::RobotTrajectory& result);

private:
	bool init();
	void shutdown();

    moveit_msgs::MotionPlanResponse sendServiceRequest(moveit_msgs::MotionPlanRequest& request); 

    /**
     * Creates a moveit! workspace that suits the robot having to move from the current position (from) to reach to position 'to'.
     * The workspace will be generated as a box between \e robot_pose and \e target_pose, which will be fruther extended
     * by \e arm_reach_span
     * \param arm_reach_span maximum distance that the robot can reach out from the current position (frame_id in 'from')
     * \param from the position where the robot currently is located (this should be the base frame ID of the robot)
     * \param to the position where the robot should reach. This can be in any frame.
     * \return false if transform between poses is not possible
     */
    bool makeWorkspace(const geometry_msgs::PoseStamped& from,
                       const geometry_msgs::PoseStamped& to,
                       float arm_reach_span, moveit_msgs::WorkspaceParameters& result);

    /**
     * Creates a workspace as an axis-aligned box around \origin, where distance from origin
     * along each axis to the box boundary is \e arm_reach_span
     */
    bool makeWorkspace(const geometry_msgs::PoseStamped& origin,
                       float arm_reach_span, 
                       moveit_msgs::WorkspaceParameters& wspace);



    bool isValidState(const moveit_msgs::RobotState& robot_state, const std::string& group);

	std::string moveit_motion_plan_service;
	std::string moveit_check_state_validity_service;

	ros::ServiceClient motion_plan_client;
	ros::ServiceClient state_validity_client;

    ros::NodeHandle node;


};
}  // namespace

#endif   // GRASP_EXECUTION_REACH_TEST_MOVEITPLANNER_H
