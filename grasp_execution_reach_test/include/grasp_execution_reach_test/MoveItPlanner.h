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

class MoveItPlanner
{
public:

    MoveItPlanner(ros::NodeHandle& n,
		const std::string& _moveit_motion_plan_service_topic,
		const std::string& _moveit_check_state_validity_service_topic,
		const std::string& _moveit_planning_scene_topic);

    ~MoveItPlanner();

    /**
     * \param target the target pose to reach, or the maximum target pose the robot effector should be able to reach out around the current robot pose.
     *        This is only used to generate workspace parameters, hence only the distance from the start_pose matters.
     * \param armReachSpan maximum distance from <start_pose> (e.g. robot base) that the arm can reach away
     */
    moveit_msgs::MoveItErrorCodes requestTrajectory(
        const geometry_msgs::PoseStamped& start_pose, 
        const geometry_msgs::PoseStamped& target, 
        float reachOutDist, const std::string& planning_group, 
        const moveit_msgs::Constraints& goal_constraints,
        const moveit_msgs::Constraints * pathConstraints,  
        const sensor_msgs::JointState& startState,
        const std::string& virtual_joint_name,
        moveit_msgs::RobotTrajectory& resultTraj); 

    moveit_msgs::MoveItErrorCodes motionRequest(const std::string& groupName, 
        const std::vector<moveit_msgs::Constraints>& goal_constraints, 
        const sensor_msgs::JointState& fromState, 
        const moveit_msgs::WorkspaceParameters& wspace,
        const sensor_msgs::MultiDOFJointState *mdjs = NULL,
        const moveit_msgs::Constraints* pathConstraints,
        moveit_msgs::RobotTrajectory& result);

private:
	bool init();
	void shutdown();

    moveit_msgs::MotionPlanResponse sendServiceRequest(moveit_msgs::MotionPlanRequest& request); 

    /**
     * Creates a moveit! workspace that suits the robot having to move from the current position (from) to reach to position 'to'.
     * \param reachOutDist maximum distance that the robot can reach out from the current position (frame_id in 'from')
     * \param from the position where the robot currently is located (this should be the base frame ID of the robot)
     * \param to the position where the robot should reach. This can be in any frame.
     * \return false if transform between poses is not possible
     */
    bool makeWorkspace(const geometry_msgs::PoseStamped& from, const geometry_msgs::PoseStamped& to, float reachOutDist, moveit_msgs::WorkspaceParameters& result);

    bool isValidState(const moveit_msgs::RobotState& robotState, const std::string& group);

	std::string moveit_motion_plan_service_topic;
	std::string moveit_check_state_validity_service_topic;

	ros::ServiceClient motion_plan_client;
	ros::ServiceClient state_validity_client;

    ros::NodeHandle node;


};
}  // namespace

#endif   // GRASP_EXECUTION_REACH_TEST_MOVEITPLANNER_H
