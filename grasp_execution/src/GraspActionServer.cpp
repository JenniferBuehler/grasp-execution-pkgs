#include <grasp_execution/GraspActionServer.h>

using grasp_execution::GraspActionServer;

GraspActionServer::GraspActionServer(ros::NodeHandle& _node,
    const std::string& _grasp_action_topic,
    GraspEligibilityCheckerPtr& _eligibility_checker):
    ActionServerT(_node,_grasp_action_topic),
    eligibilityChecker(_eligibility_checker)
{
}

bool GraspActionServer::canAccept(const ActionGoalHandleT& goal)
{
    GoalConstPtrT graspGoal = goal.getGoal();

    if (graspGoal->gripper_approach_trajectory.points.empty() 
        && (graspGoal->grasp.grasp.pre_grasp_approach.desired_distance > 1e-03))
    {
        ROS_WARN_STREAM("GraspActionServer: Inconsistency in grasp action. "<<
            " The grasp has a desired distance "
            << graspGoal->grasp.grasp.pre_grasp_approach.desired_distance <<
            " for the approach, but no trajectory is specified. The approach "<<
            " specification may do nothing in the action.");
    }

    if (graspGoal->gripper_retreat_trajectory.points.empty() 
        && (graspGoal->grasp.grasp.post_grasp_retreat.desired_distance > 1e-03))
    {
        ROS_WARN_STREAM("GraspActionServer: Inconsistency in grasp action. "<<
            " The grasp has a desired distance "
            << graspGoal->grasp.grasp.post_grasp_retreat.desired_distance <<
            " for the retreat, but no trajectory is specified. The retreat " <<
            " specification may do nothing in the action.");
    }

    if (!executionEligiblePreCheck(*graspGoal)) return false;
    if (!eligibilityChecker->executionEligible(*graspGoal)) return false;
    return executionEligiblePostCheck(*graspGoal);

}
