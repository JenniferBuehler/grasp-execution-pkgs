#include <grasp_execution/GraspEligibilityChecker.h>
#include <convenience_ros_functions/ROSFunctions.h>
#include "GraspStateHelper.h"

using convenience_ros_functions::ROSFunctions;
using grasp_execution::GraspEligibilityChecker;

GraspEligibilityChecker::GraspEligibilityChecker(ros::NodeHandle& node,
    const float& _effectorPosAccuracy,
    const float& _effectorOriAccuracy,
    const float& _jointAnglesAccuracy):
    effectorPosAccuracy(_effectorPosAccuracy),
    effectorOriAccuracy(_effectorOriAccuracy),
    jointAnglesAccuracy(_jointAnglesAccuracy),
    joint_states_sub(node)
{
    ROSFunctions::initSingleton();
}

GraspEligibilityChecker::~GraspEligibilityChecker()
{
    joint_states_sub.stop();
}

void GraspEligibilityChecker::connectSubscriber(const std::string& joint_states_topic)
{
    if (!checksJointStates())
    {
        ROS_WARN_STREAM("Not connecting joint state subscriber because the "
            <<"implementation of this class does not support checking the joint states");
        return;
    }
    joint_states_sub.start(joint_states_topic);
    joint_states_sub.setActive(true);
}

bool GraspEligibilityChecker::checksJointStates()
{
    return false;
}
    
bool GraspEligibilityChecker::goalJointStatesConsistent(const GraspGoalT& graspGoal,
        const float useJointAnglesAccuracy) const
{
    const moveit_msgs::Grasp& mgrasp = graspGoal.grasp.grasp;
    const trajectory_msgs::JointTrajectory traj = graspGoal.grasp_trajectory;
    if (traj.points.empty())
    {
        ROS_ERROR("Trajectory for the grasp is empty");
        return false;
    }
    sensor_msgs::JointState lastTrajPoint;
    if (!ROSFunctions::getJointStateAt(traj.points.size()-1, traj, lastTrajPoint))
    {
        ROS_ERROR("Could not get last joint state of grasp trajectory");
        return false;
    }
    const trajectory_msgs::JointTrajectory& graspStateTraj =
        graspGoal.is_grasp ? mgrasp.grasp_posture : mgrasp.pre_grasp_posture;
    sensor_msgs::JointState graspState;
    int jsTrans = getStateFromTrajectory(graspStateTraj, graspState);
    if (jsTrans < 0)
    {
      ROS_ERROR("JointTrajectory points is empty in grasp goal posture. Cannot check eligibility.");
      return false;
    }
    if (jsTrans != 0)
    {
      ROS_WARN_STREAM("Only one JointTrajectory point supported for grasp, "
        << "given " << graspStateTraj.points.size() << ". Only using first "
        << "point and ignoring others.");
    }
    int ret = ROSFunctions::equalJointPositions(lastTrajPoint, graspState, useJointAnglesAccuracy);
    if (ret > 0) return true;

    if (ret == -2)
    {
        ROS_ERROR("Grasp pose joint state and last trajectory point do not intersect");
    }
    else if (ret == -3)
    {
        ROS_ERROR("Consistency: intersection of grasp state and last trajectory point not of same size");
    }
    else if (ret == -1)
    {
        ROS_ERROR("Last trajectory point and grasp pose are not similar enough");
    }
    return false; 
}

bool GraspEligibilityChecker::executionEligible(const GraspGoalT& graspGoal){

    const grasp_execution_msgs::GraspData& grasp = graspGoal.grasp;
    const moveit_msgs::Grasp& mgrasp = grasp.grasp;
    const geometry_msgs::PoseStamped& graspPose = mgrasp.grasp_pose;
    const std::string& effectorLinkName = grasp.effector_link_name;

    float usePosAcc=effectorPosAccuracy;
    float useOriAcc=effectorOriAccuracy;
    float useJointAcc=jointAnglesAccuracy;
    if (graspGoal.use_custom_tolerances)
    {
        usePosAcc = graspGoal.effector_pos_tolerance;
        useOriAcc = graspGoal.effector_angle_tolerance;
        useJointAcc = graspGoal.joint_angles_tolerance;
    }

    if (!goalJointStatesConsistent(graspGoal, useJointAcc))
    {
        ROS_ERROR("Joint states in trajectory and in grasp are not conform");
        return false;
    }

    if (graspGoal.is_grasp || !graspGoal.ignore_effector_pose_ungrasp){
        if (!this->graspExecutionEligible(grasp.effector_link_name,
            graspPose, usePosAcc, useOriAcc))
        {
            return false;			
        }
    }
    return true;
}


bool GraspEligibilityChecker::graspExecutionEligible(const std::string& effectorLinkFrame,
        const geometry_msgs::PoseStamped& graspPose, 
        const float& _effectorPosAccuracy,
        const float& _effectorOriAccuracy){

    bool printErrors=true;
    float maxWaitTransform=2.0;
    
    // check if the end effector is at the same pose as it is expected to be.
    // First, get the transform from the robot pose frame to the effector frame. This transform
    // corresponds to the absolute effector pose (expressed in the robot position frame).
    geometry_msgs::PoseStamped effectorPose;
    int transRet=ROSFunctions::Singleton()->getTransform(
            graspPose.header.frame_id, effectorLinkFrame,
            effectorPose.pose,
            ros::Time(0), maxWaitTransform, printErrors);
    if (transRet!=0) {
        ROS_ERROR_STREAM("GraspEligibilityChecker: Cannot get transform from the grasp pose frame ("
            <<graspPose.header.frame_id<<") to the effector link (" <<effectorLinkFrame
            <<"), so can't determine effector pose, and not do the grasp");
        return false;
    }

    effectorPose.header.stamp=ros::Time::now();
    effectorPose.header.frame_id=graspPose.header.frame_id;

    bool useLatest=false;
    if (ROSFunctions::Singleton()->equalPoses(
        graspPose, effectorPose,
        _effectorPosAccuracy,
        _effectorOriAccuracy,
        useLatest, maxWaitTransform,printErrors)<=0)
    {
        
        ROS_ERROR_STREAM("GraspEligibilityChecker: Can't execute the grasp if end effector "
                <<"is not at required target pose. Tolerances: "
                <<_effectorPosAccuracy <<", "<< _effectorOriAccuracy);
        ROS_INFO_STREAM(graspPose);
        ROS_INFO_STREAM(effectorPose);
        float poseDist=-1;
        float angleDist=-1;
        int ret=ROSFunctions::Singleton()->poseDistance(
            graspPose,effectorPose,poseDist, angleDist,false,1,true);
        if (ret!=0) ROS_ERROR("Could not even get pose distance.");
        ROS_INFO_STREAM("GraspEligibilityChecker: pose dist "<<poseDist<<", "<<angleDist);
        return false;
    }
    return true;
}
