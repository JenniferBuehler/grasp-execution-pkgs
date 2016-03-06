#include <grasp_execution/GraspEligibilityChecker.h>
#include <convenience_ros_functions/ROSFunctions.h>

using convenience_ros_functions::ROSFunctions;
using grasp_execution::GraspEligibilityChecker;

GraspEligibilityChecker::GraspEligibilityChecker(ros::NodeHandle& node,
    const std::string& _joint_states_topic,
    const float& _effectorPosAccuracy,
    const float& _effectorOriAccuracy):
    effectorPosAccuracy(_effectorPosAccuracy),
    effectorOriAccuracy(_effectorOriAccuracy),
    joint_states_sub(node)
{
    connectSubscriber(_joint_states_topic);
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

bool GraspEligibilityChecker::executionEligible(const GraspActionGoalT& graspAction){

    const grasp_execution_msgs::GraspData& grasp = graspAction.grasp;
    const manipulation_msgs::Grasp& mgrasp = grasp.grasp;
    const geometry_msgs::PoseStamped& graspPose = mgrasp.grasp_pose;
    const std::string& effectorLinkName = grasp.effector_link_name;

    float usePosAcc=effectorPosAccuracy;
    float useOriAcc=effectorOriAccuracy;
    if (graspAction.use_custom_tolerances)
    {
        usePosAcc = graspAction.effector_pos_tolerance;
        useOriAcc = graspAction.effector_angle_tolerance;
    }

    if (graspAction.is_grasp || !graspAction.ignore_effector_pose_ungrasp){
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
    
    //check if the end effector is at the same pose as it is expected to be.
    //First, get the transform from the robot pose frame to the effector frame. This transform
    //corresponds to the absolute effector pose (expressed in the robot position frame).
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
                <<" is not at required target pose. Tolerances: "
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
