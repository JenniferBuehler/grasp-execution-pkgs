#include <grasp_execution/SimpleAutomatedGraspFromTop.h>
#include <grasp_execution/SimpleGraspGenerator.h>

using grasp_execution::SimpleAutomatedGraspFromTop;

SimpleAutomatedGraspFromTop::SimpleAutomatedGraspFromTop(): SimpleAutomatedGraspExecution() {}
SimpleAutomatedGraspFromTop::~SimpleAutomatedGraspFromTop(){}

bool SimpleAutomatedGraspFromTop::initImpl()
{
    ros::NodeHandle priv("~");
    priv.param<double>("pose_above_object", POSE_ABOVE, POSE_ABOVE);
    priv.param<double>("x_from_object", POSE_X, POSE_X);
    priv.param<double>("y_from_object", POSE_Y, POSE_Y);
    return true; 
}


bool SimpleAutomatedGraspFromTop::getGrasp(const std::string& object_name, bool isGrasp, grasp_execution_msgs::GraspGoal& graspGoal)
{
    /////////// Get object pose  ///////////////////
        
    geometry_msgs::PoseStamped obj_pose;
    float maxWaitObject=2; 
    if (!getObjectPose(object_name, maxWaitObject, obj_pose))
    {
        ROS_ERROR("Could not get pose of object %s", object_name.c_str());
        return false;
    }
    
    /////////// Generate grasp  ///////////////////

    std::string object_frame_id = object_name;

    std::vector<std::string> gripperJoints = jointsManager->getGripperJoints();
    std::string arm_base_link = jointsManager->getArmLinks().front();
    std::string effector_link = jointsManager->getEffectorLink();
    
    moveit_msgs::Grasp mgrasp;
    bool genGraspSuccess = grasp_execution::SimpleGraspGenerator::generateSimpleGraspFromTop(
        gripperJoints,
        arm_base_link,
        "TestGrasp",
        object_name,        
        obj_pose,
        object_frame_id,
        POSE_ABOVE, POSE_X, POSE_Y,
        OPEN_ANGLES, CLOSE_ANGLES,
        mgrasp);

    if (!genGraspSuccess)
    {
        ROS_ERROR("Could not generate grasp");
        return false;
    }

    // ROS_INFO_STREAM("generated moveit_msgs::Grasp: "<<std::endl<<mgrasp);
    grasp_execution::SimpleGraspGenerator::generateSimpleGraspGoal(effector_link,
        mgrasp,0, isGrasp, graspGoal);

    grasp_execution::SimpleGraspGenerator::useCustomTolerances(EFF_POS_TOL,
        EFF_ORI_TOL, JOINT_ANGLE_TOL, graspGoal);

    // ROS_INFO("################################");
    // ROS_INFO_STREAM("Generated grasp_execution_msgs::Grasp: "<<std::endl<<graspGoal);
    return true;
}    


