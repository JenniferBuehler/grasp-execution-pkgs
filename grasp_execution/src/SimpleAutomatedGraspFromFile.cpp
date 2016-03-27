#include <grasp_execution/SimpleAutomatedGraspFromFile.h>
#include <grasp_execution/SimpleGraspGenerator.h>
#include <convenience_ros_functions/ROSFunctions.h>
#include <manipulation_msgs/Grasp.h>

using grasp_execution::SimpleAutomatedGraspFromFile;

SimpleAutomatedGraspFromFile::SimpleAutomatedGraspFromFile(const std::string& _filename):
    SimpleAutomatedGraspExecution(),
    filename(_filename) {}
SimpleAutomatedGraspFromFile::~SimpleAutomatedGraspFromFile(){}

bool SimpleAutomatedGraspFromFile::initImpl()
{
    return true; 
}


bool SimpleAutomatedGraspFromFile::getGrasp(const std::string& object_name, bool isGrasp, grasp_execution_msgs::GraspGoal& graspGoal)
{

    manipulation_msgs::Grasp grasp;
    if (!ROSFunctions::readFromFile(filename,grasp,true))
    {
        ROS_ERROR_STREAM("Could not read grasp from file "<<filename);
        return false;
    }

    grasp.grasp_pose.header.frame_id=object_name;
    grasp.grasp_pose.pose.position.z+=0.05;
    //float tmpX=grasp.grasp_pose.pose.position.x;
    //grasp.grasp_pose.pose.position.x=grasp.grasp_pose.pose.position.y;
    //grasp.grasp_pose.pose.position.y=tmpX;
    
    ROS_INFO_STREAM("Read grasp from file: "<<grasp);

    // ROS_INFO_STREAM("generated manipulation_msgs::Grasp: "<<std::endl<<mgrasp);
    std::string effector_link = jointsManager->getEffectorLink();
    grasp_execution::SimpleGraspGenerator::generateSimpleGraspGoal(effector_link,
        grasp,0, isGrasp, graspGoal);

    grasp_execution::SimpleGraspGenerator::useCustomTolerances(EFF_POS_TOL,
        EFF_ORI_TOL, JOINT_ANGLE_TOL, graspGoal);

    // ROS_INFO("################################");
    // ROS_INFO_STREAM("Generated grasp_execution_msgs::Grasp: "<<std::endl<<graspGoal);

    return true;
}    


