#include <grasp_execution/SimpleAutomatedGraspOnlinePlanning.h>
#include <grasp_execution/SimpleGraspGenerator.h>
#include <convenience_ros_functions/ROSFunctions.h>
#include <moveit_msgs/Grasp.h>

using grasp_execution::SimpleAutomatedGraspOnlinePlanning;

SimpleAutomatedGraspOnlinePlanning::SimpleAutomatedGraspOnlinePlanning(const std::string& _resultsDirectory,
        const std::string& _robotName,
        const std::string& _robotFilename,
        const std::vector<std::string>& _robotFingerJointNames,
        const std::string& _objectFilename,
        const std::string& _tableFilename,
        const geometry_msgs::Pose& _objectPose):
    SimpleAutomatedGraspExecution(),
    resultsDirectory(_resultsDirectory),
    robotName(_robotName), 
    robotFilename(_robotFilename), 
    robotFingerJointNames(_robotFingerJointNames), 
    objectFilename(_objectFilename),
    objectPose(_objectPose),
    objectID(-1),
    tableFilename(_tableFilename){}

SimpleAutomatedGraspOnlinePlanning::~SimpleAutomatedGraspOnlinePlanning(){}

bool SimpleAutomatedGraspOnlinePlanning::initImpl()
{
    if (!graspitClient.isOK())
    {
        ROS_ERROR("SimpleAutomatedGraspOnlinePlanning: EigenGraspPlannerClient not configured.");
        return false;
    }
    int robotID = graspitClient.addRobot(robotName, robotFilename, robotFingerJointNames); 
    if (robotID < 0)
    {
        ROS_ERROR_STREAM("SimpleAutoamtedGraspOnlinePlanning: Could not add robot "<<robotName<<" from file "<<robotFilename<<". return code: "<<robotID);
        return false;
    }

    objectID = graspitClient.addObject("AutomatedGraspObject", objectFilename, true); 
    if (objectID < 0)
    {
        ROS_ERROR_STREAM("SimpleAutoamtedGraspOnlinePlanning: Could not add object from file "<<objectFilename<<". return code: "<<objectID);
        return false;
    }

    int tableID = graspitClient.addObject("Table", tableFilename, false); 
    if (tableID < 0)
    {
        ROS_ERROR_STREAM("SimpleAutoamtedGraspOnlinePlanning: Could not add table from file "<<tableFilename<<". return code: "<<tableID);
        return false;
    }
   
    geometry_msgs::Pose identPose;
    identPose.orientation.w=1;
    // orientation of robot does not matter as it will be moved around during planning 
    int retRobLoad = graspitClient.loadModel(robotID, true, identPose);
    if (retRobLoad != 0) 
    {
        ROS_ERROR_STREAM("SimpleAutoamtedGraspOnlinePlanning: Could not load robot "<<robotName<<" into GraspIt! world. Return code: "<<robotID);
        return false;
    }

    int retObjLoad = graspitClient.loadModel(objectID, false, objectPose);
    if (retObjLoad != 0) 
    {
        ROS_ERROR_STREAM("SimpleAutoamtedGraspOnlinePlanning: Could not load object into GraspIt! world. Return code: "<<objectID);
        return false;
    }
    
    int retTableLoad = graspitClient.loadModel(tableID, false, identPose);
    if (retTableLoad != 0) 
    {
        ROS_ERROR_STREAM("SimpleAutoamtedGraspOnlinePlanning: Could not load table into GraspIt! world. Return code: "<<tableID);
        return false;
    }

    return true;
}


bool SimpleAutomatedGraspOnlinePlanning::getGrasp(const std::string& object_name, bool isGrasp, grasp_execution_msgs::GraspGoal& graspGoal)
{
    ROS_INFO_STREAM("SimpleAutomatedGraspOnlinePlanning: Grasping for object "
                    << object_name);
        
    moveit_msgs::Grasp grasp;

    if (isGrasp)
    {   // do whole grasp planning
        std::vector<moveit_msgs::Grasp> graspResults;
        int planRet = graspitClient.plan(robotName, objectID, NULL, resultsDirectory, graspResults);
        if (planRet != 0)
        {
            ROS_ERROR_STREAM("SimpleAutomatedGraspOnlinePlanning: Could not plan the grasp for object "<<object_name);
            return false;
        } 

        ROS_INFO_STREAM("SimpleAutomatedGraspOnlinePlanning: Finished planning, got "<<graspResults.size()<<" grasps."); 

        grasp = graspResults.front();
        grasp.grasp_pose.header.frame_id=object_name;
        ROS_INFO_STREAM("SimpleAutomatedGraspOnlinePlanning: Executing grasp: "<<grasp);
        lastPlanResult=grasp;
        lastPlanObject=object_name;
    }
    else  // is an  un-grasp, use the last grasp message
    {
        if (lastPlanObject.empty() || (lastPlanObject!=object_name))
        {
            ROS_ERROR_STREAM("Can only do un-grasp for '"<<object_name
                <<"' if a grasp was done for the same object before. Last object planned was '"
                <<lastPlanObject<<"'");
        }
        grasp=lastPlanResult;
    }
    // ROS_INFO_STREAM("generated moveit_msgs::Grasp: "<<std::endl<<mgrasp);
    std::string effector_link = jointsManager->getEffectorLink();
    grasp_execution::SimpleGraspGenerator::generateSimpleGraspGoal(effector_link,
        grasp,0, isGrasp, graspGoal);

    grasp_execution::SimpleGraspGenerator::useCustomTolerances(EFF_POS_TOL,
        EFF_ORI_TOL, JOINT_ANGLE_TOL, graspGoal);

    // ROS_INFO("################################");
    // ROS_INFO_STREAM("Generated grasp_execution_msgs::Grasp: "<<std::endl<<graspGoal);
    return true;
}    


