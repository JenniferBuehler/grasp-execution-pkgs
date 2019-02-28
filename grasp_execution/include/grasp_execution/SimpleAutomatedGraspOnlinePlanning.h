#ifndef GRASP_EXECUTION_SIMPLEAUTOMATEDGRASPONLINEPLANNING_H
#define GRASP_EXECUTION_SIMPLEAUTOMATEDGRASPONLINEPLANNING_H

#include <vector>
#include <string>
#include <grasp_planning_graspit_ros/EigenGraspPlannerClient.h>
#include <grasp_execution/SimpleAutomatedGraspExecution.h>
#include <geometry_msgs/Pose.h>

namespace grasp_execution
{

/**
 * Uses the GraspIt! planner to do on-line planning before executing the grasp.
 * Only works for one type of object which will be placed on a table to allow
 * the planner to consider collisions with the support surface.
 * Only works if the object is on the support surface.
 *
 * \author Jennifer Buehler
 * \date March 2016
 */
class SimpleAutomatedGraspOnlinePlanning: public SimpleAutomatedGraspExecution
{
public:
    /**
     * \param _resultsDirectory Directory where to save the results of the on-line planning. Set to empty
     *      if results should not be saved.
     * \param _robotFilename the file with the GraspIt! robot definition
     * \param _objectFilename the file with the GraspIt! object definition
     * \param _objectPose the pose of the object such that it lies on the table. Has to be in the
     *      global GraspIt! reference frame.
     * \param _tableFilename the file with the GraspIt! definition of the table
     * \param _robotFingerJointNames the names of the URDF finger joint names of the robot,
     *      in the order they are specified in the graspit file \e _robotFilename.
     */
    SimpleAutomatedGraspOnlinePlanning(const std::string& _resultsDirectory,
        const std::string& _robotName,
        const std::string& _robotFilename,
        const std::vector<std::string>& _robotFingerJointNames,
        const std::string& _objectFilename,
        const std::string& _tableFilename,
        const geometry_msgs::Pose& _objectPose);

    virtual ~SimpleAutomatedGraspOnlinePlanning();

protected:
    virtual bool initImpl();
    virtual bool getGrasp(const std::string& object_name, bool isGrasp, grasp_execution_msgs::GraspGoal& graspGoal);

private:
    std::string resultsDirectory;
    std::string robotName;
    std::string robotFilename;
    std::vector<std::string> robotFingerJointNames;
    std::string objectFilename;
    std::string tableFilename;

    geometry_msgs::Pose objectPose;

    // only kept so it can be used for following un-grasp
    moveit_msgs::Grasp lastPlanResult;
    std::string lastPlanObject;

    int objectID;

    grasp_planning_graspit_ros::EigenGraspPlannerClient graspitClient;

};

}  // namespace

#endif   // GRASP_EXECUTION_SIMPLEAUTOMATEDGRASPONLINEPLANNING_H
