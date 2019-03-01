#ifndef GRASP_EXECUTION_SIMPLEAUTOMATEDGRASPFROMFILE_H
#define GRASP_EXECUTION_SIMPLEAUTOMATEDGRASPFROMFILE_H

#include <grasp_execution/SimpleAutomatedGraspExecution.h>

namespace grasp_execution
{

/**
 * Reads a moveit_msgs/Grasp.msg from a file and uses this
 * as the result for grasp planning. Only minimal adaptations such as
 * object name are made to the message.
 *
 * \author Jennifer Buehler
 * \date March 2016
 */
class SimpleAutomatedGraspFromFile: public SimpleAutomatedGraspExecution
{
public:
    SimpleAutomatedGraspFromFile(const std::string& _filename);
    virtual ~SimpleAutomatedGraspFromFile();

protected:
    virtual bool initImpl();
    virtual bool getGrasp(const std::string& object_name, bool isGrasp, grasp_execution_msgs::GraspGoal& graspGoal);

private:
    std::string filename;
};

}  // namespace

#endif   // GRASP_EXECUTION_SIMPLEAUTOMATEDGRASPFROMFILE_H
