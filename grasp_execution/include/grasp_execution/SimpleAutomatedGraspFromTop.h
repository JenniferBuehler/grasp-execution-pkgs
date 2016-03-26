#ifndef GRASP_EXECUTION_SIMPLEAUTOMATEDGRASPFROMTOP_H
#define GRASP_EXECUTION_SIMPLEAUTOMATEDGRASPFROMTOP_H

#include <grasp_execution/SimpleAutomatedGraspExecution.h>

namespace grasp_execution
{

/**
 * Generates a simple grasp from top (using grasp_execution::SimpleGraspGenerator).
 * In addition to the ROS parameters of base class, also takes following
 * ROS Parameters:
 *
 * ``
 *   # end effector to be positioned this much above object (z-direction)
 *   pose_above_object: 0.17
 *   # end effector to be positioned relative to object (x-direction)
 *   x_from_object: 0.0
 *
 *   # end effector to be positioned relative to object (y-direction)
 *   y_from_object: -0.02
 * ``
 */
class SimpleAutomatedGraspFromTop: public SimpleAutomatedGraspExecution
{
public:
    SimpleAutomatedGraspFromTop();
    virtual ~SimpleAutomatedGraspFromTop();


protected:
    virtual bool initImpl(); 
    virtual bool getGrasp(const std::string& object_name, bool isGrasp, grasp_execution_msgs::GraspGoal& graspGoal);

private:
    
    double POSE_ABOVE;
    double POSE_X;
    double POSE_Y;
};

}  // namespace

#endif   // GRASP_EXECUTION_SIMPLEAUTOMATEDGRASPFROMTOP_H
