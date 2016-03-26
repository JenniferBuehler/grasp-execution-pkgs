#include <grasp_execution/SimpleAutomatedGraspExecution.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grasp_action_client");

    grasp_execution::SimpleAutomatedGraspFromTop graspExe;
    if (!graspExe.init() || !graspExe.exeTest())
    {
        ROS_ERROR("Failed to run automated grasp execution");
    }
    return 0;
}
