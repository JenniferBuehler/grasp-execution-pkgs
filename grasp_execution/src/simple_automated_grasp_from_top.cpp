#include <grasp_execution/SimpleAutomatedGraspFromTop.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grasp_action_client");
    
    if (argc < 2)
    {
        ROS_ERROR("Have to pass object name as argument");
        return 0;
    }

    std::string OBJECT_NAME(argv[1]); 
    /*ros::NodeHandle priv("~");
    if (!priv.hasParam("object_name"))
    {
        ROS_ERROR("Object name required!");
        return false;
    }
    priv.param<std::string>("object_name", OBJECT_NAME, OBJECT_NAME);
    */

    grasp_execution::SimpleAutomatedGraspFromTop graspExe;
    if (!graspExe.init() || !graspExe.graspHomeAndUngrasp(OBJECT_NAME))
    {
        ROS_ERROR("Failed to run automated grasp execution");
    }

    return 0;
}
