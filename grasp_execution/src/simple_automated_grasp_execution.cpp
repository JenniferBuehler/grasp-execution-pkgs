#include <grasp_execution/SimpleAutomatedGraspFromTop.h>
#include <grasp_execution/SimpleAutomatedGraspFromFile.h>

void usage(const char * progname)
{
    ROS_ERROR_STREAM("Usage: "<<progname<<" <object_name> <run_type=1..3> [<filename>]");
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "grasp_action_client");
    
    if (argc < 3)
    {
        ROS_ERROR("Have to pass object name and run type as argument");
        usage(argv[0]);
        return 0;
    }

    std::string OBJECT_NAME(argv[1]); 
    int RUN_TYPE = atof(argv[2]);
    std::string FILE_NAME;
    if (RUN_TYPE==2)
    {
        if (argc < 4)
        {
            ROS_ERROR("run_type = 2 requires additional specification of filename");
            usage(argv[0]);
            return 0;
        }
        FILE_NAME=std::string(argv[3]); 
    }
    
    grasp_execution::SimpleAutomatedGraspExecution * graspExe;
    if (RUN_TYPE == 1) graspExe = new grasp_execution::SimpleAutomatedGraspFromTop();
    else if (RUN_TYPE == 2) graspExe = new grasp_execution::SimpleAutomatedGraspFromFile(FILE_NAME);
    
    if (!graspExe)
    {
        ROS_ERROR("Unknown run type");
        return 1;
    }

    if (!graspExe->init() || !graspExe->graspHomeAndUngrasp(OBJECT_NAME))
    {
        ROS_ERROR("Failed to run automated grasp execution");
    }

    return 0;
}
