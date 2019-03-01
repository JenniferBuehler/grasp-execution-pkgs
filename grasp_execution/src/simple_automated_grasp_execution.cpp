#include <ros/ros.h>
#include <grasp_execution/SimpleAutomatedGraspFromTop.h>
#include <grasp_execution/SimpleAutomatedGraspFromFile.h>
#include <grasp_execution/SimpleAutomatedGraspOnlinePlanning.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grasp_action_client");

    ros::NodeHandle priv("~");
    std::string OBJECT_NAME;
    if (!priv.hasParam("object_name"))
    {
        ROS_ERROR("have to specify 'object_name' as ROS parameter");
        return 0;
    }
    priv.getParam("object_name",OBJECT_NAME);

    int RUN_TYPE;
    if (!priv.hasParam("run_type"))
    {
        ROS_ERROR("have to specify 'run_type' as ROS parameter");
        return 0;
    }
    priv.getParam("run_type",RUN_TYPE);
    
    std::string GRASP_FILENAME;
    priv.getParam("grasp_filename",GRASP_FILENAME);
    if ((RUN_TYPE==2) && (!priv.hasParam("grasp_filename") || GRASP_FILENAME.empty()))
    {
        {
            ROS_ERROR("run_type = 2 requires additional specification of 'grasp_filename'");
            return 0;
        }
    }

    std::vector<std::string> ROBOT_JOINT_NAMES;
    std::string ROBOT_NAME;
    std::string ROBOT_FILENAME;
    std::string OBJECT_FILENAME;
    std::string TABLE_FILENAME;
    std::string RESULTS_DIRECTORY;
    geometry_msgs::Pose OBJECT_POSE;

    if (RUN_TYPE==3)
    {
        priv.getParam("robot_name",ROBOT_NAME);
        if (!priv.hasParam("robot_name") || ROBOT_NAME.empty())
        {
            ROS_ERROR("run_type = 3 requires additional specification of 'robot_name'");
            return 0;
        }
         
        priv.getParam("robot_finger_joint_names",ROBOT_JOINT_NAMES);
        if (!priv.hasParam("robot_finger_joint_names") || ROBOT_JOINT_NAMES.empty())
        {
            ROS_ERROR("run_type = 3 requires additional specification of 'robot_finger_joint_names'");
            return 0;
        }
        ROS_INFO("Robot finger joint names: ");
        for (int i=0; i<ROBOT_JOINT_NAMES.size(); ++i) ROS_INFO_STREAM(ROBOT_JOINT_NAMES[i]);

        priv.getParam("robot_filename",ROBOT_FILENAME);
        if (!priv.hasParam("robot_filename") || ROBOT_FILENAME.empty())
        {
            ROS_ERROR("run_type = 3 requires additional specification of 'robot_filename'");
            return 0;
        }

        priv.getParam("object_filename",OBJECT_FILENAME);
        if (!priv.hasParam("object_filename") || OBJECT_FILENAME.empty())
        {
            ROS_ERROR("run_type = 3 requires additional specification of 'object_filename'");
            return 0;
        }

        priv.getParam("table_filename",TABLE_FILENAME);
        if (!priv.hasParam("table_filename") || TABLE_FILENAME.empty())
        {
            ROS_ERROR("run_type = 3 requires additional specification of 'table_filename'");
            return 0;
        }
    
        std::map<std::string,float> coords;
        priv.getParam("graspit_object_pose", coords);
        OBJECT_POSE.position.x = coords["x"];
        OBJECT_POSE.position.y = coords["y"];
        OBJECT_POSE.position.z = coords["z"];
        if (coords.find("qw")==coords.end())
        {
            OBJECT_POSE.orientation.x=0;
            OBJECT_POSE.orientation.y=0;
            OBJECT_POSE.orientation.z=0;
            OBJECT_POSE.orientation.w=1;
        }
        else
        {
            OBJECT_POSE.orientation.x = coords["qx"];
            OBJECT_POSE.orientation.y = coords["qy"];
            OBJECT_POSE.orientation.z = coords["qz"];
            OBJECT_POSE.orientation.w = coords["qw"];
        }
        ROS_INFO_STREAM("Putting object as pose: "<<OBJECT_POSE);

        priv.getParam("results_directory",RESULTS_DIRECTORY);
    }
 
    grasp_execution::SimpleAutomatedGraspExecution * graspExe;
    if (RUN_TYPE == 1) graspExe = new grasp_execution::SimpleAutomatedGraspFromTop();
    else if (RUN_TYPE == 2) graspExe = new grasp_execution::SimpleAutomatedGraspFromFile(GRASP_FILENAME);
    else if (RUN_TYPE == 3) 
        graspExe = new grasp_execution::SimpleAutomatedGraspOnlinePlanning(RESULTS_DIRECTORY,
                ROBOT_NAME, ROBOT_FILENAME, ROBOT_JOINT_NAMES, OBJECT_FILENAME, TABLE_FILENAME, OBJECT_POSE);
    
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
