#include <grasp_execution/SimpleGraspControlServer.h>
#include <arm_components_name_manager/ArmComponentsNameManager.h>

#define DEFAULT_CHECK_FINGER_STATE_RATE 20
#define DEFAULT_NO_MOVE_TOLERANCE 0.05
#define DEFAULT_NO_MOVE_STILL_CNT 3 
#define DEFAULT_GOAL_TOLERANCE 5e-02 

/***
 * Starts up a SimpleGraspControlServer.
 * Launch this node with the following launch file (or include it in another launch file):
 *
 * `` \`rospack find grasp_execution\`/launch/simple_grasp_control_server.launch `` 
 *
 * Please also refer to this file (and the SimpleGraspControlServer header documentation)
 * for more details about the required parameters.
 *
 * \author Jennifer Buehler
 * \date March 2016
 */
int main(int argc, char**argv){
	ros::init(argc, argv, "simple_grasp_action");

	ros::NodeHandle priv("~");
	ros::NodeHandle pub;

	std::string JOINT_STATES_TOPIC="/joint_states";
	priv.param<std::string>("joint_states_topic", JOINT_STATES_TOPIC, JOINT_STATES_TOPIC);
	
    std::string JOINT_CONTROL_TOPIC="/joint_control";
	priv.param<std::string>("joint_control_topic", JOINT_CONTROL_TOPIC, JOINT_CONTROL_TOPIC);

	std::string GRASP_ACTION_TOPIC="/grasp_control_action";
	priv.param<std::string>("grasp_control_action_topic", GRASP_ACTION_TOPIC, GRASP_ACTION_TOPIC);
	
    std::string ROBOT_NAMESPACE;
	if (!priv.hasParam("robot_namespace"))
    {
        ROS_ERROR_STREAM(ros::this_node::getName()<<": Must have at least 'robot_namespace' defined in private node namespace");
        return 0;
    }
	priv.param<std::string>("robot_namespace", ROBOT_NAMESPACE, ROBOT_NAMESPACE);

	double CHECK_FINGER_STATE_RATE=DEFAULT_CHECK_FINGER_STATE_RATE;
	priv.param<double>("check_movement_rate", CHECK_FINGER_STATE_RATE, CHECK_FINGER_STATE_RATE);
	
	double NO_MOVE_TOLERANCE=DEFAULT_NO_MOVE_TOLERANCE;
	priv.param<double>("no_move_tolerance", NO_MOVE_TOLERANCE, NO_MOVE_TOLERANCE);
	
    int NO_MOVE_STILL_CNT=DEFAULT_NO_MOVE_STILL_CNT;
	priv.param<int>("no_move_still_cnt", NO_MOVE_STILL_CNT, NO_MOVE_STILL_CNT);

	double GOAL_TOLERANCE=DEFAULT_GOAL_TOLERANCE;
	priv.param<double>("goal_tolerance", GOAL_TOLERANCE, GOAL_TOLERANCE);

    ROS_INFO("Launching arm components name manager");
    arm_components_name_manager::ArmComponentsNameManager jointsManager(ROBOT_NAMESPACE, false);
    float maxWait=5;
    ROS_INFO("Waiting for joint info parameters to be loaded...");
    if (!jointsManager.waitToLoadParameters(1,maxWait,1))
    {
        ROS_ERROR("Joint names (ArmComponentsNameManager) could not be launched due to missing ROS parameters.");
        return 0;
    }

	grasp_execution::SimpleGraspControlServer actionServer(
        pub,
        GRASP_ACTION_TOPIC,
        JOINT_STATES_TOPIC, 
		JOINT_CONTROL_TOPIC,
        jointsManager,
        GOAL_TOLERANCE,
        NO_MOVE_TOLERANCE,
        NO_MOVE_STILL_CNT,
        CHECK_FINGER_STATE_RATE);

	actionServer.init();

    // ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    // spinner.spin(); // spin() will not return until the node has been shutdown
	ros::spin();
    return 0;
}
