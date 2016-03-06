#include <grasp_execution/GraspEligibilityChecker.h>
//#include <grasp_execution/SimpleGraspActionServer.h>
#include <grasp_execution/GraspActionServer.h>

#define DEFAULT_EFFECTOR_POS_TOLERANCE 0.03
#define DEFAULT_EFFECTOR_ORI_TOLERANCE 0.05

/***
 * Starts up a SimpleGraspActionServer.
 * Launch this node with the following launch file (or include it in another launch file):
 *
 * `` \`rospack find grasp_execution\`/launch/simple_grasp_server.launch `` 
 *
 * Please also refer to this file (and the GraspActionServer and 
 * SimpleGraspActionServer header documentation)
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
	
   	std::string GRASP_ACTION_TOPIC="/grasp_action";
	priv.param<std::string>("grasp_action_topic", GRASP_ACTION_TOPIC, GRASP_ACTION_TOPIC);

    std::string JOINT_CONTROL_TOPIC="/joint_control";
	priv.param<std::string>("joint_control_topic", JOINT_CONTROL_TOPIC, JOINT_CONTROL_TOPIC);

	std::string GRASP_CONTROL_ACTION_TOPIC="/grasp_control";
	priv.param<std::string>("grasp_control_action_topic", GRASP_CONTROL_ACTION_TOPIC, GRASP_CONTROL_ACTION_TOPIC);
	
	double EFFECTOR_POS_TOLERANCE=DEFAULT_EFFECTOR_POS_TOLERANCE;
	priv.param<double>("effector_pos_tolerance", EFFECTOR_POS_TOLERANCE, EFFECTOR_POS_TOLERANCE);
	
    double EFFECTOR_ORI_TOLERANCE=DEFAULT_EFFECTOR_ORI_TOLERANCE;
	priv.param<double>("effector_pos_tolerance", EFFECTOR_ORI_TOLERANCE, EFFECTOR_ORI_TOLERANCE);


    
	grasp_execution::GraspEligibilityChecker * _eligibilityChecker = new grasp_execution::GraspEligibilityChecker(
        pub,
        JOINT_STATES_TOPIC, 
        EFFECTOR_POS_TOLERANCE,
        EFFECTOR_ORI_TOLERANCE);

    typedef architecture_binding::shared_ptr<grasp_execution::GraspEligibilityChecker>::type GraspEligibilityCheckerPtr;
	GraspEligibilityCheckerPtr eligibilityChecker(_eligibilityChecker);

	grasp_execution::GraspActionServer actionServer(
        pub,
        GRASP_ACTION_TOPIC,
        GRASP_CONTROL_ACTION_TOPIC,
        eligibilityChecker);

	actionServer.init();

    // ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    // spinner.spin(); // spin() will not return until the node has been shutdown
	ros::spin();
    return 0;
}
