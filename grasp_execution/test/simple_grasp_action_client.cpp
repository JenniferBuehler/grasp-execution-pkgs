#include <grasp_execution_msgs/GraspAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grasp_action_client");

    ros::NodeHandle priv("~");
    
    bool UNGRASP=true;
	priv.param<bool>("ungrasp_action", UNGRASP, UNGRASP);
    
    float TARGET_ANGLES=0.5;
	priv.param<float>("target_angles", TARGET_ANGLES, TARGET_ANGLES);

    std::string GRASP_ACTION_TOPIC = "/grasp_action";
	priv.param<std::string>("grasp_action_topic", GRASP_ACTION_TOPIC, GRASP_ACTION_TOPIC);

    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<grasp_execution_msgs::GraspAction> ac(GRASP_ACTION_TOPIC, true);

    ROS_INFO("Waiting for action server to start: %s", GRASP_ACTION_TOPIC.c_str());
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time
    ROS_INFO("Action server started.");

    /*
    sensor_msgs::JointState target;
    target.name = gripperJoints;
    for (int i=0; i<gripperJoints.size(); ++i)
    {
        target.position.push_back(TARGET_ANGLES);
    }

    ROS_INFO("Now constructing goal");

    // send a goal to the action
    //grasp_execution_msgs::JointTrajectoryActionGoal actionGoal;
    grasp_execution_msgs::GraspGoal goal;
    goal.target_joint_state = target;
    goal.closing = UNGRASP;
    // XXX trajectory not supported yet by server used for test
    goal.use_trajectory = false; 
    // goal.trajectory = trajectory;
*/

    ROS_INFO("Now sending goal");
    ac.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(15.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");

    return 0;
}
