#include <arm_components_name_manager/ArmComponentsNameManager.h>

#include <grasp_execution/SimpleGraspControlServer.h>
#include <convenience_math_functions/MathFunctions.h>

using grasp_execution::SimpleGraspControlServer;
using convenience_math_functions::MathFunctions;
using arm_components_name_manager::ArmComponentsNameManager;
 
SimpleGraspControlServer::SimpleGraspControlServer(
		ros::NodeHandle &n, 
		std::string& action_topic_name,
		std::string& joint_states_topic,
		std::string& joint_control_topic,
        const ArmComponentsNameManager& _joints_manager,
		float goalTolerance,
		float noMoveTolerance,
		float checkStateFreq):
	GOAL_TOLERANCE(goalTolerance),
	NO_MOVE_TOLERANCE(noMoveTolerance),
	gripper_angles_check_freq(checkStateFreq),
    gripper_check_thread(NULL),
    joints_manager(_joints_manager),    
	joint_state_subscriber(_joints_manager, n,joint_states_topic),
	has_goal(false), 
	execution_finished(false),
	execution_successful(false),
	initialized(false){
    
    ROS_INFO_STREAM("SimpleGraspControlServer publishing joint control on " << joint_control_topic);
    joint_control_pub = n.advertise<sensor_msgs::JointState>(joint_control_topic, 1000, true);

    action_server= new GraspControlActionServerT(n, 
            action_topic_name,
            boost::bind(&SimpleGraspControlServer::actionCallback, this, _1),
            boost::bind(&SimpleGraspControlServer::cancelCallback, this, _1),
            false);
}


SimpleGraspControlServer::~SimpleGraspControlServer() {
	shutdown();
	delete action_server; 
}

void SimpleGraspControlServer::shutdown(){
}

bool SimpleGraspControlServer::init(){
	ROS_INFO("Initialising SimpleGraspControlServer. Starting the action server.");
	action_server->start();
	ROS_INFO("SimpleGraspControlServer: action server started.");
	initialized=true;
	return true;
}

void SimpleGraspControlServer::cancelCallback(GoalHandle& goal) {
	ROS_INFO("GripperControlActionServer: Received action cancel request");
	unique_lock lock(goal_lock);
	bool currentGoal = (goal == current_goal);
	if (currentGoal) 
		abortExecution();
	else ROS_ERROR("This goal is not currently being executed in this server");
}


void SimpleGraspControlServer::actionCallback(GoalHandle& goal) {
	ROS_INFO("Received gripping goal");
	if (!initialized) {
		ROS_ERROR("Action server not initialised, can't accept goal");
		goal.setRejected();
		return;
	}
	

	if (hasCurrentGoal()){
		ROS_ERROR("Currently executiong a grip, can't accept another one");
		goal.setRejected();
		return;
	}

	bool use_trajectory=goal.getGoal()->use_trajectory;
	if (use_trajectory) {
		ROS_ERROR("This simple implementation SimpleGraspControlServer does not \
            support a full JointTrajectory action yet (message parameter use_trajectory was true).");
		goal.setRejected();
		return;
	}
	
	bool valid;
	last_gripper_angles=joint_state_subscriber.gripperAngles(valid);
	time_last_gripper_angles=ros::Time::now();
	if (!valid){
		ROS_ERROR("SimpleGraspControlServer: Could not get the robot's current gripper state");
		goal.setRejected();
		return;
	}

	sensor_msgs::JointState goal_state=goal.getGoal()->target_joint_state;

	std::vector<float> _target_gripper_angles;
    if (!joints_manager.extractFromJointState(goal_state, 2, _target_gripper_angles))
    {
		ROS_ERROR("SimpleGraspControlServer: Not all gripper joints specified in target state.");
        goal.setRejected();
        return;
    }
	
    ROS_INFO("SimpleGraspControlServer: Goal accepted");
    for (int i=0; i<_target_gripper_angles.size(); ++i) ROS_INFO_STREAM("Debug "<<i<<": "<<_target_gripper_angles[i]);

    // subset of target joint state only containing the
    // gripper joints (just to make sure possibly existent other joints are filtered out)
    sensor_msgs::JointState goal_state_subset;
    joints_manager.copyToJointState(goal_state_subset, 2, &_target_gripper_angles);

    ROS_INFO_STREAM("Debug: "<<goal_state_subset);
	
	goal_lock.lock();
	current_goal=goal;
	current_goal.setAccepted();
	has_goal = true;
	target_gripper_angles = _target_gripper_angles;
	goal_lock.unlock();

	setExecutionFinished(false,false);

    while (joint_control_pub.getNumSubscribers() == 0)
    {
        ROS_INFO("SimpleGraspControlServer: Waiting for subscribers for joint control...");
        ros::Duration(0.5).sleep();
    }

    joint_control_pub.publish(goal_state_subset);

    no_move_stat.clear();
    no_move_stat.assign(joints_manager.numGripperJoints(), 0);
    
    move_stat.clear();
    move_stat.assign(joints_manager.numGripperJoints(), 0);

    joint_state_subscriber.setActive(true);
 
    // start the thread which continuously checks if the grippers are not moving any more	
    if (gripper_check_thread) cancelGripperCheckThread();
    gripper_check_thread = new architecture_binding::thread(updateGrippersCheckLoop, this, gripper_angles_check_freq);
     
}

void SimpleGraspControlServer::cancelGripperCheckThread()
{
    if (gripper_check_thread)
    {
        ROS_INFO("Cancelling gripper check thread...");
        gripper_check_thread->detach();
        delete gripper_check_thread;
        gripper_check_thread = NULL;
        ROS_INFO("Gripper check thread cancelled.");
    }
}

void SimpleGraspControlServer::abortExecution(){
	setExecutionFinished(true,false);
	goal_lock.lock();
	current_goal.setAborted();
	has_goal = false;
	goal_lock.unlock();
    cancelGripperCheckThread();
    joint_state_subscriber.setActive(false);
}

void SimpleGraspControlServer::updateGrippersCheckLoop(SimpleGraspControlServer * _this, float updateRate)
{
    float sleepTime = 1.0 / updateRate;
    while (true)
    {
        ROS_INFO("Debug: calling updateGrippersCheck()");

        int check = _this->updateGrippersCheck();
        // on error, or if action finished, quit loop.
        if ((check < 0) == (check == 1))
        {
            ROS_INFO("Grasp finished, exiting thread loop.");
            return;
        }
        ros::Duration(sleepTime).sleep();
    }
}

int SimpleGraspControlServer::updateGrippersCheck() {

	ros::Time now=ros::Time::now();
	ros::Duration time_since_last_state = now-time_last_gripper_angles;


    // get current state
    bool valid=false;
	std::vector<float> gripper_angles=joint_state_subscriber.gripperAngles(valid);
	ros::Time time_gripper_angles=ros::Time::now();
	if (!valid){
		ROS_ERROR("SimpleGraspControlServer: Could not get the robot's current gripper state");
		return -1;
	}

    if (gripper_angles.size() != last_gripper_angles.size())
    {
        ROS_ERROR("Inconcistency in SimpleGraspControlServer: Gripper states between calls are differently sized.");
        return -1;
    }
    
    if (gripper_angles.size() != no_move_stat.size())
    {
        ROS_ERROR("Inconcistency in SimpleGraspControlServer: Gripper states have to be same size as no_move_stat.");
        return -1;
    }
    
    if (move_stat.size() != no_move_stat.size())
    {
        ROS_ERROR("Inconcistency in SimpleGraspControlServer: move_stat has to be same size as no_move_stat.");
        return -1;
    }

    // see if the grippers are at target state
    if (MathFunctions::equalFloats(gripper_angles, target_gripper_angles, GOAL_TOLERANCE))
    {
        ROS_INFO("SimpleGraspControlServer: Hand at target state.");
        setExecutionFinished(true,true);
        return 1;
    }    

	if (time_since_last_state.toSec() < 1.0/gripper_angles_check_freq) {
        //no need to record last gripper state or to check if a gripper stopped moving
		return 0;
	}
    
	//count which of the grippers have not moved since last time
    for (int i=0; i<gripper_angles.size(); ++i)
    {
	    float diff=fabs(gripper_angles[i]-last_gripper_angles[i]);
	    if (diff < NO_MOVE_TOLERANCE)
        {
            ++no_move_stat[i];
        }
        else
        {
            no_move_stat[i]=0;
            ++move_stat[i];
        }
    }

	static const int no_moves_still=1;
    bool allJointsStill = true;
    for (int i=0; i<gripper_angles.size(); ++i)
    {
	    if (no_move_stat[i] < no_moves_still)
        {
            allJointsStill=false;
        }
	}
		
    bool finished = false;

    if (allJointsStill)
    {
        // at least one of the joints must have moved at least a bit.
        // Otherwise it may be that the hand hasn't moved at all since the goal was
        // accepted.
        bool oneMoved=false;
        for (int i=0; i < move_stat.size(); ++i)
        {
                if (move_stat[i] > 0)
                {
                    oneMoved=true;
                    break;
                }
        }
        if (!oneMoved)
        {
            ROS_WARN_STREAM("SimpleGraspControlServer: Detected no gripper to have moved at all, \
                and hand is still. Are topics properly connected, or is the update rate very high?");
        }
        else
        {
            ROS_INFO("SimpleGraspControlServer: Hand considered still, stopping grip action. Move counts: ");
            for (int i=0; i < no_move_stat.size(); ++i) ROS_INFO_STREAM(i<<": "<<no_move_stat[i]);
            finished = true;
            setExecutionFinished(true,true);
        }
    }

	last_gripper_angles=gripper_angles;
    time_last_gripper_angles=time_gripper_angles;	
    return finished ? 1 : 0;
}

bool SimpleGraspControlServer::executingGoal(){
	goal_lock.lock();
	bool hasOneGoal=has_goal;
	bool cancelled=false;
	if (hasOneGoal) {
		actionlib_msgs::GoalStatus stat=current_goal.getGoalStatus();
		cancelled= (stat.status != actionlib_msgs::GoalStatus::ACTIVE);
			//(stat.status == actionlib_msgs::GoalStatus::PREEMPTED) 
			//|| (stat.status == actionlib_msgs::GoalStatus::ABORTED)
			//|| (stat.status == actionlib_msgs::GoalStatus::LOST);
	}
	goal_lock.unlock();
	return ros::ok() && !cancelled && hasOneGoal;
}

bool SimpleGraspControlServer::hasCurrentGoal(){
	unique_lock lock(goal_lock);
	return has_goal;
}


void SimpleGraspControlServer::setExecutionFinished(bool flag, bool success){
	unique_lock lock(goal_lock);
	execution_finished=flag;
	execution_successful=success;
	if (flag && has_goal) {
		if (success) current_goal.setSucceeded();
		else current_goal.setAborted();
		has_goal=false;
	}
}

bool SimpleGraspControlServer::executionFinished(bool& success){
	unique_lock lock(goal_lock);
	bool ret=execution_finished;
	success=execution_successful;
	return ret;
}
