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
  int noMoveStillCnt,
  float checkStateFreq):
    GraspControlActionServerT(n,action_topic_name),  
    GOAL_TOLERANCE(goalTolerance),
    NO_MOVE_TOLERANCE(noMoveTolerance),
    NO_MOVE_STILL_CNT(noMoveStillCnt),
    no_move_stat_all(0),
    gripper_angles_check_freq(checkStateFreq),
    gripper_check_thread(NULL),
    joints_manager(_joints_manager),  
    joint_state_subscriber(_joints_manager, n,joint_states_topic),
    initialized(false){
  
//  ROS_INFO_STREAM("SimpleGraspControlServer for action '"
//    << action_topic_name <<"': publishing joint control on " << joint_control_topic);
  joint_control_pub = n.advertise<sensor_msgs::JointState>(joint_control_topic, 1000, true);
}


SimpleGraspControlServer::~SimpleGraspControlServer() {
  shutdown();
}

void SimpleGraspControlServer::shutdownImpl(){
}

bool SimpleGraspControlServer::initImpl(){
  ROS_INFO("Initialising SimpleGraspControlServer.");
  initialized=true;
  return true;
}

void SimpleGraspControlServer::cancelGripperCheckThread()
{
  if (gripper_check_thread)
  {
    // ROS_INFO("Cancelling gripper check thread...");
    gripper_check_thread->detach();
    delete gripper_check_thread;
    gripper_check_thread = NULL;
    // ROS_INFO("Gripper check thread cancelled.");
  }
}


bool SimpleGraspControlServer::canAccept(const ActionGoalHandleT& goal)
{
  if (!initialized) {
  ROS_ERROR("Action server not initialised, can't accept goal");
  return false;
  }

  bool use_trajectory=goal.getGoal()->use_trajectory;
  if (use_trajectory) {
  ROS_ERROR("This simple implementation SimpleGraspControlServer does not \
      support a full JointTrajectory action yet (message parameter use_trajectory was true).");
  return false;
  }
  
  joint_state_subscriber.setActive(true);
  ROS_INFO("SimpleGraspControlServer: Waiting to obtain the current state of the gripper...");
  joint_state_subscriber.waitForUpdate();
  ROS_INFO("SimpleGraspControlServer: Obtained most recent joint state.");
  
  bool valid;
  last_gripper_angles=joint_state_subscriber.gripperAngles(valid);
  time_last_gripper_angles=ros::Time::now();
  if (!valid){
  ROS_ERROR("SimpleGraspControlServer: Could not get the robot's current gripper state");
    joint_state_subscriber.setActive(false);
  return false;
  }

  sensor_msgs::JointState goal_state=goal.getGoal()->target_joint_state;

  std::vector<float> _target_gripper_angles;
  if (!joints_manager.extractFromJointState(goal_state, 2, _target_gripper_angles, 0))
  {
    ROS_ERROR_STREAM("SimpleGraspControlServer: Not all gripper joints "
                     << "specified in target state " << goal_state);
    joint_state_subscriber.setActive(false);
    return false;
  }
  target_gripper_angles_mutex.lock();
  target_gripper_angles = _target_gripper_angles;
  target_gripper_angles_mutex.unlock();
  // ROS_INFO("SimpleGraspControlServer: Action can be accepted.");
  return true;
}

void SimpleGraspControlServer::actionCallbackImpl(const ActionGoalHandleT& goal)
{
  // ROS_INFO("SimpleGraspControlServer: Goal accepted");

  target_gripper_angles_mutex.lock();
  // for (int i=0; i<target_gripper_angles.size(); ++i) ROS_INFO_STREAM("Debug "<<i<<": "<<_target_gripper_angles[i]);

  // subset of target joint state only containing the
  // gripper joints (just to make sure possibly existent other joints are filtered out)
  sensor_msgs::JointState goal_state_subset;
  joints_manager.copyToJointState(goal_state_subset, 2, &target_gripper_angles, 0, true);
  target_gripper_angles_mutex.unlock();

  while (joint_control_pub.getNumSubscribers() == 0)
  {
    ROS_INFO("SimpleGraspControlServer: Waiting for subscribers for joint control...");
    ros::Duration(0.5).sleep();
  }

  // ROS_INFO_STREAM("Sending out goal JointState = "<<goal_state_subset);
  joint_control_pub.publish(goal_state_subset);

  no_move_stat.clear();
  no_move_stat.assign(joints_manager.numGripperJoints(), 0);
  no_move_stat_all = 0;
 
  move_stat.clear();
  move_stat.assign(joints_manager.numGripperJoints(), 0);
  
 
  // start the thread which continuously checks if the grippers are not moving any more  
  if (gripper_check_thread) cancelGripperCheckThread();
  gripper_check_thread = new baselib_binding::thread(updateGrippersCheckLoop, this, gripper_angles_check_freq);
}

void SimpleGraspControlServer::actionCancelCallbackImpl(ActionGoalHandleT& goal)
{
  cancelGripperCheckThread();
  joint_state_subscriber.setActive(false);
}


void SimpleGraspControlServer::updateGrippersCheckLoop(SimpleGraspControlServer * _this, float updateRate)
{
  float sleepTime = 1.0 / updateRate;
  while (true)
  {
    int check = _this->updateGrippersCheck();
    // on error, or if action finished, quit loop.
    if ((check < 0) || (check == 1))
    {
      // ROS_INFO("Grasp finished, exiting thread loop.");
      return;
    }
    ros::Duration(sleepTime).sleep();
  }
}

int SimpleGraspControlServer::updateGrippersCheck()
{
  ros::Time now=ros::Time::now();
  ros::Duration time_since_last_state = now-time_last_gripper_angles;

  // get current state
  bool valid=false;
  std::vector<float> gripper_angles=joint_state_subscriber.gripperAngles(valid);
  ros::Time time_gripper_angles=ros::Time::now();
  if (!valid)
  {
    ROS_ERROR("SimpleGraspControlServer: Could not get the robot's current gripper state");
    return -1;
  }

  if (gripper_angles.size() != last_gripper_angles.size())
  {
    ROS_ERROR("Inconsistency in SimpleGraspControlServer: Gripper states between calls are differently sized.");
    return -1;
  }
  
  if (gripper_angles.size() != no_move_stat.size())
  {
    ROS_INFO_STREAM("gripper_angles size: "<<gripper_angles.size());   
    ROS_ERROR("Inconsistency in SimpleGraspControlServer: Gripper states have to be same size as no_move_stat.");
    return -1;
  }
  
  if (move_stat.size() != no_move_stat.size())
  {
    ROS_ERROR("Inconsistency in SimpleGraspControlServer: move_stat has to be same size as no_move_stat.");
    return -1;
  }

  // see if the grippers are at target state
  target_gripper_angles_mutex.lock();
  if (MathFunctions::equalFloats(gripper_angles, target_gripper_angles, GOAL_TOLERANCE))
  {
    ROS_INFO("SimpleGraspControlServer: Hand at target state.");
    setExecutionFinished(true);
    target_gripper_angles_mutex.unlock();
    return 1;
  }  
  target_gripper_angles_mutex.unlock();
  
  // count which of the grippers have not moved since last time
  for (int i=0; i < gripper_angles.size(); ++i)
  {
    ROS_DEBUG_STREAM("Gripper move diff " << i << ": " <<
                     (gripper_angles[i]-last_gripper_angles[i])
                     << ", tolerance is " << NO_MOVE_TOLERANCE);
    float diff=fabs(gripper_angles[i]-last_gripper_angles[i]);
    if (diff < NO_MOVE_TOLERANCE)
    {
      move_stat[i] = 0;
      ++no_move_stat[i];
    }
    else
    {
      no_move_stat[i]=0;
      ++move_stat[i];
    }
  }

  bool allJointsStill = true;
  for (int i = 0; i < gripper_angles.size(); ++i)
  {
    if (no_move_stat[i] < NO_MOVE_STILL_CNT)
    {
      allJointsStill=false;
    }
  }
  
  bool finished = false;

  if (allJointsStill)
  {
    // at least one of the joints must have moved at least a bit.
    // Otherwise it may be that the hand hasn't moved at all since the goal was
    // accepted. Only exception: the object is already grasped and the hand
    // cannot reach its target state exactly because the object is blocking the fingers.
    // for this, the no_move_stat_all is used and then the action finishes with a warning.
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
      ++no_move_stat_all;
      if (no_move_stat_all >= NO_MOVE_STILL_CNT)
      {
        ROS_WARN_STREAM("SimpleGraspControlServer: Detected no gripper to have moved for a while, "
          <<"presuming hand cannot move and is already grasping an object, just not in its exact target state.");
        finished = true;
        setExecutionFinished(true);
      }
      else
      {
        ROS_INFO_STREAM("SimpleGraspControlServer: Detected no gripper to have moved, "
          <<"and hand is still but not in target state.");
      }
    }
    else
    {
      ROS_INFO("SimpleGraspControlServer: Hand considered still, stopping grip action.");
      /*ROS_INFO("Move counts: ");
      for (int i=0; i < no_move_stat.size(); ++i) ROS_INFO_STREAM(i<<": "<<no_move_stat[i]);
      */
      finished = true;
      setExecutionFinished(true);
    }
  }

  last_gripper_angles=gripper_angles;
  time_last_gripper_angles=time_gripper_angles;  
  return finished ? 1 : 0;
}

void SimpleGraspControlServer::setExecutionFinished(bool success){
  joint_state_subscriber.setActive(false);
  ResultT graspResult;
  graspResult.execution_time = ros::Duration(this->timeRunning());
  graspResult.success=success;
  currentActionDone(graspResult,actionlib::SimpleClientGoalState::SUCCEEDED);
}
