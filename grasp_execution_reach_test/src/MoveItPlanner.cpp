#include <grasp_execution_reach_test/MoveItPlanner.h>
#include <convenience_ros_functions/RobotInfo.h>


#define ALLOWED_PLANNING_TIME 3
#define ALLOWED_PLANNING_TIME_PATH_CONSTRAINTS 60
#define ALLOWED_PLANNING_ATTEMPTS 10

using grasp_execution_reach_test::MoveItPlanner;
using convenience_ros_functions::RobotInfo;
using convenience_ros_functions::ROSFunctions;


MoveItPlanner::MoveItPlanner(ros::NodeHandle& n,
    const std::string& _moveit_motion_plan_service,
    const std::string& _moveit_check_state_validity_service):
    node(n),
    moveit_motion_plan_service(_moveit_motion_plan_service),
    moveit_check_state_validity_service(_moveit_check_state_validity_service)
{
    init();
}

MoveItPlanner::~MoveItPlanner()
{
    shutdown();
}

bool MoveItPlanner::init(){
    ROS_INFO("Initialising MoveItPlanner");
    ROSFunctions::initSingleton();
    state_validity_client=node.serviceClient<moveit_msgs::GetStateValidity>(moveit_check_state_validity_service);
    motion_plan_client = node.serviceClient<moveit_msgs::GetMotionPlan>(moveit_motion_plan_service);
    return true;
}

void MoveItPlanner::shutdown(){
    motion_plan_client.shutdown();
    state_validity_client.shutdown();
    ROS_INFO("Shutting down MoveItPlanner.");
}


moveit_msgs::MoveItErrorCodes MoveItPlanner::requestTrajectory(
	const geometry_msgs::PoseStamped& robot_pose, 
	const geometry_msgs::PoseStamped& target_pose, 
	float armReachSpan,
    const std::string& planning_group, 
	const moveit_msgs::Constraints& goal_constraints,
    const moveit_msgs::Constraints * pathConstraints,  
	const sensor_msgs::JointState& startState,
    moveit_msgs::RobotTrajectory& resultTraj) {

	moveit_msgs::WorkspaceParameters wspace;
	if (!makeWorkspace(robot_pose,target_pose,armReachSpan,wspace)) {
		ROS_ERROR("Could not create MoveIt workspace");
		ROS_ERROR_STREAM("Start pose: "<<robot_pose);
		ROS_ERROR_STREAM("Target: "<<target_pose);
		moveit_msgs::MoveItErrorCodes ret;
		ret.val=moveit_msgs::MoveItErrorCodes::FAILURE;
		return ret;
	}

	//ROS_INFO("Made workspace: "); ROS_INFO_STREAM(wspace);

	std::vector<moveit_msgs::Constraints> constr;
	constr.push_back(goal_constraints);
	moveit_msgs::MoveItErrorCodes error_code=motionRequest(planning_group, constr, startState, wspace, NULL, pathConstraints,resultTraj);
	
	return error_code;
}


moveit_msgs::MoveItErrorCodes MoveItPlanner::requestTrajectoryForMobileRobot(
	const geometry_msgs::PoseStamped& start_pose, 
	const geometry_msgs::PoseStamped& target_pose, 
	float armReachSpan, const std::string& planning_group, 
	const moveit_msgs::Constraints& goal_constraints,
    const moveit_msgs::Constraints * pathConstraints,  
	const sensor_msgs::JointState& startState,
    const std::string& virtual_joint_name,
    moveit_msgs::RobotTrajectory& resultTraj) {

	moveit_msgs::WorkspaceParameters wspace;
	if (!makeWorkspace(start_pose,target_pose,armReachSpan,wspace)) {
		ROS_ERROR("Could not create MoveIt workspace");
		ROS_ERROR_STREAM("Start pose: "<<start_pose);
		ROS_ERROR_STREAM("Target: "<<target_pose);
		moveit_msgs::MoveItErrorCodes ret;
		ret.val=moveit_msgs::MoveItErrorCodes::FAILURE;
		return ret;
	}

	sensor_msgs::MultiDOFJointState * virtualJointState=NULL;
	if (!virtual_joint_name.empty()) {
		virtualJointState = new sensor_msgs::MultiDOFJointState();
		//XXX HACK: MoveIt likes "odom" as frame id, but can't transform, this could change later?
		std::string transformFrame="map"; //XXX Paramterezize? This depends on the robot????!!!
		ROS_INFO_STREAM("Transforming virtual joint pose to '"<<transformFrame<<"'");
		*virtualJointState=RobotInfo::getVirtualJointState(start_pose,virtual_joint_name,transformFrame); 
	}


	//ROS_INFO("Made workspace: "); ROS_INFO_STREAM(wspace);

	std::vector<moveit_msgs::Constraints> constr;
	constr.push_back(goal_constraints);
	moveit_msgs::MoveItErrorCodes error_code=motionRequest(planning_group, constr, startState, wspace, virtualJointState, pathConstraints,resultTraj);
	
	if (virtualJointState) delete virtualJointState;
	
	return error_code;
}


moveit_msgs::MoveItErrorCodes MoveItPlanner::motionRequest(const std::string& groupName, 
	const std::vector<moveit_msgs::Constraints>& goal_constraints, 
	const sensor_msgs::JointState& fromState, 
	const moveit_msgs::WorkspaceParameters& wspace,
	const sensor_msgs::MultiDOFJointState *mdjs,
    const moveit_msgs::Constraints* pathConstraints,
    moveit_msgs::RobotTrajectory& result){

	moveit_msgs::MotionPlanResponse response;

	//---- construct robot state
	moveit_msgs::RobotState robotState;
	robotState.is_diff=false;
	robotState.joint_state=fromState;

	if (mdjs) robotState.multi_dof_joint_state=*mdjs;

	//the actual motion plan will also do this check and return proper error code
	if (!isValidState(robotState,groupName)) {
		ROS_ERROR("MoveItPlanner: Invalid robot state, can't do planning");
		moveit_msgs::MoveItErrorCodes ret;
		ret.val=moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE;
		return ret;
	}


	//---- send request	
	
	moveit_msgs::MotionPlanRequest request;
	request.goal_constraints=goal_constraints;
	request.group_name=groupName;
	request.start_state=robotState;
	request.workspace_parameters=wspace;	
		
	request.planner_id="RRTConnectkConfigDefault";
	
	//request.trajectory_constraints=...
	request.num_planning_attempts=ALLOWED_PLANNING_ATTEMPTS;
	request.allowed_planning_time=ALLOWED_PLANNING_TIME;

	if (pathConstraints) {
		//ROS_WARN_STREAM("Path constraints "<<*pathConstraints);
		request.path_constraints=*pathConstraints;
		request.allowed_planning_time=ALLOWED_PLANNING_TIME_PATH_CONSTRAINTS;
	}

	//ROS_INFO_STREAM("Received final moveIt! request: "<<std::endl<<request);


	response=sendServiceRequest(request);

	if (response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
		result=response.trajectory;
	}/*else{
		ROS_ERROR("ReachPlan: Could not get Reaching plan from MoveIt. Error code=%i",response.error_code.val);
	}*/

	//ROS_INFO("trajectory size: %lu ", response.trajectory.joint_trajectory.points.size() );
	return response.error_code;
}


moveit_msgs::MotionPlanResponse MoveItPlanner::sendServiceRequest(moveit_msgs::MotionPlanRequest& request) {
	moveit_msgs::GetMotionPlan mp_srv;
	mp_srv.request.motion_plan_request = request;
	ROS_DEBUG("Sending planning request");
	if (!motion_plan_client.exists()) {
		ROS_ERROR("Service %s does not exist, can't get motion plan",motion_plan_client.getService().c_str());
		mp_srv.response.motion_plan_response.error_code.val=moveit_msgs::MoveItErrorCodes::FAILURE;
	}else{
		bool success=motion_plan_client.call(mp_srv);
		//if (!success) ROS_ERROR("Motion planning failure: error_code %i ", mp_srv.response.motion_plan_response.error_code.val); 
	}
	return mp_srv.response.motion_plan_response;
}



double pickMin(const double pos, const double rel, const double extend, const double minExtend=0.1){
	double m=std::min(pos,rel+pos);
	if (fabs(rel) < extend) return m-minExtend;
	return m-extend;
}

double pickMax(const double pos, const double rel, const double extend, const double minExtend=0.1){
	double m=std::max(pos,rel+pos);
	if (fabs(rel) < extend) return m+minExtend;
	return m+extend;
}


bool MoveItPlanner::makeWorkspace(const geometry_msgs::PoseStamped& from,
    const geometry_msgs::PoseStamped& to, 
	float armReachSpan, moveit_msgs::WorkspaceParameters& wspace){

	static const float maxWait=2.0;
	static const bool latestTime=false;
	static const bool printErrors=true;
	geometry_msgs::Pose rel;
	int err=ROSFunctions::Singleton()->relativePose(from,to,rel,latestTime,maxWait,printErrors);
	if (err < 0) {
		ROS_ERROR("Can't make MoveIt! workspace because transform between frames not possible. Error code %i",err);
		//ROS_ERROR_STREAM(from);
		//ROS_ERROR_STREAM(to);
		return false;
	}

	wspace.header=from.header;
	double minExt=0.5;
	wspace.min_corner.x= pickMin(from.pose.position.x, rel.position.x,armReachSpan,minExt);
	wspace.min_corner.y= pickMin(from.pose.position.y, rel.position.y,armReachSpan,minExt);
	wspace.min_corner.z= pickMin(from.pose.position.z, rel.position.z,armReachSpan,minExt);
	wspace.max_corner.x= pickMax(from.pose.position.x, rel.position.x,armReachSpan,minExt);
	wspace.max_corner.y= pickMax(from.pose.position.y, rel.position.y,armReachSpan,minExt);
	wspace.max_corner.z= pickMax(from.pose.position.z, rel.position.z,armReachSpan,minExt);

	return true;
}



bool MoveItPlanner::isValidState(const moveit_msgs::RobotState& robotState, const std::string& group) {
	//confirm valid robot state:
	moveit_msgs::GetStateValidity::Request get_state_validity_request;
	moveit_msgs::GetStateValidity::Response get_state_validity_response;
	get_state_validity_request.robot_state = robotState;
	get_state_validity_request.group_name = group;

	// Service client for checking state validity 
	ROS_DEBUG("checking state validity..");
	if (!state_validity_client.exists()) {
		ROS_ERROR("MoveIt! state valididty client not connected");
		return false;
	}
	state_validity_client.call(get_state_validity_request, get_state_validity_response);
	if(get_state_validity_response.valid) {
		//ROS_INFO("State was valid");
		//std::cout<<get_state_validity_response<<std::endl;
		//std::cout<<"-----------"<<std::endl;
		//std::cout<<robotState<<std::endl;
		return true;
		
	} else {
		ROS_ERROR("State was invalid");
		std::cout<<get_state_validity_response<<std::endl;
		//std::cout<<"-----------"<<std::endl;
		//std::cout<<robotState<<std::endl;
		return false;
	}
	return false;
}

