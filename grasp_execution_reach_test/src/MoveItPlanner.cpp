#include <grasp_execution_reach_test/MoveItPlanner.h>
#include <convenience_ros_functions/RobotInfo.h>
#include <shape_tools/solid_primitive_dims.h>

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
	const geometry_msgs::PoseStamped& arm_base_pose, 
	float arm_reach_span,
    const std::string& planning_group, 
	const moveit_msgs::Constraints& goal_constraints,
    const moveit_msgs::Constraints * path_constraints,  
	const sensor_msgs::JointState& start_state,
    moveit_msgs::RobotTrajectory& result_traj) {

	moveit_msgs::WorkspaceParameters wspace;
	if (!makeWorkspace(arm_base_pose,arm_reach_span,wspace)) {
		ROS_ERROR("Could not create MoveIt workspace");
		ROS_ERROR_STREAM("Origin pose: "<<arm_base_pose);
		ROS_ERROR_STREAM("Arm reach span: "<<arm_reach_span);
		moveit_msgs::MoveItErrorCodes ret;
		ret.val=moveit_msgs::MoveItErrorCodes::FAILURE;
		return ret;
	}
	// ROS_INFO_STREAM("Using workspace: "<<wspace);
    // ROS_INFO_STREAM("Requesting trajectory from state: "<<start_state);

	std::vector<moveit_msgs::Constraints> constr;
	constr.push_back(goal_constraints);
	moveit_msgs::MoveItErrorCodes error_code=requestTrajectory(planning_group, constr, start_state, wspace, NULL, path_constraints,result_traj);
	
	return error_code;
}


moveit_msgs::MoveItErrorCodes MoveItPlanner::requestTrajectoryForMobileRobot(
	const geometry_msgs::PoseStamped& start_pose, 
	const geometry_msgs::PoseStamped& target_pose, 
	float arm_reach_span, const std::string& planning_group, 
	const moveit_msgs::Constraints& goal_constraints,
    const moveit_msgs::Constraints * path_constraints,  
	const sensor_msgs::JointState& start_state,
    const std::string& virtual_joint_name,
    const std::string& target_frame_virtual_joint,
    moveit_msgs::RobotTrajectory& result_traj) {

	moveit_msgs::WorkspaceParameters wspace;
	if (!makeWorkspace(start_pose,target_pose,arm_reach_span,wspace)) {
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
		ROS_INFO_STREAM("Transforming virtual joint pose to '"<<target_frame_virtual_joint<<"'");
		*virtualJointState=RobotInfo::getVirtualJointState(start_pose, virtual_joint_name, target_frame_virtual_joint); 
	}


	//ROS_INFO("Made workspace: "); ROS_INFO_STREAM(wspace);

	std::vector<moveit_msgs::Constraints> constr;
	constr.push_back(goal_constraints);
	moveit_msgs::MoveItErrorCodes error_code=requestTrajectory(planning_group, constr, start_state, wspace, virtualJointState, path_constraints,result_traj);
	
	if (virtualJointState) delete virtualJointState;
	
	return error_code;
}


moveit_msgs::MoveItErrorCodes MoveItPlanner::requestTrajectory(const std::string& group_name, 
	const std::vector<moveit_msgs::Constraints>& goal_constraints, 
	const sensor_msgs::JointState& from_state, 
	const moveit_msgs::WorkspaceParameters& wspace,
	const sensor_msgs::MultiDOFJointState *mdjs,
    const moveit_msgs::Constraints* path_constraints,
    moveit_msgs::RobotTrajectory& result){

	moveit_msgs::MotionPlanResponse response;

	//---- construct robot state
	moveit_msgs::RobotState robotState;
	robotState.is_diff=false;
	robotState.joint_state=from_state;

	if (mdjs) robotState.multi_dof_joint_state=*mdjs;

	//the actual motion plan will also do this check and return proper error code
	if (!isValidState(robotState,group_name)) {
		ROS_ERROR("MoveItPlanner: Invalid robot state, can't do planning");
		moveit_msgs::MoveItErrorCodes ret;
		ret.val=moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE;
		return ret;
	}


	//---- send request	
	
	moveit_msgs::MotionPlanRequest request;
	request.goal_constraints=goal_constraints;
	request.group_name=group_name;
	request.start_state=robotState;
	request.workspace_parameters=wspace;	
		
	request.planner_id="RRTConnectkConfigDefault";
	
	//request.trajectory_constraints=...
	request.num_planning_attempts=ALLOWED_PLANNING_ATTEMPTS;
	request.allowed_planning_time=ALLOWED_PLANNING_TIME;

	if (path_constraints) {
		//ROS_WARN_STREAM("Path constraints "<<*path_constraints);
		request.path_constraints=*path_constraints;
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

double pickMin(const double pos, const double rel, const double extend){
	double m=std::min(pos,rel+pos);
	return m-extend;
}

double pickMax(const double pos, const double rel, const double extend){
	double m=std::max(pos,rel+pos);
	return m+extend;
}


bool MoveItPlanner::makeWorkspace(const geometry_msgs::PoseStamped& from,
    const geometry_msgs::PoseStamped& to, 
	float arm_reach_span, moveit_msgs::WorkspaceParameters& wspace){

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
	wspace.min_corner.x= pickMin(from.pose.position.x, rel.position.x,arm_reach_span);
	wspace.min_corner.y= pickMin(from.pose.position.y, rel.position.y,arm_reach_span);
	wspace.min_corner.z= pickMin(from.pose.position.z, rel.position.z,arm_reach_span);
	wspace.max_corner.x= pickMax(from.pose.position.x, rel.position.x,arm_reach_span);
	wspace.max_corner.y= pickMax(from.pose.position.y, rel.position.y,arm_reach_span);
	wspace.max_corner.z= pickMax(from.pose.position.z, rel.position.z,arm_reach_span);
	return true;
}

bool MoveItPlanner::makeWorkspace(const geometry_msgs::PoseStamped& from,
	float arm_reach_span, 
    moveit_msgs::WorkspaceParameters& wspace){

	wspace.header=from.header;
	wspace.min_corner.x= from.pose.position.x-fabs(arm_reach_span);
	wspace.min_corner.y= from.pose.position.y-fabs(arm_reach_span);
	wspace.min_corner.z= from.pose.position.z-fabs(arm_reach_span);
	wspace.max_corner.x= from.pose.position.x+fabs(arm_reach_span);
	wspace.max_corner.y= from.pose.position.y+fabs(arm_reach_span);
	wspace.max_corner.z= from.pose.position.z+fabs(arm_reach_span);
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




shape_msgs::SolidPrimitive getCone(const double& height, const double& radius){
	shape_msgs::SolidPrimitive bv;
	bv.type = shape_msgs::SolidPrimitive::CONE;
	bv.dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CONE>::value);
	bv.dimensions[shape_msgs::SolidPrimitive::CONE_HEIGHT] = height;
	bv.dimensions[shape_msgs::SolidPrimitive::CONE_RADIUS] = radius;
	return bv;

}




shape_msgs::SolidPrimitive getCylinder(const double& height, const double& radius){
	shape_msgs::SolidPrimitive bv;
	bv.type = shape_msgs::SolidPrimitive::CYLINDER;
	bv.dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
	bv.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = height;
	bv.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = radius;
	return bv;

}


shape_msgs::SolidPrimitive getBox(const double& x,const  double& y,const  double& z){
	shape_msgs::SolidPrimitive bv;
	bv.type = shape_msgs::SolidPrimitive::BOX;
	bv.dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
	bv.dimensions[shape_msgs::SolidPrimitive::BOX_X] = x;
	bv.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = y;
	bv.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = z;
	return bv;

}
shape_msgs::SolidPrimitive getSphere(const double& radius){
	shape_msgs::SolidPrimitive bv;
	bv.type = shape_msgs::SolidPrimitive::SPHERE;
	bv.dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::SPHERE>::value);
	bv.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = radius;
	return bv;
}


moveit_msgs::PositionConstraint MoveItPlanner::getBoxConstraint(const std::string &link_name,
        const geometry_msgs::PoseStamped& boxOrigin, const double& x, const double& y, const double& z) {
	
	moveit_msgs::PositionConstraint pc;
	pc.link_name = link_name;
	pc.target_point_offset.x = 0;
	pc.target_point_offset.y = 0;
	pc.target_point_offset.z = 0;
	
	pc.header=boxOrigin.header;
	
	pc.constraint_region.primitives.resize(1);
	shape_msgs::SolidPrimitive &bv = pc.constraint_region.primitives[0];
	bv=getBox(x,y,z);

	pc.constraint_region.primitive_poses.resize(1);
	geometry_msgs::Pose& pose=pc.constraint_region.primitive_poses[0];
	pose=boxOrigin.pose;	

	pc.weight=1.0;

	return pc;
}


moveit_msgs::Constraints MoveItPlanner::getJointConstraint(const std::string& link_name,
    const sensor_msgs::JointState& js, const float& joint_tolerance){
	moveit_msgs::Constraints c;
	for (int i=0; i<js.name.size(); ++i){
		moveit_msgs::JointConstraint jc;
		jc.joint_name=js.name[i];
		jc.position=js.position[i];
		jc.tolerance_above=joint_tolerance;
		jc.tolerance_below=joint_tolerance;
		jc.weight=1.0;
		c.joint_constraints.push_back(jc);
	}
	return c;

}


moveit_msgs::OrientationConstraint MoveItPlanner::getOrientationConstraint(const std::string& link_name,
    const geometry_msgs::QuaternionStamped& quat, 
	const float& x_tolerance,
	const float& y_tolerance,
	const float& z_tolerance) { 
	moveit_msgs::OrientationConstraint ocm;
	ocm.link_name = link_name;
	ocm.header = quat.header;
	ocm.orientation = quat.quaternion;
	ocm.absolute_x_axis_tolerance = x_tolerance;
	ocm.absolute_y_axis_tolerance = y_tolerance;
	ocm.absolute_z_axis_tolerance = z_tolerance;
	ocm.weight = 1.0;
	return ocm;
}

moveit_msgs::PositionConstraint MoveItPlanner::getSpherePoseConstraint(
		const std::string &link_name, 
		const geometry_msgs::PoseStamped &target_pose, 
		float maxArmReachDistance) {

	//ROS_INFO_STREAM("Combined path: "<<std::endl<<target_pose<<std::endl<<target_pose2);
	//ROS_INFO_STREAM("Combined path: "<<_target_pose<<", dir "<<_target1_to_target2);

	moveit_msgs::PositionConstraint pc;
	pc.link_name = link_name;
	pc.weight=1.0;
	pc.target_point_offset.x = 0;
	pc.target_point_offset.y = 0;
	pc.target_point_offset.z = 0;
	pc.header=target_pose.header;
	
	pc.constraint_region.primitives.resize(1);
	pc.constraint_region.primitive_poses.resize(1);
	
	shape_msgs::SolidPrimitive &bv = pc.constraint_region.primitives[0];
	geometry_msgs::Pose& pose=pc.constraint_region.primitive_poses[0];

	geometry_msgs::Pose sphereOrigin=target_pose.pose;
	pose=sphereOrigin;	
	bv=getSphere(maxArmReachDistance);
	return pc;
}



moveit_msgs::Constraints MoveItPlanner::getPoseConstraint(const std::string &link_name,
    const geometry_msgs::PoseStamped &pose, double tolerance_pos, double tolerance_angle, int type) {

	moveit_msgs::Constraints goal;

	if (type<=1) {	
		goal.position_constraints.resize(1);
		moveit_msgs::PositionConstraint &pcm = goal.position_constraints[0];
		pcm.link_name = link_name;
		pcm.target_point_offset.x = 0;
		pcm.target_point_offset.y = 0;
		pcm.target_point_offset.z = 0;
		pcm.constraint_region.primitives.resize(1);
		shape_msgs::SolidPrimitive &bv = pcm.constraint_region.primitives[0];
		bv=getSphere(tolerance_pos);

		pcm.header = pose.header;
		pcm.constraint_region.primitive_poses.resize(1);
		pcm.constraint_region.primitive_poses[0].position = pose.pose.position;

		// orientation of constraint region does not affect anything, since it is a sphere
		pcm.constraint_region.primitive_poses[0].orientation.x = 0.0;
		pcm.constraint_region.primitive_poses[0].orientation.y = 0.0;
		pcm.constraint_region.primitive_poses[0].orientation.z = 0.0;
		pcm.constraint_region.primitive_poses[0].orientation.w = 1.0;
		pcm.weight = 1.0;
	}
	
	if ((type==1) || (type==2)){
		goal.orientation_constraints.resize(1);
		moveit_msgs::OrientationConstraint &ocm = goal.orientation_constraints[0];
		ocm.link_name = link_name;
		ocm.header = pose.header;
		ocm.orientation = pose.pose.orientation;
		ocm.absolute_x_axis_tolerance = tolerance_angle;
		ocm.absolute_y_axis_tolerance = tolerance_angle;
		ocm.absolute_z_axis_tolerance = tolerance_angle;
		ocm.weight = 1.0;
	}/*else{
		ROS_WARN("An unsupported constraint type was passed into MoveItPlanner::getPoseConstraint: %i",type);
	}*/
	return goal;
}

