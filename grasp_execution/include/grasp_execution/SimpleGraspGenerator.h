#ifndef GRASP_EXECUTION_SIMPLEGRASPGENERATOR_H
#define GRASP_EXECUTION_SIMPLEGRASPGENERATOR_H

#include <convenience_ros_functions/ROSFunctions.h>
#include <eigen_conversions/eigen_msg.h>

namespace grasp_execution
{

using convenience_ros_functions::ROSFunctions;

/**
 * Helper functions to generate simple Grasp message objects.
 */
class SimpleGraspGenerator
{
public:
    SimpleGraspGenerator(){}
    ~SimpleGraspGenerator(){}


    /**
     * Generates a simple test manipulaton_msgs::Grasp in which all gripper joints have the same angles.
     * The grasp pose is just vertical above the object, at distance \e pose_above_object
     * (which is z-direction) and shifted \e pose_x_from_object and \e pose_y_from_object
     * relative to the object.
     *
     * The resulting grasp pose will be **relative to the object** (i.e. the objects frame
     * id will be stated in the grasp pose). Therefore, it will be valid even if the object
     * is moved around the world.
     *
     * Note however that if the object is tipped over, the grasp still tries to grasp
     * the object from the top, which won't work.
     * moveit_msgs::Grasp::allowed_touch_objects is initialized with the object name.
     *
     * \param joint_names the names of all gripper joints in the order they should
     *      appear in the JointState.
     * \param robot_link_frame a frame of a link of the robot. The object pose will be transformed
     *      into this frame. This transformed pose is then used as a base for the target grasp
     *      pose (\e grasp.grasp_pose). Ideally, choose the base of the arm for this, or any
     *      other link on the robot which does not change in-between this call and the arm moving to grasp.
     * \param grasp_id the name/ID of the grasp
     * \param object_name the name which is put in the allowed_touch_objects
     */
    static bool generateSimpleGraspFromTop(
        const std::vector<std::string> joint_names,
        const std::string& robot_link_frame,
        const std::string& grasp_id,
        const std::string& object_name,
        const geometry_msgs::PoseStamped& object_pose,
        const std::string& object_frame_id,
        float pose_above_object,
        float pose_x_from_object,
        float pose_y_from_object,
        float grasp_open_angles, //=0.05; //on real jaco, 0 can't be reached.
        float grasp_close_angles, //=0.7;
        moveit_msgs::Grasp& grasp) {

        ROSFunctions::initSingleton();

        //first, transform the object into a frame relative to the robot.
        geometry_msgs::PoseStamped _object_pose=object_pose;
        _object_pose.header.stamp=ros::Time(0); //most recent time for transformPose
        geometry_msgs::PoseStamped aboveObj;
        if (ROSFunctions::Singleton()->transformPose(_object_pose,robot_link_frame,aboveObj,1)!=0) {
            ROS_ERROR("Test transform failed.");
            return false;
        }

        //Now, find the right Hand orientation above the object.

        //Eigen::Vector3d _pos;
        //tf::vectorMsgToEigen(aboveObj.pose.position,_pos);
        Eigen::Quaterniond _quat;
        tf::quaternionMsgToEigen(aboveObj.pose.orientation,_quat);

        Eigen::Vector3d _x(1,0,0);
        Eigen::Vector3d _y(0,1,0);
        Eigen::Vector3d _z(0,0,1);
        _x=_quat*_x;
        _y=_quat*_y;
        _z=_quat*_z;
        _x*=pose_x_from_object;
        _y*=pose_y_from_object;
        _z*=pose_above_object;

        // ROS_INFO_STREAM("Generating grasp pose "<<aboveObj);
        // ROS_INFO("Left/right: %f/%f",pose_x_from_object,pose_y_from_object);

        grasp.id=grasp_id;
        grasp.pre_grasp_posture=simpleGrasp(joint_names, grasp_open_angles); //sensor_msgs::JointState, hand posture for the pre-grasp
        grasp.grasp_posture=simpleGrasp(joint_names, grasp_close_angles); //sensor_msgs::JointState, hand posture for the grasp

        //grasp.grasp_pose.pose.position.x+=_x.x()+_y.x()+_z.x();
        //aboveObj.pose.position.y+=_x.y()+_y.y()+_z.y();
        aboveObj.pose.position.z+=pose_above_object;//_x.z()+_y.z()+_z.z();

        //ROS_INFO_STREAM("Adapted grasp pose "<<aboveObj);

        Eigen::Quaterniond rq;
        tf::quaternionMsgToEigen(aboveObj.pose.orientation,rq);
        // rotate -90 around y is the orientation we'd like to have, in relation to the object orientation. We assume that the
        // object orientation is such that the x axis points into its longer side (the long side of the cube) and lies on the ground plane.
        // orientation: rotate -90 around y is 0, -sqrt(0.5), 0, sqrt(0.5)
        Eigen::Quaterniond align_ori(sqrt(0.5), 0, -sqrt(0.5), 0); //constructor has w first
        Eigen::Quaterniond rori=rq*align_ori;
        aboveObj.pose.orientation.x=rori.x();
        aboveObj.pose.orientation.y=rori.y();
        aboveObj.pose.orientation.z=rori.z();
        aboveObj.pose.orientation.w=rori.w();

        //now, transform aboveObj to the object's frame:
        //ROS_INFO("Transforming to the frame above the object");

        //first, transform the object into a frame relative to the robot.
        aboveObj.header.stamp=ros::Time(0);
        if (ROSFunctions::Singleton()->transformPose(aboveObj,object_frame_id,aboveObj,2.0)!=0) {
            ROS_ERROR("Transform to object frame failed.");
            return false;
        }

        grasp.grasp_pose=aboveObj; //geometry_msgs::PoseStamped, effector pose for the grasp
        // ROS_INFO_STREAM("Final grasp pose "<<grasp.grasp_pose.pose);

        grasp.grasp_quality=0.5; //probability of success
        //grasp.approach= //moveit_msgs::GripperTranslation
        //grasp.retreat= //moveit_msgs::GripperTranslation
        grasp.max_contact_force=-1; //disable maximum contact force

        grasp.allowed_touch_objects.push_back(object_name);
        return true;
    }

    /**
     * Generates a simple grasp goal, without gripper_approach_trajectory
     * and gripper_retreat_trajectory: This is just a grasp where the
     * effector is positioned correctly and can then grasp the object right away.
     * Default tolerances are used, so if you want to adapt them, you need
     * to pass the resulting \e graspGoal through setCustomTolerances().
     * The current robot state is also not initialized, you have to do this
     * manually.
     *
     * \param isGrasp true if this is a grasp action, false if un-grasp
     */
    static void generateSimpleGraspGoal(const std::string& effectorLink,
        const moveit_msgs::Grasp& grasp,
        const int graspID,
        bool isGrasp,
        grasp_execution_msgs::GraspGoal& graspGoal)
    {
        graspGoal.grasp.grasp=grasp;
        graspGoal.grasp.id=graspID;
        graspGoal.grasp.effector_link_name = effectorLink;
        graspGoal.is_grasp = isGrasp;
        graspGoal.ignore_effector_pose_ungrasp = true;
        graspGoal.use_custom_tolerances = false;

        if (grasp.grasp_posture.joint_names.size() !=
            grasp.pre_grasp_posture.joint_names.size())
        {
          ROS_ERROR_STREAM("Expecting pre-grasp joint trajectory points to be "
            << "of same size as grasp posture. Returning empty trajectory.");
          return;
        }

        trajectory_msgs::JointTrajectory graspTrajectory;
        // the joint names should be the same in pre_grasp_posture
        // and grasp_posture.
        graspTrajectory.joint_names = grasp.grasp_posture.joint_names;
        graspTrajectory.header.stamp = ros::Time::now();
        if (isGrasp)
        {
            graspTrajectory.points = grasp.pre_grasp_posture.points;
            graspTrajectory.points.insert(graspTrajectory.points.end(),
              grasp.grasp_posture.points.begin(),
              grasp.grasp_posture.points.end());
        }
        else
        {
            graspTrajectory.points = grasp.grasp_posture.points;
            graspTrajectory.points.insert(graspTrajectory.points.end(),
              grasp.pre_grasp_posture.points.begin(),
              grasp.pre_grasp_posture.points.end());
        }
        graspGoal.grasp_trajectory=graspTrajectory;
    }

    /**
     * Adjust fields in \graspGoal to use custom tolerances.
     */
    static void useCustomTolerances(const float effector_pos,
        const float effector_angle, const float joint_angles,
        grasp_execution_msgs::GraspGoal& graspGoal)
    {
        graspGoal.use_custom_tolerances = true;
        graspGoal.effector_pos_tolerance = effector_pos;
        graspGoal.effector_angle_tolerance = effector_angle;
        graspGoal.joint_angles_tolerance = joint_angles;
    }

private:

    // returns joint state for fingers in the grasp position for the simple test grasp
    static trajectory_msgs::JointTrajectory simpleGrasp(
        const std::vector<std::string> jointNames,
        float pos, float effort=0)
    {
        trajectory_msgs::JointTrajectory js;
        js.joint_names = jointNames;
        trajectory_msgs::JointTrajectoryPoint jsp;
        for (int i=0; i<jointNames.size(); ++i)
        {
            jsp.positions.push_back(pos);
            jsp.velocities.push_back(0);
            jsp.effort.push_back(effort);
        }
        js.points.push_back(jsp);
        return js;
    }

};

}  // namespace

#endif  //  GRASP_EXECUTION_SIMPLEGRASPGENERATOR_H
