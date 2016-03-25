#ifdef DOXYGEN_SHOULD_SKIP_THIS
/**
   Sends a sensor_msgs/JointState message to a topic in order to set the joint state.

   Copyright (C) 2015 Jennifer Buehler

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
*/
#endif

#include <ros/ros.h>

#include <jaco_joints/JacoJointManager.h>
#include <moveit_object_handling/GraspedObjectHandler.h>
#include <string>
#include <vector>

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr<<"Usage: "<<argv[0]<<" <object-name> [--detach]"<<std::endl;
        return 1;
    } 
    std::string objName=argv[1];
   
    bool doDetach=false;
    if (argc > 2) 
    {
        std::string arg=argv[2];
        if (arg=="--detach")
        {
            ROS_INFO("detach cube");
            doDetach=true;
        }
    }

    ros::init(argc, argv, "set_joint_state_cube_publisher");
    ros::NodeHandle n("");

    JacoJointManager joints;
    std::vector<std::string> gripperLinkNames=joints.getGripperLinks();
    std::string palmLinkName = joints.getPalmLink();
    gripperLinkNames.push_back(palmLinkName);

    std::string get_planning_scene("/get_planning_scene");
    std::string set_planning_scene("/planning_scene");

    moveit_object_handling::GraspedObjectHandlerMoveIt graspHandler(n,gripperLinkNames,get_planning_scene,set_planning_scene);
    graspHandler.waitForSubscribers();

    if (!doDetach)
    {
        graspHandler.attachObjectToRobot(objName,palmLinkName);
    }
    else
    {
        graspHandler.detachObjectFromRobot(objName);
    }

    // wait to allow that the last message arrives before quitting node
    ros::Duration(1).sleep();
    return 0;
}
