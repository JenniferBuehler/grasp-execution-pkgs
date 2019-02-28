#include "GraspStateHelper.h"

int grasp_execution::getStateFromTrajectory(const trajectory_msgs::JointTrajectory &jt,
                            sensor_msgs::JointState &js)
{
  js.header = jt.header;
  js.name = jt.joint_names;
  if (jt.points.empty())
  {
    return -1;
  }
  js.position = jt.points.front().positions;
  if (jt.points.size() != 1)
    return 1;
  return 0;
}
