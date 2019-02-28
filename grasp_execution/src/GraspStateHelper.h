#ifndef GRASP_STATE_HELPER_H
#define GRASP_STATE_HELPER_H

#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace grasp_execution
{

// Helper for intermediately fixing the changeover from sensor_msgs/JointState
// to trajectory_msgs/JointTrajectory: writes the first joint trajectory point
// as joint state.
// Returns 0 on success, -1 if there is no JointTrajectoryPoint, and 1 if
// there are several points (a warning, because only the first is used and
// the rest is ignored).
int getStateFromTrajectory(const trajectory_msgs::JointTrajectory &jt,
                            sensor_msgs::JointState &js);

}  // namespace

#endif  // GRASP_STATE_HELPER_H
