## moveit python trajectory planning

in move_group.py
```
from moveit_ros_planning_interface import _moveit_move_group_interface
class MoveGroupCommander(object):
    """
    Execution of simple commands for a particular group
    """

    def __init__(
        self, name, robot_description="robot_description", ns="", wait_for_servers=5.0
    ):
        """Specify the group name for which to construct this commander instance. Throws an exception if there is an initialization error."""
        self._g = _moveit_move_group_interface.MoveGroupInterface(
            name, robot_description, ns, wait_for_servers
        )

    def plan(self, joints=None):
        """Return a tuple of the motion planning results such as
        (success flag : boolean, trajectory message : RobotTrajectory,
         planning time : float, error code : MoveitErrorCodes)"""
        if type(joints) is str:
            self.set_joint_value_target(self.get_remembered_joint_values()[joints])
        elif type(joints) is Pose:
            self.set_pose_target(joints)
        elif joints is not None:
            self.set_joint_value_target(joints)

        (error_code_msg, trajectory_msg, planning_time) = self._g.plan()

        error_code = MoveItErrorCodes()
        error_code.deserialize(error_code_msg)
        plan = RobotTrajectory()
        return (
            error_code.val == MoveItErrorCodes.SUCCESS,
            plan.deserialize(trajectory_msg),
            planning_time,
            error_code,
        )
```
In move_group_interface.cpp
```
  void setNumPlanningAttempts(unsigned int num_planning_attempts)
  {
    num_planning_attempts_ = num_planning_attempts;
  }
```

![Alt text](image.png)