"""
behaviours.py – py_trees Behaviour subclasses for bt_interaction_nav.

Tree structure (built in tree.py):

    Selector(memory=False)
    ├─ Sequence(memory=False)   [person interaction branch]
    │   ├─ CheckPersonA          – SUCCESS if person A is visible on blackboard
    │   ├─ NotMetYet             – SUCCESS if is_person_a_met == False
    │   ├─ ApproachPerson        – drives toward person A until within target_distance
    │   ├─ WaitSeconds(10)       – pauses for N seconds
    │   └─ SetMetFlag            – sets is_person_a_met = True
    └─ NavigateToPoseB           – sends Nav2 NavigateToPose action goal B
"""

from __future__ import annotations

import math
import time
from typing import Optional

import py_trees
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

from . import blackboard_keys as BK


# ---------------------------------------------------------------------------
# CheckPersonA
# ---------------------------------------------------------------------------
class CheckPersonA(py_trees.behaviour.Behaviour):
    """
    Return SUCCESS when person A is visible on the blackboard.

    Reads:  person_visible (bool)
    """

    def __init__(self, name: str = "CheckPersonA") -> None:
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            key=BK.PERSON_VISIBLE, access=py_trees.common.Access.READ
        )

    def update(self) -> py_trees.common.Status:
        try:
            visible: bool = self.blackboard.person_visible
        except KeyError:
            visible = False
        return (
            py_trees.common.Status.SUCCESS
            if visible
            else py_trees.common.Status.FAILURE
        )


# ---------------------------------------------------------------------------
# NotMetYet
# ---------------------------------------------------------------------------
class NotMetYet(py_trees.behaviour.Behaviour):
    """
    Return SUCCESS when is_person_a_met == False (person has NOT yet been met).

    Reads:  is_person_a_met (bool)
    """

    def __init__(self, name: str = "NotMetYet") -> None:
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            key=BK.IS_PERSON_A_MET, access=py_trees.common.Access.READ
        )

    def update(self) -> py_trees.common.Status:
        try:
            met: bool = self.blackboard.is_person_a_met
        except KeyError:
            met = False
        return (
            py_trees.common.Status.FAILURE
            if met
            else py_trees.common.Status.SUCCESS
        )


# ---------------------------------------------------------------------------
# ApproachPerson
# ---------------------------------------------------------------------------
class ApproachPerson(py_trees.behaviour.Behaviour):
    """
    Drive toward person A using /cmd_vel until within target_distance.

    Uses a simple proportional controller:
        linear  = clamp(linear_kp  * (distance - target_distance), 0, max_linear)
        angular = clamp(angular_kp * angle, -max_angular, max_angular)

    Returns:
        RUNNING  – still approaching
        SUCCESS  – distance <= target_distance
        FAILURE  – person_visible becomes False mid-approach

    On terminate() (either SUCCESS, FAILURE, or external preemption) a stop
    command is published to /cmd_vel.

    Reads:   person_visible, person_distance, person_angle
    """

    def __init__(
        self,
        node: Node,
        cmd_vel_pub,
        target_distance: float = 0.5,
        max_linear: float = 0.20,
        max_angular: float = 0.8,
        linear_kp: float = 0.6,
        angular_kp: float = 1.2,
        name: str = "ApproachPerson",
    ) -> None:
        super().__init__(name)
        self._node = node
        self._pub = cmd_vel_pub
        self._target_distance = target_distance
        self._max_linear = max_linear
        self._max_angular = max_angular
        self._linear_kp = linear_kp
        self._angular_kp = angular_kp

        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            key=BK.PERSON_VISIBLE, access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key=BK.PERSON_DISTANCE, access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key=BK.PERSON_ANGLE, access=py_trees.common.Access.READ
        )

    # ------------------------------------------------------------------
    def _publish_stop(self) -> None:
        self._pub.publish(Twist())

    def terminate(self, new_status: py_trees.common.Status) -> None:  # noqa: ARG002
        self._publish_stop()

    def update(self) -> py_trees.common.Status:
        try:
            visible: bool = self.blackboard.person_visible
            distance: float = self.blackboard.person_distance
            angle: float = self.blackboard.person_angle
        except KeyError:
            self._publish_stop()
            return py_trees.common.Status.FAILURE

        if not visible:
            return py_trees.common.Status.FAILURE

        if distance <= self._target_distance:
            self._publish_stop()
            return py_trees.common.Status.SUCCESS

        # proportional controller
        linear = min(
            self._linear_kp * (distance - self._target_distance),
            self._max_linear,
        )
        angular = max(
            -self._max_angular,
            min(self._angular_kp * angle, self._max_angular),
        )

        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self._pub.publish(cmd)
        return py_trees.common.Status.RUNNING


# ---------------------------------------------------------------------------
# WaitSeconds
# ---------------------------------------------------------------------------
class WaitSeconds(py_trees.behaviour.Behaviour):
    """
    Block for *duration* seconds, then return SUCCESS.

    Resets its internal timer on initialise() so it is safe to re-enter.
    """

    def __init__(self, duration: float, name: str = "WaitSeconds") -> None:
        super().__init__(name)
        self._duration = duration
        self._start: Optional[float] = None

    def initialise(self) -> None:
        self._start = time.monotonic()

    def update(self) -> py_trees.common.Status:
        if self._start is None:
            return py_trees.common.Status.RUNNING
        elapsed = time.monotonic() - self._start
        if elapsed >= self._duration:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


# ---------------------------------------------------------------------------
# SetMetFlag
# ---------------------------------------------------------------------------
class SetMetFlag(py_trees.behaviour.Behaviour):
    """
    Set is_person_a_met = True on the blackboard, then return SUCCESS.

    Writes:  is_person_a_met (bool)
    """

    def __init__(self, name: str = "SetMetFlag") -> None:
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            key=BK.IS_PERSON_A_MET, access=py_trees.common.Access.WRITE
        )

    def update(self) -> py_trees.common.Status:
        self.blackboard.is_person_a_met = True
        return py_trees.common.Status.SUCCESS


# ---------------------------------------------------------------------------
# NavigateToPoseB
# ---------------------------------------------------------------------------
class NavigateToPoseB(py_trees.behaviour.Behaviour):
    """
    Send a Nav2 NavigateToPose action goal (goal B) and track its result.

    States:
        RUNNING  – waiting for the action server or goal is in progress
        SUCCESS  – goal reached (SUCCEEDED)
        FAILURE  – goal rejected, aborted, or cancelled

    On terminate() (preemption from the Selector), the goal is cancelled so
    that nav2 does not continue driving after the tree switches branches.
    """

    def __init__(
        self,
        node: Node,
        action_name: str,
        goal_pose,  # geometry_msgs.msg.PoseStamped
        name: str = "NavigateToPoseB",
    ) -> None:
        super().__init__(name)
        self._node = node
        self._action_name = action_name
        self._goal_pose = goal_pose
        self._client: ActionClient = ActionClient(
            node, NavigateToPose, action_name
        )
        self._goal_handle = None
        self._result_future = None
        self._status = py_trees.common.Status.INVALID

    # ------------------------------------------------------------------
    def initialise(self) -> None:
        """Send the goal on first tick (or re-entry after preemption)."""
        self._goal_handle = None
        self._result_future = None
        self._status = py_trees.common.Status.RUNNING

        if not self._client.wait_for_server(timeout_sec=0.1):
            self._node.get_logger().warn(
                f"NavigateToPoseB: action server '{self._action_name}' not available yet"
            )
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._goal_pose
        send_future = self._client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_response_callback)

    # ------------------------------------------------------------------
    def _goal_response_callback(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self._node.get_logger().warn("NavigateToPoseB: goal rejected")
            self._status = py_trees.common.Status.FAILURE
            return
        self._goal_handle = handle
        self._result_future = handle.get_result_async()
        self._result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future) -> None:
        result_response = future.result()
        code = result_response.status
        if code == GoalStatus.STATUS_SUCCEEDED:
            self._status = py_trees.common.Status.SUCCESS
        else:
            self._status = py_trees.common.Status.FAILURE

    # ------------------------------------------------------------------
    def update(self) -> py_trees.common.Status:
        return self._status

    # ------------------------------------------------------------------
    def terminate(self, new_status: py_trees.common.Status) -> None:
        """Cancel the navigation goal when preempted."""
        if (
            self._goal_handle is not None
            and new_status == py_trees.common.Status.INVALID
        ):
            self._goal_handle.cancel_goal_async()
            self._node.get_logger().info(
                "NavigateToPoseB: goal cancelled (preempted)"
            )
        self._goal_handle = None
        self._result_future = None
        self._status = py_trees.common.Status.INVALID
