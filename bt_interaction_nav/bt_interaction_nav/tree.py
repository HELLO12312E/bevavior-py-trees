"""
tree.py – Build and return the root of the bt_interaction_nav behavior tree.

Tree structure
--------------

    Selector(memory=False)                          [root]
    ├─ Sequence(memory=False)                       [person interaction branch]
    │   ├─ CheckPersonA    – person A is visible?
    │   ├─ NotMetYet       – is_person_a_met == False?
    │   ├─ ApproachPerson  – drive toward person A (P-controller)
    │   ├─ WaitSeconds(N)  – pause N seconds
    │   └─ SetMetFlag      – set is_person_a_met = True
    └─ NavigateToPoseB     – navigate to goal B via Nav2

Logic
-----
On each tick the Selector tries the person-interaction Sequence first.
* If person A is visible AND has not been met yet → the Sequence runs, the
  robot approaches, waits, sets the flag, and returns SUCCESS → Selector
  succeeds, NavigateToPoseB is NOT ticked (Nav2 goal cancelled if active).
* Otherwise the Sequence returns FAILURE → Selector falls through to
  NavigateToPoseB.
* Once is_person_a_met == True, NotMetYet returns FAILURE every tick, so the
  Sequence never progresses and NavigateToPoseB runs uninterrupted.
"""

from __future__ import annotations

import math

import py_trees
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.node import Node

from .behaviours import (
    ApproachPerson,
    CheckPersonA,
    NavigateToPoseB,
    NotMetYet,
    SetMetFlag,
    WaitSeconds,
)
from . import blackboard_keys as BK


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert a yaw angle (radians) to a geometry_msgs Quaternion."""
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def create_tree(
    node: Node,
    cmd_vel_pub,
    *,
    action_name: str = "/navigate_to_pose",
    target_distance: float = 0.5,
    max_linear: float = 0.20,
    max_angular: float = 0.8,
    linear_kp: float = 0.6,
    angular_kp: float = 1.2,
    wait_seconds: float = 10.0,
    goal_b_frame_id: str = "map",
    goal_b_x: float = 0.0,
    goal_b_y: float = 0.0,
    goal_b_yaw: float = 0.0,
) -> py_trees.trees.BehaviourTree:
    """
    Construct and return the complete behavior tree.

    Parameters
    ----------
    node          : owning ROS 2 node (for action/publisher handles)
    cmd_vel_pub   : rclpy publisher for geometry_msgs/msg/Twist
    action_name   : Nav2 NavigateToPose action server name
    target_distance : stop approaching when person is within this distance (m)
    max_linear    : maximum linear speed for approach (m/s)
    max_angular   : maximum angular speed for approach (rad/s)
    linear_kp     : proportional gain for linear velocity
    angular_kp    : proportional gain for angular velocity
    wait_seconds  : how long to wait after reaching person A (s)
    goal_b_*      : target pose for NavigateToPoseB
    """

    # ── goal B pose ────────────────────────────────────────────────────
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = goal_b_frame_id
    goal_pose.pose.position.x = goal_b_x
    goal_pose.pose.position.y = goal_b_y
    goal_pose.pose.orientation = _yaw_to_quaternion(goal_b_yaw)

    # ── initialise blackboard defaults ────────────────────────────────
    bb = py_trees.blackboard.Client(name="TreeInit")
    bb.register_key(key=BK.PERSON_VISIBLE,  access=py_trees.common.Access.WRITE)
    bb.register_key(key=BK.PERSON_DISTANCE, access=py_trees.common.Access.WRITE)
    bb.register_key(key=BK.PERSON_ANGLE,    access=py_trees.common.Access.WRITE)
    bb.register_key(key=BK.IS_PERSON_A_MET, access=py_trees.common.Access.WRITE)
    bb.person_visible = False
    bb.person_distance = 0.0
    bb.person_angle = 0.0
    bb.is_person_a_met = False

    # ── behaviours ────────────────────────────────────────────────────
    check_person_a = CheckPersonA()
    not_met_yet = NotMetYet()
    approach_person = ApproachPerson(
        node=node,
        cmd_vel_pub=cmd_vel_pub,
        target_distance=target_distance,
        max_linear=max_linear,
        max_angular=max_angular,
        linear_kp=linear_kp,
        angular_kp=angular_kp,
    )
    wait = WaitSeconds(duration=wait_seconds, name=f"WaitSeconds({wait_seconds:.0f}s)")
    set_flag = SetMetFlag()
    nav_b = NavigateToPoseB(
        node=node,
        action_name=action_name,
        goal_pose=goal_pose,
    )

    # ── tree structure ─────────────────────────────────────────────────
    person_seq = py_trees.composites.Sequence(
        name="PersonInteraction",
        memory=False,
        children=[check_person_a, not_met_yet, approach_person, wait, set_flag],
    )

    root = py_trees.composites.Selector(
        name="Root",
        memory=False,
        children=[person_seq, nav_b],
    )

    tree = py_trees.trees.BehaviourTree(root=root)
    return tree
