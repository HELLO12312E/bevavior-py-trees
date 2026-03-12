"""
bt_runner_node.py – ROS 2 node that owns the behavior tree and ticks it.

Parameters (all declared via declare_parameter):
    tick_rate_hz        (float, default 10.0)
    min_confidence      (float, default 0.7)
    target_distance     (float, default 0.50)
    wait_seconds        (float, default 10.0)
    max_linear_speed    (float, default 0.20)
    max_angular_speed   (float, default 0.80)
    linear_kp           (float, default 0.60)
    angular_kp          (float, default 1.20)
    person_topic        (str,   default "/person")
    cmd_vel_topic       (str,   default "/cmd_vel")
    nav_action_name     (str,   default "/navigate_to_pose")
    goal_b.frame_id     (str,   default "map")
    goal_b.x            (float, default 0.0)
    goal_b.y            (float, default 0.0)
    goal_b.yaw          (float, default 0.0)
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from .adapters import PersonAdapter
from .tree import create_tree


class BTRunnerNode(Node):
    def __init__(self) -> None:
        super().__init__("bt_runner_node")

        # ── declare & read parameters ──────────────────────────────────
        self.declare_parameter("tick_rate_hz",     10.0)
        self.declare_parameter("min_confidence",    0.7)
        self.declare_parameter("target_distance",   0.50)
        self.declare_parameter("wait_seconds",     10.0)
        self.declare_parameter("max_linear_speed",  0.20)
        self.declare_parameter("max_angular_speed", 0.80)
        self.declare_parameter("linear_kp",         0.60)
        self.declare_parameter("angular_kp",        1.20)
        self.declare_parameter("person_topic",      "/person")
        self.declare_parameter("cmd_vel_topic",     "/cmd_vel")
        self.declare_parameter("nav_action_name",   "/navigate_to_pose")
        self.declare_parameter("goal_b.frame_id",   "map")
        self.declare_parameter("goal_b.x",          0.0)
        self.declare_parameter("goal_b.y",          0.0)
        self.declare_parameter("goal_b.yaw",        0.0)

        tick_rate_hz       = self.get_parameter("tick_rate_hz").value
        min_confidence     = self.get_parameter("min_confidence").value
        target_distance    = self.get_parameter("target_distance").value
        wait_seconds       = self.get_parameter("wait_seconds").value
        max_linear_speed   = self.get_parameter("max_linear_speed").value
        max_angular_speed  = self.get_parameter("max_angular_speed").value
        linear_kp          = self.get_parameter("linear_kp").value
        angular_kp         = self.get_parameter("angular_kp").value
        person_topic       = self.get_parameter("person_topic").value
        cmd_vel_topic      = self.get_parameter("cmd_vel_topic").value
        nav_action_name    = self.get_parameter("nav_action_name").value
        goal_b_frame_id    = self.get_parameter("goal_b.frame_id").value
        goal_b_x           = self.get_parameter("goal_b.x").value
        goal_b_y           = self.get_parameter("goal_b.y").value
        goal_b_yaw         = self.get_parameter("goal_b.yaw").value

        self.get_logger().info(
            f"BTRunnerNode starting – tick_rate={tick_rate_hz} Hz, "
            f"min_confidence={min_confidence}, target_distance={target_distance} m"
        )

        # ── publishers ─────────────────────────────────────────────────
        self._cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # ── behavior tree ──────────────────────────────────────────────
        self._tree = create_tree(
            node=self,
            cmd_vel_pub=self._cmd_vel_pub,
            action_name=nav_action_name,
            target_distance=target_distance,
            max_linear=max_linear_speed,
            max_angular=max_angular_speed,
            linear_kp=linear_kp,
            angular_kp=angular_kp,
            wait_seconds=wait_seconds,
            goal_b_frame_id=goal_b_frame_id,
            goal_b_x=goal_b_x,
            goal_b_y=goal_b_y,
            goal_b_yaw=goal_b_yaw,
        )
        self._tree.setup(timeout=15)

        # ── person adapter (writes to blackboard) ──────────────────────
        import py_trees

        _bb = py_trees.blackboard.Client(name="PersonAdapterBB")
        from . import blackboard_keys as BK

        _bb.register_key(key=BK.PERSON_VISIBLE,  access=py_trees.common.Access.WRITE)
        _bb.register_key(key=BK.PERSON_DISTANCE, access=py_trees.common.Access.WRITE)
        _bb.register_key(key=BK.PERSON_ANGLE,    access=py_trees.common.Access.WRITE)

        self._person_adapter = PersonAdapter(
            node=self,
            topic=person_topic,
            min_confidence=min_confidence,
            blackboard=_bb,
        )

        # ── tick timer ─────────────────────────────────────────────────
        period = 1.0 / max(tick_rate_hz, 0.1)
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info("BTRunnerNode ready.")

    # ------------------------------------------------------------------
    def _tick(self) -> None:
        self._tree.tick()

    # ------------------------------------------------------------------
    def destroy_node(self) -> None:
        # Stop the robot on shutdown
        self._cmd_vel_pub.publish(Twist())
        super().destroy_node()


# ---------------------------------------------------------------------------
def main(args=None) -> None:
    rclpy.init(args=args)
    node = BTRunnerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
