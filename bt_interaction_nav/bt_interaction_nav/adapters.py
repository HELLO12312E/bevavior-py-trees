"""
adapters.py – ROS 2 subscription adapters that write to the py_trees blackboard.

The /person topic carries a message with fields:
    id          (string)   – identity tag; "A" for target person; "" for no-detection
    distance    (float64)  – metres
    angle       (float64)  – radians (positive = left)
    x           (float64)  – position in sensor frame
    y           (float64)  – position in sensor frame
    confidence  (float32)  – detection confidence in [0.0, 1.0]

No-detection is encoded as:  id == ""  AND  confidence == 0.0

TODO: replace the placeholder import with the real message type:
    from your_person_msgs.msg import Person
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from . import blackboard_keys as BK

# ---------------------------------------------------------------------------
# Placeholder message type – replace with the real import once the package is
# known.  The adapter will fail at runtime if this import is not updated.
# ---------------------------------------------------------------------------
try:
    # TODO: replace with real import, e.g.:
    #   from your_person_msgs.msg import Person
    from std_msgs.msg import String as Person  # noqa: F401 (placeholder)
    _PERSON_MSG_PLACEHOLDER = True
except ImportError:
    _PERSON_MSG_PLACEHOLDER = True


class PersonAdapter:
    """
    Subscribe to the /person topic and write detection data to the blackboard.

    Parameters
    ----------
    node : rclpy.node.Node
        The owning ROS 2 node.
    topic : str
        Topic name (default "/person").
    min_confidence : float
        Detections below this confidence are ignored (treated as no-detection).
    person_id : str
        The id value we are looking for (default "A").
    blackboard : py_trees.blackboard.Client
        Pre-configured blackboard client with WRITE access to the required keys.
    """

    def __init__(
        self,
        node: Node,
        topic: str,
        min_confidence: float,
        blackboard,
        person_id: str = "A",
    ) -> None:
        self._node = node
        self._person_id = person_id
        self._min_confidence = min_confidence
        self._bb = blackboard

        # TODO: replace Person with the real message class once the package is
        #       available.  The subscription QoS is best-effort / sensor data.
        from rclpy.qos import (
            QoSDurabilityPolicy,
            QoSHistoryPolicy,
            QoSProfile,
            QoSReliabilityPolicy,
        )

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # We use a generic subscription callback; replace the message type as
        # soon as the real package is available.
        #
        # *** REPLACE THE LINE BELOW ***
        #   msg_type=Person  →  msg_type=<your_person_msgs>.msg.Person
        #
        # For now we subscribe to the raw topic with Any-like approach so the
        # node at least starts.  The callback guards against missing attributes.
        self._sub = node.create_subscription(
            msg_type=Person,       # <-- TODO: swap placeholder
            topic=topic,
            callback=self._callback,
            qos_profile=qos,
        )
        node.get_logger().info(
            f"PersonAdapter: subscribed to '{topic}' "
            f"(min_confidence={min_confidence})"
        )

    # ------------------------------------------------------------------
    def _callback(self, msg) -> None:
        """Process incoming /person message and update the blackboard."""
        try:
            person_id: str = getattr(msg, "id", "")
            confidence: float = float(getattr(msg, "confidence", 0.0))
            distance: float = float(getattr(msg, "distance", 0.0))
            angle: float = float(getattr(msg, "angle", 0.0))
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().error(
                f"PersonAdapter: failed to parse message: {exc}"
            )
            self._bb.person_visible = False
            return

        # No-detection sentinel or low-confidence → treat as invisible
        if (
            person_id == ""
            or confidence < self._min_confidence
            or person_id != self._person_id
        ):
            self._bb.person_visible = False
            return

        self._bb.person_visible = True
        self._bb.person_distance = distance
        self._bb.person_angle = angle
