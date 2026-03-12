"""Blackboard key constants for bt_interaction_nav."""

# ── /person detection ──────────────────────────────────────────────────────
PERSON_VISIBLE = "person_visible"      # bool  – person A is currently visible
PERSON_DISTANCE = "person_distance"   # float – distance to person A (metres)
PERSON_ANGLE = "person_angle"         # float – bearing to person A (radians)

# ── state flags ────────────────────────────────────────────────────────────
IS_PERSON_A_MET = "is_person_a_met"   # bool  – person A has been approached
