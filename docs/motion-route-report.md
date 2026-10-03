# Two-anchor motion route

The authoritative diagnosis, measured route, state transition table, and
limitations are in [deep-debug-report.md](deep-debug-report.md).

The active cyclic route has exactly two persistent Cartesian anchors in
`cell`: A_PICK at `[1.4, 0, 1.2, 0, 180, 0]` and B_DROP at
`[-1.4, 0, 0.72, 0, 180, 0]`. Bag-specific hover, contact, and lift poses are
temporary. The fixed transfer is a validated 81-sample joint sweep dominated
by positive base yaw, followed after release by freshly timed reverse geometry.

MoveIt checks the robot, pallet, and box. The stateful payload is a contact-
gated marker with a preserved tool-relative transform; it is not an attached
MoveIt collision object or an Isaac physical attachment. This simulation route
is therefore not physical commissioning evidence.
