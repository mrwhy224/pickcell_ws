"""Regression tests for a non-cumulative, box-relative release target."""

from pickcell_motion_server.drop_target import DropTarget


def test_all_four_bags_use_one_immutable_box_centre() -> None:
    """Later releases must not inherit a slot or offset from earlier bags."""
    target = DropTarget.from_values((0.0, 0.0, 0.63), 0.03, 0.04)
    actual_positions = (
        (0.000, 0.000, 0.630),
        (0.010, -0.012, 0.625),
        (-0.015, 0.011, 0.646),
        (0.004, 0.018, 0.601),
    )

    assert all(target.accepts(position) for position in actual_positions)
    assert target.center == (0.0, 0.0, 0.63)


def test_box_edge_or_rail_release_is_rejected() -> None:
    """The old multi-slot position near a wall is not accepted as centred."""
    target = DropTarget.from_values((0.0, 0.0, 0.63), 0.03, 0.04)

    assert not target.accepts((0.30, -0.25, 0.63))
