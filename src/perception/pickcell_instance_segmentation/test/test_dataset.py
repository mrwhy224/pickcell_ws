"""Tests for RGB-D dataset models, parsing, loading, and validation."""

import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from pickcell_instance_segmentation import DatasetValidationError
from pickcell_instance_segmentation import load_scene
from pickcell_instance_segmentation import parse_camera_json
from pickcell_instance_segmentation import parse_capture_camera_json
from pickcell_instance_segmentation import parse_isaac_camera_json
from pickcell_instance_segmentation import validate_scene


def capture_camera() -> dict:
    """Return a minimal ROS capture camera document."""
    return {
        "width": 3,
        "height": 2,
        "frame_id": "camera_optical_frame",
        "intrinsic_matrix": [100.0, 0.0, 1.5, 0.0, 101.0, 1.0, 0.0, 0.0, 1.0],
        "projection_matrix": [100.0, 0.0, 1.5, 0.0, 0.0, 101.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        "distortion_model": "plumb_bob",
        "distortion_coefficients": [0.0] * 5,
        "depth_scale": 1000.0,
        "stored_depth_encoding": "16UC1",
    }


def projection() -> list[float]:
    """Return a supported symmetric row-major projection matrix."""
    return [2.0, 0.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0,
            0.0, 0.0, -1.0, -1.0, 0.0, 0.0, -0.2, 0.0]


def isaac_camera(**updates) -> dict:
    """Return a minimal supported Isaac camera document."""
    result = {
        "renderProductResolution": [3, 2],
        "cameraModel": "pinhole",
        "cameraProjection": projection(),
        "fx": 120.0,
        "fy": 121.0,
        "cx": 1.5,
        "cy": 1.0,
        "depth_scale": 1000.0,
    }
    result.update(updates)
    return result


def write_scene(
    directory: Path,
    *,
    color: np.ndarray | None = None,
    depth: np.ndarray | None = None,
    instance: np.ndarray | None = None,
    mapping: dict | None = None,
    camera: dict | None = None,
    scene: dict | None = None,
) -> Path:
    """Write a small scene fixture into a pytest temporary directory."""
    directory.mkdir()
    if color is None:
        color = np.zeros((2, 3, 3), dtype=np.uint8)
    if depth is None:
        depth = np.full((2, 3), 1000, dtype=np.uint16)
    assert cv2.imwrite(str(directory / "color.png"), color)
    assert cv2.imwrite(str(directory / "depth.png"), depth)
    (directory / "camera.json").write_text(
        json.dumps(camera or capture_camera()), encoding="utf-8"
    )
    if instance is not None:
        assert cv2.imwrite(str(directory / "instance.png"), instance)
    if mapping is not None:
        (directory / "instance_mapping.json").write_text(
            json.dumps(mapping), encoding="utf-8"
        )
    if scene is not None:
        (directory / "scene.json").write_text(
            json.dumps(scene), encoding="utf-8"
        )
    return directory


def test_valid_capture_camera_schema() -> None:
    """ROS capture intrinsics and depth scale are parsed directly."""
    camera = parse_capture_camera_json(capture_camera())
    assert (camera.width, camera.height) == (3, 2)
    assert (camera.fx, camera.fy, camera.cx, camera.cy) == (100.0, 101.0, 1.5, 1.0)
    assert camera.depth_scale == 1000.0
    assert camera.source_schema == "ros_capture"
    assert not camera.distortion_coefficients.flags.writeable


def test_valid_isaac_camera_schema() -> None:
    """Positive explicit Isaac OpenCV intrinsics take precedence."""
    camera = parse_isaac_camera_json(isaac_camera())
    assert (camera.fx, camera.fy, camera.cx, camera.cy) == (120.0, 121.0, 1.5, 1.0)
    assert camera.source_schema == "isaac_replicator"


def test_zero_isaac_focals_use_supported_projection() -> None:
    """Zero focal values use only the supported symmetric projection."""
    camera = parse_isaac_camera_json(isaac_camera(fx=0.0, fy=0.0, cx=0.0, cy=0.0))
    assert (camera.fx, camera.fy) == (3.0, 3.0)
    assert (camera.cx, camera.cy) == (1.5, 1.0)


@pytest.mark.parametrize("document", [{"width": 3}, {
    **capture_camera(), "renderProductResolution": [3, 2]
}])
def test_unsupported_or_ambiguous_camera_schema(document: dict) -> None:
    """Automatic schema selection rejects unsupported and mixed documents."""
    with pytest.raises(DatasetValidationError, match="schema"):
        parse_camera_json(document)


def test_bgr_input_is_converted_to_rgb(tmp_path: Path) -> None:
    """Convert OpenCV BGR input into an RGB array."""
    bgr = np.zeros((2, 3, 3), dtype=np.uint8)
    bgr[0, 0] = [1, 2, 3]
    scene = load_scene(write_scene(tmp_path / "scene", color=bgr))
    assert scene.color[0, 0].tolist() == [3, 2, 1]
    assert scene.color.shape == (2, 3, 3)


def test_bgra_input_is_converted_to_rgb(tmp_path: Path) -> None:
    """Convert OpenCV BGRA input into RGB without alpha."""
    bgra = np.zeros((2, 3, 4), dtype=np.uint8)
    bgra[0, 0] = [1, 2, 3, 4]
    scene = load_scene(write_scene(tmp_path / "scene", color=bgra))
    assert scene.color[0, 0].tolist() == [3, 2, 1]
    assert scene.color.shape == (2, 3, 3)


def test_uint16_depth_and_instances_are_preserved(tmp_path: Path) -> None:
    """Depth and raw instance values retain uint16 precision exactly."""
    depth = np.array([[1, 257, 65535], [2, 3, 4]], dtype=np.uint16)
    instance = np.array([[0, 257, 65535], [0, 0, 0]], dtype=np.uint16)
    scene = load_scene(write_scene(
        tmp_path / "scene", depth=depth, instance=instance,
        mapping={"0": "BACKGROUND", "257": "a", "65535": "b"},
    ), require_ground_truth=True)
    assert scene.depth.dtype == np.uint16
    assert scene.ground_truth.dtype == np.uint16
    np.testing.assert_array_equal(scene.depth, depth)
    np.testing.assert_array_equal(scene.ground_truth, instance)
    assert not scene.depth.flags.writeable


def test_mismatched_image_dimensions_are_rejected(tmp_path: Path) -> None:
    """RGB and depth must share dimensions."""
    path = write_scene(tmp_path / "scene", depth=np.ones((1, 3), dtype=np.uint16))
    with pytest.raises(DatasetValidationError, match="image_shape_mismatch"):
        load_scene(path)


def test_visible_instance_must_exist_in_mapping(tmp_path: Path) -> None:
    """A provided mapping must cover every visible nonzero ID."""
    instance = np.array([[0, 2, 0], [0, 0, 0]], dtype=np.uint16)
    path = write_scene(tmp_path / "scene", instance=instance, mapping={"0": "BACKGROUND"})
    with pytest.raises(DatasetValidationError, match="mapping_missing_id"):
        load_scene(path, require_ground_truth=True)


def test_unused_mapping_entry_is_warning(tmp_path: Path) -> None:
    """Mapping entries for fully occluded objects do not invalidate a scene."""
    instance = np.array([[0, 1, 0], [0, 0, 0]], dtype=np.uint16)
    path = write_scene(tmp_path / "scene", instance=instance,
                       mapping={"0": "BACKGROUND", "1": "a", "2": "b"})
    report = validate_scene(path, require_ground_truth=True)
    assert report.valid
    assert any(issue.code == "instance_mapping_unused_id" and issue.severity == "warning"
               for issue in report.issues)


def test_instance_pixel_with_zero_depth_is_error(tmp_path: Path) -> None:
    """Annotations cannot identify a surface where depth is invalid."""
    depth = np.full((2, 3), 1000, dtype=np.uint16)
    depth[0, 1] = 0
    instance = np.zeros((2, 3), dtype=np.uint16)
    instance[0, 1] = 1
    path = write_scene(tmp_path / "scene", depth=depth, instance=instance,
                       mapping={"0": "BACKGROUND", "1": "a"})
    report = validate_scene(path, require_ground_truth=True)
    assert not report.valid
    assert any(issue.code == "instance_on_invalid_depth" for issue in report.issues)


def test_optional_ground_truth_can_be_absent(tmp_path: Path) -> None:
    """Inference input loads without optional ground truth."""
    path = write_scene(tmp_path / "scene")
    assert load_scene(path).ground_truth is None
    report = validate_scene(path)
    assert report.valid
    assert any(issue.code == "ground_truth_missing" and issue.severity == "info"
               for issue in report.issues)


def test_required_ground_truth_must_be_present(tmp_path: Path) -> None:
    """Evaluation mode fails clearly when instance.png is absent."""
    path = write_scene(tmp_path / "scene")
    with pytest.raises(DatasetValidationError, match="ground_truth_missing"):
        load_scene(path, require_ground_truth=True)


def test_total_object_count_difference_is_informational(tmp_path: Path) -> None:
    """Scene object count may exceed the number of visible instances."""
    instance = np.array([[0, 1, 0], [0, 0, 0]], dtype=np.uint16)
    path = write_scene(tmp_path / "scene", instance=instance,
                       mapping={"0": "BACKGROUND", "1": "a"}, scene={"bag_count": 3})
    report = validate_scene(path, require_ground_truth=True)
    assert report.valid
    assert any(issue.code == "object_count_differs_from_visible" and
               issue.severity == "info" for issue in report.issues)


def test_missing_and_conflicting_depth_scale_are_rejected(tmp_path: Path) -> None:
    """Depth units are never inferred from stored numeric magnitude."""
    missing = isaac_camera()
    missing.pop("depth_scale")
    with pytest.raises(DatasetValidationError, match="depth scale is missing"):
        parse_isaac_camera_json(missing)
    path = write_scene(
        tmp_path / "scene",
        camera=capture_camera(),
        scene={"depth_scale": 1.0},
    )
    report = validate_scene(path)
    assert not report.valid
    assert "conflicting depth-scale" in report.issues[0].message


def test_pointcloud_is_not_read_and_scene_remains_unchanged(tmp_path: Path) -> None:
    """Loading ignores PLY data and performs no writes to the input tree."""
    path = write_scene(tmp_path / "scene")
    (path / "pointcloud.ply").write_bytes(b"not a valid PLY\x00\xff")
    before = {item.name: item.read_bytes() for item in path.iterdir()}
    scene = load_scene(path)
    after = {item.name: item.read_bytes() for item in path.iterdir()}
    assert scene.depth.shape == (2, 3)
    assert before == after
