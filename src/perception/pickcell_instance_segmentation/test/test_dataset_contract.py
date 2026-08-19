"""Opt-in contract test for an existing read-only dataset scene."""

import os
from pathlib import Path

import pytest

from pickcell_instance_segmentation import load_scene
from pickcell_instance_segmentation import validate_scene


def test_configured_real_scene_contract() -> None:
    """Validate and describe the scene named by PICKCELL_TEST_SCENE."""
    configured = os.environ.get("PICKCELL_TEST_SCENE")
    if not configured:
        pytest.skip("PICKCELL_TEST_SCENE is not set; real-scene contract not run")
    path = Path(configured)
    report = validate_scene(path)
    assert report.valid, report.issues
    scene = load_scene(path)
    details = {
        "color_shape": scene.color.shape,
        "color_dtype": str(scene.color.dtype),
        "depth_shape": scene.depth.shape,
        "depth_dtype": str(scene.depth.dtype),
        "camera_schema": scene.camera.source_schema,
        "depth_scale": scene.camera.depth_scale,
        "visible_instance_ids": report.visible_instance_ids,
    }
    print(f"PICKCELL_TEST_SCENE contract: {details}")
