#!/usr/bin/env python3
# flake8: noqa: E402
"""Import PickCell, render an overhead depth camera, and publish ROS 2 data."""

import argparse
from pathlib import Path


def parse_arguments():
    """Parse arguments before starting Isaac Sim."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--usd-dir", required=True)
    parser.add_argument("--pickcell-description", required=True)
    parser.add_argument("--kuka-description", required=True)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--test-steps", type=int, default=0)
    return parser.parse_known_args()


args, _unknown = parse_arguments()

from isaacsim import SimulationApp


simulation_app = SimulationApp({
    "renderer": "RayTracedLighting",
    "headless": args.headless,
})

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.core.experimental.utils.transform as transform_utils
import numpy as np
import omni
import omni.graph.core as og
import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as sd
from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.sensors.camera import Camera


FRAME_ID = "overhead_depth_camera_optical_frame"
NAMESPACE = "pickcell"
DEPTH_TOPIC = "sensors/overhead_depth/depth/image_raw"
INFO_TOPIC = "sensors/overhead_depth/camera_info"
POINTS_TOPIC = "sensors/overhead_depth/points"
CAMERA_PATH = "/World/overhead_depth_camera"
SIMULATION_RATE_HZ = 60
CAMERA_RATE_HZ = 15


def import_robot() -> str:
    """Convert the expanded workspace URDF to a USD stage."""
    config = URDFImporterConfig()
    config.urdf_path = str(Path(args.urdf).resolve())
    config.usd_path = str(Path(args.usd_dir).resolve())
    config.fix_base = True
    config.joint_target_type = "position"
    config.ros_package_paths = [
        {
            "name": "pickcell_description",
            "path": str(Path(args.pickcell_description).resolve()),
        },
        {
            "name": "kuka_quantec_support",
            "path": str(Path(args.kuka_description).resolve()),
        },
    ]
    output = URDFImporter(config).import_urdf()
    if not output:
        raise RuntimeError("Isaac Sim failed to import the PickCell URDF")
    return output


def create_clock_graph() -> None:
    """Publish Isaac simulation time on the standard /clock topic."""
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/PickCellClock", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ],
            keys.SET_VALUES: [
                ("PublishClock.inputs:topicName", "/clock"),
            ],
        },
    )


def initialize_writer(writer_name: str, topic: str, render_product: str):
    """Attach an image-derived ROS writer to the camera render product."""
    writer = rep.writers.get(writer_name)
    writer.initialize(
        frameId=FRAME_ID,
        nodeNamespace=NAMESPACE,
        queueSize=1,
        topicName=topic,
    )
    writer.attach([render_product])
    return writer


def publish_camera_info(render_product: str) -> None:
    """Publish pinhole calibration corresponding to the rendered camera."""
    # This module becomes available only after enabling the ROS 2 extension.
    from isaacsim.ros2.core import read_camera_info

    camera_info, _ = read_camera_info(render_product_path=render_product)
    writer = rep.writers.get("ROS2PublishCameraInfo")
    writer.initialize(
        frameId=FRAME_ID,
        nodeNamespace=NAMESPACE,
        queueSize=1,
        topicName=INFO_TOPIC,
        width=camera_info.width,
        height=camera_info.height,
        projectionType=camera_info.distortion_model,
        k=camera_info.k.reshape([1, 9]),
        r=camera_info.r.reshape([1, 9]),
        p=camera_info.p.reshape([1, 12]),
        physicalDistortionModel=camera_info.distortion_model,
        physicalDistortionCoefficients=camera_info.d,
    )
    writer.attach([render_product])


def main() -> None:
    """Run Isaac until the application or parent ROS launch exits."""
    app_utils.enable_extension("isaacsim.ros2.bridge")
    simulation_app.update()

    usd_path = import_robot()
    omni.usd.get_context().open_stage(usd_path, None)
    while omni.usd.get_context().get_stage_loading_status()[2] > 0:
        simulation_app.update()
    stage_utils.set_stage_units(meters_per_unit=1.0)

    # Camera uses the world/ROS body-axis convention. Pitching +90 degrees
    # aligns its optical +Z axis with cell -Z, directly above the pallet.
    camera = Camera(
        prim_path=CAMERA_PATH,
        position=np.array([1.4, 0.0, 2.0]),
        orientation=transform_utils.euler_angles_to_quaternion(
            np.array([0.0, 90.0, 0.0]), degrees=True
        ).numpy(),
        frequency=CAMERA_RATE_HZ,
        resolution=(640, 480),
    )
    camera.initialize()
    camera.set_clipping_range(0.1, 3.0)
    camera.set_horizontal_aperture(20.955)
    camera.set_focal_length(13.656)
    simulation_app.update()

    render_product = camera._render_product_path
    depth_rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
        sd.SensorType.DistanceToImagePlane.name
    )
    initialize_writer(depth_rv + "ROS2PublishImage", DEPTH_TOPIC, render_product)
    initialize_writer(depth_rv + "ROS2PublishPointCloud", POINTS_TOPIC, render_product)
    publish_camera_info(render_product)
    camera_step = SIMULATION_RATE_HZ // CAMERA_RATE_HZ
    depth_gate = omni.syntheticdata.SyntheticData._get_node_path(
        depth_rv + "IsaacSimulationGate", render_product
    )
    info_gate = omni.syntheticdata.SyntheticData._get_node_path(
        "PostProcessDispatchIsaacSimulationGate", render_product
    )
    og.Controller.attribute(depth_gate + ".inputs:step").set(camera_step)
    og.Controller.attribute(info_gate + ".inputs:step").set(camera_step)
    create_clock_graph()

    SimulationManager.setup_simulation(
        dt=1.0 / SIMULATION_RATE_HZ, device="cpu"
    )
    app_utils.play()
    simulation_app.update()
    steps = 0
    while simulation_app.is_running():
        simulation_app.update()
        steps += 1
        if args.test_steps and steps >= args.test_steps:
            break
    app_utils.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()
