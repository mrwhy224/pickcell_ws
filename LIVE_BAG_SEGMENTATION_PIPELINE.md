# Live Bag Segmentation Pipeline

This document explains the implemented path from Isaac Sim RGB-D topics to
surface patches, affinity-connected candidate regions, published candidate
point clouds, deterministic next-pick selection, and RViz overlays.

## Current capability and terminology

The implemented pipeline is:

```text
RGB + depth + camera calibration
  -> organized XYZRGB point cloud
  -> depth edges and surface normals
  -> small surface patches
  -> patch adjacency graph and edge features
  -> deterministic patch affinity
  -> connected candidate instances
  -> upper-then-right next-pick selection
  -> candidate and selected PointCloud2 messages
  -> live visualization
```

The output called `candidate_bag_overlay` is not yet a semantic guarantee that
every colored region is a chemical bag. It is a collection of neighboring
surface patches that the geometric affinity rules consider part of the same
surface or object. Pallet, floor, robot, or background surfaces can therefore
also appear as candidates. A bag/background classifier and workspace filtering
are still needed before the output should be called confirmed bag detection.

Ground-truth `instance.png` images and the files under
`/home/mahdi/pickcell_phase7_evaluation/visualizations` are used only for
offline evaluation. Ground truth is not provided to the live pipeline.

## Source-file map

| Responsibility | File |
|---|---|
| System configuration and topic parameters | `src/foundation/pickcell_config/config/application.yaml` |
| Launch all configured nodes | `src/foundation/pickcell_bringup/launch/system.launch.py` |
| Isaac camera and ROS publishers | `src/backends/pickcell_isaac_sim/scripts/run_pickcell_scene.py` |
| Live ROS synchronization and overlays | `src/perception/pickcell_instance_segmentation/pickcell_instance_segmentation/live_node.py` |
| Public in-memory pipeline | `src/perception/pickcell_instance_segmentation/pickcell_instance_segmentation/pipeline.py` |
| Immutable data/configuration contracts | `src/perception/pickcell_instance_segmentation/pickcell_instance_segmentation/models.py` |
| RGB-D projection | `src/perception/pickcell_instance_segmentation/pickcell_instance_segmentation/projection.py` |
| Depth edges and normals | `src/perception/pickcell_instance_segmentation/pickcell_instance_segmentation/normals.py` |
| Surface-patch segmentation | `src/perception/pickcell_instance_segmentation/pickcell_instance_segmentation/patches.py` |
| Patch graph and its 18 features | `src/perception/pickcell_instance_segmentation/pickcell_instance_segmentation/adjacency.py` |
| Patch-edge affinity decisions | `src/perception/pickcell_instance_segmentation/pickcell_instance_segmentation/affinity.py` |
| Connected candidate components | `src/perception/pickcell_instance_segmentation/pickcell_instance_segmentation/partition.py` |
| Candidate measurement and pick ordering | `src/perception/pickcell_instance_segmentation/pickcell_instance_segmentation/selection.py` |
| ROS PointCloud2 conversion | `src/perception/pickcell_instance_segmentation/pickcell_instance_segmentation/ros_cloud.py` |
| RViz displays | `src/foundation/pickcell_description/rviz/pickcell.rviz` |

## 1. Launch and node configuration

The system starts with:

```bash
cd /home/mahdi/pickcell_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch pickcell_bringup system.launch.py
```

`system.launch.py` reads `application.yaml`, applies the `pickcell` namespace,
and starts every entry in the `nodes` mapping. The live node is configured as:

```yaml
live_instance_segmentation:
  mode: live
  package: pickcell_instance_segmentation
  executable: live_instance_segmentation
  parameters:
    rgb_topic: sensors/overhead_depth/color/image_raw
    depth_topic: sensors/overhead_depth/depth/image_raw
    camera_info_topic: sensors/overhead_depth/camera_info
    patch_overlay_topic: perception/patch_overlay
    instance_overlay_topic: perception/candidate_bag_overlay
    segmented_cloud_topic: perception/candidate_bag_cloud
    selected_cloud_topic: perception/selected_bag_cloud
    selected_point_topic: perception/selected_bag_point
    maximum_processing_rate_hz: 2.0
    minimum_candidate_pixels: 100
    same_height_tolerance_m: 0.02
    top_depth_percentile: 10.0
    synchronization_slop_seconds: 0.05
    input_is_rectified: true
```

The resulting absolute topics are:

```text
/pickcell/sensors/overhead_depth/color/image_raw
/pickcell/sensors/overhead_depth/depth/image_raw
/pickcell/sensors/overhead_depth/camera_info
/pickcell/perception/patch_overlay
/pickcell/perception/candidate_bag_overlay
/pickcell/perception/candidate_bag_cloud
/pickcell/perception/selected_bag_cloud
/pickcell/perception/selected_bag_point
```

The simulated camera publishes at about 15 Hz. Processing is deliberately
limited to 2 Hz because a full 640x480 geometric frame is relatively expensive.

## 2. Isaac Sim camera output

`run_pickcell_scene.py` creates an overhead camera and attaches ROS writers for:

```text
RGB image:   /pickcell/sensors/overhead_depth/color/image_raw
Depth image: /pickcell/sensors/overhead_depth/depth/image_raw
Calibration: /pickcell/sensors/overhead_depth/camera_info
Raw cloud:   /pickcell/sensors/overhead_depth/points
```

RGB and depth are produced by the same render product and are pixel aligned.
Therefore RGB pixel `(v, u)` and depth pixel `(v, u)` describe the same camera
ray. This alignment is essential for building an XYZRGB point cloud.

## 3. ROS message synchronization

`LiveInstanceSegmentation` is implemented in `live_node.py`. It stores the most
recent `CameraInfo` and uses `ApproximateTimeSynchronizer` for RGB and depth:

```python
self._synchronizer = ApproximateTimeSynchronizer(
    [rgb, depth],
    queue_size=3,
    slop=0.05,
)
```

An RGB and depth message can form a pair when their timestamps differ by at
most 0.05 seconds. `_on_rgbd()` receives each synchronized pair. Frames arriving
faster than `maximum_processing_rate_hz` are skipped.

## 4. ROS images become NumPy arrays

The callback uses `cv_bridge`:

```python
color = bridge.imgmsg_to_cv2(rgb_message, "rgb8")
depth = bridge.imgmsg_to_cv2(depth_message, "passthrough")
```

The color contract is `H x W x 3`, `uint8`, RGB. Two depth forms are supported:

```text
float32: depth in metres, depth_scale = 1.0
uint16:  depth in millimetres, depth_scale = 1000.0
```

NaN and infinite floating-depth values become zero. Zero means no valid depth.

## 5. Camera model

The ROS `CameraInfo.k` array represents:

```text
K = [fx  0  cx
      0 fy  cy
      0  0   1]
```

`live_node.py` constructs the `CameraModel` defined in `models.py` with width,
height, `fx`, `fy`, `cx`, `cy`, depth scale, distortion, and optical frame ID.

## 6. Public processing entry point

The reusable API is `process_rgbd_frame()` in `pipeline.py`:

```python
result = process_rgbd_frame(
    color,
    depth,
    camera,
    input_is_rectified=True,
)
```

It runs these stages in exact order:

```python
cloud = rgbd_to_organized_cloud(...)
geometry = compute_organized_geometry(...)
patches = segment_surface_patches(...)
graph = build_patch_graph(...)
affinity = predictor.predict(graph)
```

`RGBDProcessingResult` contains:

```python
result.scene
result.cloud
result.geometry
result.patches
result.graph
result.affinity
```

## 7. RGB-D to organized XYZRGB cloud

`rgbd_to_organized_cloud()` is implemented in `projection.py`. It validates:

- color is `H x W x 3 uint8`;
- depth is `H x W uint16` or `H x W float32`;
- RGB, depth, and camera resolutions agree;
- depth is finite and nonnegative;
- focal lengths and depth scale are positive.

For each pixel `(v, u)`:

```text
Z = stored_depth / depth_scale
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
```

The result stays organized:

```text
cloud.xyz:   H x W x 3, float32, metres
cloud.rgb:   H x W x 3, uint8
cloud.valid: H x W, boolean
```

Invalid points are `[NaN, NaN, NaN]`. The optical coordinate convention is
positive X right, positive Y down, and positive Z forward from the camera.

## 8. Depth-edge detection

`compute_depth_edges()` in `normals.py` compares valid neighboring depths:

```text
difference = abs(Za - Zb)
limit = max(absolute_depth_jump_m,
            relative_depth_jump * min(Za, Zb))
```

Defaults from `NormalEstimationConfig` are:

```text
absolute_depth_jump_m = 0.02 m
relative_depth_jump   = 0.02
```

If the difference exceeds the limit, both endpoints are marked as depth-edge
pixels. This finds bag silhouettes, gaps between stacked bags, pallet edges,
occlusion boundaries, and other sudden depth transitions.

## 9. Surface-normal estimation

`estimate_organized_normals()` in `normals.py` forms two 3D tangent vectors:

```text
horizontal = right point - left point
vertical   = bottom point - top point
normal     = normalize(horizontal x vertical)
```

A normal is accepted only when the center and tangent paths are valid, do not
cross a large depth jump, have sufficient baseline, and produce a finite,
non-degenerate cross product. Normals are oriented toward the camera.

The outputs are:

```python
result.geometry.normals
result.geometry.normal_valid
result.geometry.depth_edges
result.geometry.normal_variation
```

Normal variation averages `1 - dot(normal_a, normal_b)` across safe neighbors.
Small variation indicates a smooth surface; large variation indicates curvature
or a possible boundary.

## 10. Conservative surface-patch segmentation

`segment_surface_patches()` is implemented in `patches.py`. Default settings
from `PatchSegmentationConfig` are:

```text
tile size                  = 12 pixels
connectivity               = 4
maximum normal angle       = 20 degrees
absolute point jump        = 0.03 m
relative point jump        = 0.02
maximum color distance     = disabled
valid normals required     = false
```

Two neighboring pixels may join when:

1. Both have valid XYZ points.
2. Their 3D distance is below the absolute/relative limit.
3. Neither endpoint is a depth-edge pixel.
4. If both normals exist, their unsigned angle is at most 20 degrees.
5. If color gating is enabled, their RGB distance is below its limit.
6. Both pixels are inside the same 12x12 tile.

Union-find merges accepted pixel neighbors inside each tile. The tile boundary
is intentional: it creates many small patches rather than prematurely merging
an entire object. Patch labels use `-1` for invalid depth and `0, 1, 2, ...`
for valid patches.

For every patch, the code records pixel count, XYZ centroid, mean RGB, mean
normal, fraction of valid normals, image bounding box, and source tile index.

## 11. Patch adjacency graph

`build_patch_graph()` in `adjacency.py` makes every patch a graph node. An edge
is created only when pixels of two different patches touch in image space.

Every graph edge has 18 features:

1. `shared_boundary_count`
2. `diagonal_contact_count`
3. `shared_boundary_fraction_min_patch`
4. `centroid_distance_m`
5. `centroid_depth_difference_m`
6. `patch_size_ratio`
7. `mean_boundary_point_distance_m`
8. `max_boundary_point_distance_m`
9. `mean_boundary_depth_difference_m`
10. `max_boundary_depth_difference_m`
11. `depth_edge_endpoint_fraction`
12. `normal_pair_valid_fraction`
13. `mean_unsigned_normal_angle_deg`
14. `max_unsigned_normal_angle_deg`
15. `mean_boundary_rgb_distance`
16. `max_boundary_rgb_distance`
17. `mean_patch_rgb_distance`
18. `cross_tile_boundary_fraction`

These features measure boundary support, 3D continuity, depth continuity,
surface orientation, edge evidence, color similarity, relative patch size, and
whether an artificial tile boundary separated the patches.

## 12. Patch-edge affinity

`predict_patch_affinity()` in `affinity.py` is deterministic and rule based; it
is not a neural network. Default `PatchAffinityConfig` thresholds are:

```text
maximum mean boundary point distance = 0.025 m
maximum boundary depth difference    = 0.025 m
maximum centroid depth difference    = 0.08 m
maximum unsigned normal angle        = 35 degrees
maximum depth-edge fraction          = 0.50
minimum shared-boundary fraction     = 0.03
minimum valid-normal fraction        = 0.20
decision threshold                   = 0.50
```

Hard rejection gates are applied in this order:

1. Excessive boundary point distance.
2. Excessive boundary depth difference.
3. Excessive centroid depth difference.
4. Excessive depth-edge evidence.
5. Excessive normal-angle evidence.
6. Insufficient boundary support.
7. Insufficient valid-normal evidence, when required.
8. Excessive RGB distance, when enabled.

For surviving edges, boundary support, point continuity, depth continuity,
centroid-depth compatibility, lack of depth-edge evidence, available normal
agreement, and optional color agreement are mapped into `[0, 1]` compatibility
components and averaged. An edge is marked `same_object` when it survives all
hard gates and its score is at least 0.50.

Outputs are:

```python
result.affinity.scores
result.affinity.same_object
result.affinity.hard_rejected
result.affinity.rejection_codes
```

Each entry corresponds to the same row in `result.graph.edges`.

## 13. Connected candidate instances

`connected_instance_labels()` in `partition.py` uses only graph edges for which
`affinity.same_object` is true. Union-find merges their patch nodes. Connectivity
is transitive: if A joins B and B joins C, A, B, and C become one candidate.

Component pixel counts are summed, and regions smaller than
`minimum_candidate_pixels` (currently 100) are removed. The returned `H x W`
`int32` map uses:

```text
0 = invalid, rejected, or too small
1 = candidate instance 1
2 = candidate instance 2
...
```

## 14. Separating candidate point clouds

The candidate-label image is pixel aligned with the organized point cloud. A
single candidate can therefore be extracted exactly:

```python
candidate_labels = connected_instance_labels(
    result.patches,
    result.graph,
    result.affinity,
    minimum_pixels=100,
)

mask = (candidate_labels == 1) & result.cloud.valid
candidate_1_xyz = result.cloud.xyz[mask]  # N x 3
candidate_1_rgb = result.cloud.rgb[mask]  # N x 3
```

Extract all candidates with:

```python
candidate_clouds = {}

for instance_id in np.unique(candidate_labels):
    if instance_id == 0:
        continue
    mask = (candidate_labels == instance_id) & result.cloud.valid
    candidate_clouds[int(instance_id)] = {
        "xyz": result.cloud.xyz[mask],
        "rgb": result.cloud.rgb[mask],
    }
```

This is the exact boundary where one organized scene cloud becomes multiple
candidate clouds. The live node publishes all positive candidates together as
one compact `PointCloud2`; its `instance_id` field preserves the separation.
It separately publishes the chosen candidate as another `PointCloud2`.

## 15. Choosing the next bag

`UpperRightSegmentSelector` in `selection.py` implements the requested order
for an overhead camera. It uses a configurable percentile of each segment's
optical depth as a robust top-surface measurement. The smallest depth is the
upper/closest segment. Every segment within `same_height_tolerance_m` of that
depth is considered level, and the segment with the largest image-column
centroid (the rightmost one) is chosen. Instance ID is the final deterministic
tie breaker.

The selected centroid and point cloud remain in the camera optical frame. The
next grasp-planning stage must transform them into the robot planning frame and
compute a collision-checked end-effector pose.

## 16. Live overlays and RViz

`_colorize()` in `live_node.py` assigns deterministic colors to integer labels,
blends 65% label color with 35% original RGB, and draws white boundaries.

Patch visualization uses:

```python
_colorize(result.patches.labels + 1, color)
```

Candidate visualization uses:

```python
_colorize(candidate_labels, color)
```

The node publishes `rgb8` ROS images with the original camera timestamp and
frame. The selected candidate is boxed and marked `NEXT`. RViz displays the
raw cloud, the instance-colored candidate cloud, and the selected bag cloud.
The selected centroid is also published as a `PointStamped` for downstream
planning.

## 17. Ground truth versus live inference

Offline evaluation has `instance.png` where zero is background and positive IDs
identify known bags. It can compare predicted patch relationships against truth.
Live inference has no `instance.png`, so it sees only RGB, depth, and calibration.

```text
Offline evaluation: prediction + known bag IDs -> metrics and error images
Live processing:    prediction only          -> geometric candidates
```

## 18. Work needed for confirmed bag detection and grasp execution

Dependable bag-only clouds require an additional semantic/filtering stage:

1. Restrict processing to the pallet/workspace ROI.
2. Remove the pallet or support plane.
3. Remove robot points.
4. Check component dimensions, height, area, and volume against expected bags.
5. Validate filled-bag shape, potentially with an RGB or RGB-D learned model.
6. Track stable bag IDs across video frames.
7. Transform results from camera coordinates into the `cell` frame.
8. Compute and collision-check an executable grasp pose.

Until this stage exists, the accurate API/output name is
`candidate_bag_point_clouds`, not `bags_point_cloud`.
