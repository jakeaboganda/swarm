# map

The pure-Rust road-network model. The "compiled map" that map importers bake
into and every consumer (physics, the vehicle driver, rendering, agents) reads.
Right-handed, **Y-up, meters**, matching viz/Bevy. Format-agnostic: nothing here
knows OpenDRIVE or OSM, so swapping the importer never touches downstream code.

## Contents

- **`RoadNetwork`**: the baked map: a list of `Lane`s (road grouping + a
  routing graph arrive with the OpenDRIVE importer). `nearest_lane` /
  `driving_lanes` are the everyday queries; `surface_mesh` tessellates the road
  for the collider and the viewer.
- **`Lane`**: a drivable strip: centerline `Polyline` + width + direction. An
  agent lays a path down the centerline; the vehicle drives it.
- **`Polyline` / `Pose` / `Projection`**: arc-length geometry. `pose_at(s)`
  gives position + heading along a lane; `project(point)` gives the nearest
  point, its arc length, and the signed lateral offset (the lane-keeping error).
  Curves are pre-sampled to points, so this is all an importer has to produce.
- **`Mesh`**: positions + triangle indices for the road surface.
- **`demo_road()`**: a hand-authored straight-then-curve road on a grade, so
  the pipeline can be built and tested before the libOpenDRIVE importer exists.

Depends only on `glam`.
