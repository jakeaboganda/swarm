# map

The pure-Rust road-network model. The "compiled map" that map importers bake
into and every consumer (physics, the vehicle driver, rendering, agents)
reads. Right-handed, **Y-up, meters**, matching viz/Bevy. Format-agnostic.
Nothing here knows OpenDRIVE or OSM, so swapping the importer never touches
downstream code.

## Contents

- **`RoadNetwork`**. The baked map: a list of `Lane`s plus a lane
  connectivity graph. `nearest_lane` and `driving_lanes` are the everyday
  queries; `successors`, `predecessors`, and `neighbors` walk the graph;
  `route(from, to)` finds a lane path and samples it to a plan;
  `surface_mesh` tessellates the road for the collider and the viewer.
- **`Lane`**. A drivable strip: centerline `Polyline` plus width and
  direction, with its graph links (`successors`, `predecessors`,
  `neighbors`). An agent lays a path down the centerline; the vehicle drives
  it.
- **`Polyline` / `Pose` / `Projection`**. Arc-length geometry. `pose_at(s)`
  gives position and heading along a lane. `project(point)` gives the
  nearest point, its arc length, and the signed lateral offset (the
  lane-keeping error). Curves are pre-sampled to points, so this is all an
  importer has to produce.
- **`Mesh`**. Positions plus triangle indices for the road surface.
- **`demo_road()`**. A hand-authored straight-then-curve road on a grade, so
  the pipeline could be built and tested before the libOpenDRIVE importer
  existed.

Depends only on `glam`.
