# map-opendrive

A pure-Rust **OpenDRIVE (`.xodr`) importer** that bakes a real map into
`map::RoadNetwork` at load. Depends on `map` + `roxmltree`; geometry
cross-checked against the reference C++
[libOpenDRIVE](https://github.com/pageldev/libOpenDRIVE).

## What it imports

- **Reference geometry**: `line`, `arc`, `spiral` (clothoid), `paramPoly3`,
  `poly3`.
- **Elevation** profile, and **superelevation** (banked curves) baked as a real
  cant: the cross-section rolls about the reference line, so an outer lane rides
  higher and a vehicle leans into the bank.
- **Lanes**: per-lane widths, `laneOffset`, and multiple lane sections.
- **Connectivity**: road/lane `<link>`s and `<junction>`s resolved into each
  `Lane`'s `successors`/`predecessors` (the `links` module), which `map`'s
  router then walks.

## API

- **`load_file(path)`** / **`load_str(xml)`** → `Result<RoadNetwork,
  ImportError>`.

Curved geometry is baked to points at a fixed arc-length step, so consumers only
ever sample a polyline.

## Coordinate mapping

OpenDRIVE is right-handed **Z-up**; our world is **Y-up**. `(x, y, elev)` maps to
`(x, elev, -y)`, so an OpenDRIVE left turn curves toward our -Z, matching the
hand-authored `demo_road`.

## Not yet

`<lateralProfile>` `<shape>` (per-`t` crowning/camber).
