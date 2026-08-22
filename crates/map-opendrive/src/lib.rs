//! A small, pure-Rust OpenDRIVE (`.xodr`) importer that bakes a road into
//! `map::RoadNetwork`. It implements `line`, `arc`, `spiral` (clothoid),
//! `paramPoly3`, and `poly3` reference geometry; an elevation profile; per-lane
//! widths; `laneOffset`; multiple lane sections; and **drive-direction lane
//! connectivity** -- road/lane `<link>`s and `<junction>`s resolved into each
//! `Lane`'s `successors`/`predecessors` (see `links`). Not yet: a higher-level
//! routing/pathfinding API over the graph (see the DECISIONS "roll our own"
//! note). Anything richer is future work.
//!
//! ## Coordinate mapping
//! OpenDRIVE is right-handed **Z-up** (reference line in the X-Y plane, `hdg`
//! the heading in that plane, elevation along +Z). Our world is **Y-up** with
//! the ground in X-Z. We map `(x_od, y_od, elev)` -> `(x_od, elev, -y_od)`, so a
//! left turn in OpenDRIVE (increasing heading) curves toward our -Z, matching
//! the hand-authored `demo_road`.

use glam::Vec3;
use map::{Direction, Lane, LaneId, LaneKind, Polyline, RoadNetwork};

mod links;
use links::{LaneMeta, RoadInfo, Topology};

/// Arc-length spacing (meters) at which curved geometry is baked to points.
const SAMPLE_STEP: f64 = 2.0;

#[derive(Debug, thiserror::Error)]
pub enum ImportError {
    #[error("invalid OpenDRIVE XML: {0}")]
    Xml(#[from] roxmltree::Error),
    #[error("malformed OpenDRIVE: {0}")]
    Malformed(String),
}

/// Load an OpenDRIVE file from disk and bake it into a `RoadNetwork`.
pub fn load_file(path: impl AsRef<std::path::Path>) -> Result<RoadNetwork, ImportError> {
    let xml = std::fs::read_to_string(path.as_ref())
        .map_err(|e| ImportError::Malformed(format!("reading {:?}: {e}", path.as_ref())))?;
    load_str(&xml)
}

/// Bake an OpenDRIVE document (as a string) into a `RoadNetwork`.
pub fn load_str(xml: &str) -> Result<RoadNetwork, ImportError> {
    let cleaned = sanitize(xml);
    let doc = roxmltree::Document::parse(cleaned.as_ref())?;
    let root = doc.root_element();
    let mut lanes = Vec::new();
    let mut topo = Topology::default();
    for road in root.children().filter(|n| n.has_tag_name("road")) {
        parse_road(road, &mut lanes, &mut topo);
    }
    if lanes.is_empty() {
        return Err(ImportError::Malformed("no driving lanes found".into()));
    }
    // Resolve connectivity once all lanes exist and are registered.
    topo.junctions = links::junctions(root);
    links::resolve(&mut lanes, &topo);
    Ok(RoadNetwork { lanes })
}

/// Make a real-world document parseable: strip a UTF-8 BOM and remove the
/// `<?xml ... ?>` declaration. Tools (e.g. CARLA) emit a license comment
/// *before* the declaration, which is malformed XML that strict parsers reject;
/// the declaration only names version/encoding, which we don't need for UTF-8.
fn sanitize(xml: &str) -> std::borrow::Cow<'_, str> {
    let xml = xml.trim_start_matches('\u{feff}');
    if let Some(start) = xml.find("<?xml") {
        if let Some(rel_end) = xml[start..].find("?>") {
            let mut out = String::with_capacity(xml.len());
            out.push_str(&xml[..start]);
            out.push_str(&xml[start + rel_end + 2..]);
            return std::borrow::Cow::Owned(out);
        }
    }
    std::borrow::Cow::Borrowed(xml)
}

// --- Reference-line geometry --------------------------------------------------

enum Geom {
    Line,
    Arc {
        curvature: f64,
    },
    /// A curve with no elementary arc-length form (spiral/clothoid or
    /// paramPoly3), baked to `(ds, x, y, hdg)` samples at load -- see
    /// `bake_spiral` / `bake_param_poly3`. `pose` interpolates them by arc
    /// length `ds`.
    Baked {
        samples: Vec<(f64, f64, f64, f64)>,
    },
}

/// Bake a clothoid to fine `(ds, x, y, hdg)` samples. Curvature varies linearly
/// `curv_start -> curv_end` over `length`, so heading is the closed form
/// `hdg0 + curv_start*u + (c_dot/2)*u^2`; position is its running integral,
/// which has no elementary form, so integrate cos/sin(heading) by the midpoint
/// rule at a fine step (mm-accurate over hundreds of meters).
fn bake_spiral(
    x0: f64,
    y0: f64,
    hdg0: f64,
    curv_start: f64,
    curv_end: f64,
    length: f64,
) -> Vec<(f64, f64, f64, f64)> {
    const STEP: f64 = 0.25;
    let c_dot = if length > 1e-9 {
        (curv_end - curv_start) / length
    } else {
        0.0
    };
    let heading = |u: f64| hdg0 + curv_start * u + 0.5 * c_dot * u * u;
    let (mut x, mut y, mut u) = (x0, y0, 0.0);
    let mut out = vec![(0.0, x0, y0, hdg0)];
    while u < length - 1e-9 {
        let step = STEP.min(length - u);
        let theta_mid = heading(u + step / 2.0);
        x += theta_mid.cos() * step;
        y += theta_mid.sin() * step;
        u += step;
        out.push((u, x, y, heading(u)));
    }
    out
}

/// Evaluate `a + b*p + c*p^2 + d*p^3`.
fn eval_poly3([a, b, c, d]: [f64; 4], p: f64) -> f64 {
    a + b * p + c * p * p + d * p * p * p
}

/// Bake a paramPoly3 to `(ds, x, y, hdg)` samples. The curve is given in a local
/// `(u, v)` frame (u forward, v left of `hdg0`) by two cubics in a parameter
/// `p`; `p_max` is `1` for `pRange="normalized"`, else the geometry length.
/// Because `p` is not arc length, we step `p` finely, transform each point into
/// world space, and accumulate the true arc length `ds` so `pose` can sample by
/// distance like every other geometry.
fn bake_param_poly3(
    x0: f64,
    y0: f64,
    hdg0: f64,
    u: [f64; 4],
    v: [f64; 4],
    p_max: f64,
    length: f64,
) -> Vec<(f64, f64, f64, f64)> {
    let (sin, cos) = hdg0.sin_cos();
    let world = |p: f64| {
        let (uu, vv) = (eval_poly3(u, p), eval_poly3(v, p));
        (x0 + uu * cos - vv * sin, y0 + uu * sin + vv * cos)
    };
    let heading = |p: f64| {
        let du = u[1] + 2.0 * u[2] * p + 3.0 * u[3] * p * p;
        let dv = v[1] + 2.0 * v[2] * p + 3.0 * v[3] * p * p;
        hdg0 + dv.atan2(du)
    };
    // ~0.25 m resolution, from the geometry length.
    let n = ((length / 0.25).ceil() as usize).max(8);
    let (mut px, mut py) = world(0.0);
    let mut ds = 0.0;
    let mut out = vec![(0.0, px, py, heading(0.0))];
    for k in 1..=n {
        let p = p_max * (k as f64) / (n as f64);
        let (x, y) = world(p);
        ds += ((x - px).powi(2) + (y - py).powi(2)).sqrt();
        out.push((ds, x, y, heading(p)));
        (px, py) = (x, y);
    }
    out
}

/// One `<geometry>` record: its start pose on the reference line plus its shape.
struct GeomRec {
    s: f64,
    x: f64,
    y: f64,
    hdg: f64,
    geom: Geom,
}

impl GeomRec {
    /// Position `(x, y)` and heading at road arc-length `s` within this record.
    fn pose(&self, s: f64) -> (f64, f64, f64) {
        let ds = s - self.s;
        match &self.geom {
            Geom::Arc { curvature } if curvature.abs() > 1e-9 => {
                let h = self.hdg + curvature * ds;
                let x = self.x + (h.sin() - self.hdg.sin()) / curvature;
                let y = self.y - (h.cos() - self.hdg.cos()) / curvature;
                (x, y, h)
            }
            Geom::Baked { samples } => {
                let max = samples.last().map(|p| p.0).unwrap_or(0.0);
                let ds = ds.clamp(0.0, max);
                let i = samples
                    .partition_point(|p| p.0 <= ds)
                    .saturating_sub(1)
                    .min(samples.len().saturating_sub(2));
                let (a_s, ax, ay, ah) = samples[i];
                let (b_s, bx, by, bh) = samples[i + 1];
                let t = if b_s > a_s {
                    (ds - a_s) / (b_s - a_s)
                } else {
                    0.0
                };
                (ax + (bx - ax) * t, ay + (by - ay) * t, ah + (bh - ah) * t)
            }
            // A line, or a degenerate (straight) arc.
            _ => (
                self.x + ds * self.hdg.cos(),
                self.y + ds * self.hdg.sin(),
                self.hdg,
            ),
        }
    }
}

/// One cubic record (elevation, or a lane width), evaluated relative to its
/// own start offset: `a + b*ds + c*ds^2 + d*ds^3`.
struct Cubic {
    start: f64,
    a: f64,
    b: f64,
    c: f64,
    d: f64,
}

impl Cubic {
    fn eval(&self, s: f64) -> f64 {
        let ds = s - self.start;
        self.a + self.b * ds + self.c * ds * ds + self.d * ds * ds * ds
    }
}

/// The record whose start is the greatest not exceeding `s` (records sorted by
/// start). `None` if `s` precedes them all / the list is empty.
fn active(records: &[Cubic], s: f64) -> Option<&Cubic> {
    records.iter().rev().find(|r| r.start <= s + 1e-9)
}

struct LaneDef {
    id: i32,
    widths: Vec<Cubic>,
    pred_link: Option<i32>,
    succ_link: Option<i32>,
}

// --- Parsing ------------------------------------------------------------------

/// A numeric attribute, or `None` if it is absent, unparseable, or non-finite.
///
/// The finiteness check is not paranoia: Rust's float parser accepts the
/// literal `NaN`, and turns an out-of-range exponent (`1e400`) into infinity,
/// so an XML attribute carries either straight into the baked geometry. One
/// such value poisons every point derived from it, and a NaN vertex makes the
/// road's physics trimesh impossible to build.
fn attr_f64(node: roxmltree::Node, name: &str) -> Option<f64> {
    node.attribute(name)
        .and_then(|s| s.parse::<f64>().ok())
        .filter(|v| v.is_finite())
}

/// Bakes one `<road>`'s lanes into `out`.
///
/// A road the importer cannot interpret -- no length, no `<planView>`, no
/// supported geometry, no `<lanes>` -- is *skipped*, not fatal: real exports
/// carry the occasional junk road, and losing a whole city map to one of them
/// is the worse failure. Individual malformed lanes are already skipped the
/// same way. `load_str` still errors if the document as a whole yielded no
/// lanes at all, so a thoroughly broken file is never silently accepted.
fn parse_road(road: roxmltree::Node, out: &mut Vec<Lane>, topo: &mut Topology) {
    let road_id = road.attribute("id").unwrap_or_default().to_string();
    let Some(length) = attr_f64(road, "length") else {
        return;
    };

    let Some(plan_view) = child(road, "planView") else {
        return;
    };
    let mut geoms: Vec<GeomRec> = Vec::new();
    for g in plan_view.children().filter(|n| n.has_tag_name("geometry")) {
        // A geometry record missing (or carrying a non-finite) pose is skipped
        // rather than baked: the rest of the road is still usable.
        let (Some(s), Some(x), Some(y), Some(hdg), Some(length)) = (
            attr_f64(g, "s"),
            attr_f64(g, "x"),
            attr_f64(g, "y"),
            attr_f64(g, "hdg"),
            attr_f64(g, "length"),
        ) else {
            continue;
        };
        let geom = if let Some(arc) = child(g, "arc") {
            let Some(curvature) = attr_f64(arc, "curvature") else {
                continue;
            };
            Geom::Arc { curvature }
        } else if let Some(sp) = child(g, "spiral") {
            let (Some(curv_start), Some(curv_end)) =
                (attr_f64(sp, "curvStart"), attr_f64(sp, "curvEnd"))
            else {
                continue;
            };
            Geom::Baked {
                samples: bake_spiral(x, y, hdg, curv_start, curv_end, length),
            }
        } else if let Some(pp) = child(g, "paramPoly3") {
            let coeff = |n: &str| attr_f64(pp, n).unwrap_or(0.0);
            // "arcLength" -> p in [0,len]; anything else, including an absent
            // attribute, is "normalized" p in [0,1] -- matching libOpenDRIVE's
            // default and case-insensitive compare (files set it explicitly).
            let p_max = match pp.attribute("pRange").map(str::to_ascii_lowercase) {
                Some(ref r) if r == "arclength" => length,
                _ => 1.0,
            };
            Geom::Baked {
                samples: bake_param_poly3(
                    x,
                    y,
                    hdg,
                    [coeff("aU"), coeff("bU"), coeff("cU"), coeff("dU")],
                    [coeff("aV"), coeff("bV"), coeff("cV"), coeff("dV")],
                    p_max,
                    length,
                ),
            }
        } else if let Some(p3) = child(g, "poly3") {
            // poly3 is the special case of paramPoly3 with u(p)=p and v(p) the
            // cubic in u; reuse the same baker over p in [0, length].
            let coeff = |n: &str| attr_f64(p3, n).unwrap_or(0.0);
            Geom::Baked {
                samples: bake_param_poly3(
                    x,
                    y,
                    hdg,
                    [0.0, 1.0, 0.0, 0.0],
                    [coeff("a"), coeff("b"), coeff("c"), coeff("d")],
                    length,
                    length,
                ),
            }
        } else if child(g, "line").is_some() {
            Geom::Line
        } else {
            // An unknown geometry shape: skip it rather than fail the load.
            continue;
        };
        geoms.push(GeomRec { s, x, y, hdg, geom });
    }
    if geoms.is_empty() {
        return; // nothing drivable to bake
    }
    geoms.sort_by(|a, b| a.s.total_cmp(&b.s));

    let Some(lanes_node) = child(road, "lanes") else {
        return;
    };
    let elevations = child(road, "elevationProfile")
        .map(|n| cubics_in(n, "elevation", "s"))
        .unwrap_or_default();
    // laneOffset shifts the whole lane cross-section laterally off lane 0 (lane
    // widening, merges, a centerline that isn't the road reference). It adds to
    // every lane's offset, so it must be applied or all lanes are mis-placed.
    let lane_offsets = cubics_in(lanes_node, "laneOffset", "s");

    // Every lane section becomes its own set of lanes, each spanning that
    // section's `s`-range `[start, next-start-or-length]`.
    let mut sections: Vec<roxmltree::Node> = lanes_node
        .children()
        .filter(|n| n.has_tag_name("laneSection"))
        .collect();
    if sections.is_empty() {
        return;
    }
    sections.sort_by(|a, b| {
        attr_f64(*a, "s")
            .unwrap_or(0.0)
            .total_cmp(&attr_f64(*b, "s").unwrap_or(0.0))
    });

    // Record the road's link targets + section count for connectivity.
    let (predecessor, successor) = links::road_link(road);
    topo.roads.insert(
        road_id.clone(),
        RoadInfo {
            sections: sections.len(),
            predecessor,
            successor,
        },
    );

    for (i, section) in sections.iter().enumerate() {
        let s_start = attr_f64(*section, "s").unwrap_or(0.0);
        let s_end = sections
            .get(i + 1)
            .map(|n| attr_f64(*n, "s").unwrap_or(length))
            .unwrap_or(length)
            .min(length);
        if s_end - s_start < 1e-3 {
            continue; // zero-length section
        }
        emit_section(
            *section,
            s_start,
            s_end,
            &geoms,
            &elevations,
            &lane_offsets,
            out,
            topo,
            &road_id,
            i,
        );
    }
}

/// Append each driving lane of one section as a `Lane` spanning `[s_start,
/// s_end]`. Malformed individual lanes are skipped, not fatal (real files).
#[allow(clippy::too_many_arguments)]
fn emit_section(
    section: roxmltree::Node,
    s_start: f64,
    s_end: f64,
    geoms: &[GeomRec],
    elevations: &[Cubic],
    lane_offsets: &[Cubic],
    out: &mut Vec<Lane>,
    topo: &mut Topology,
    road_id: &str,
    section_idx: usize,
) {
    let (mut left, mut right) = (Vec::new(), Vec::new());
    for side in ["left", "right"] {
        let Some(side_node) = child(section, side) else {
            continue;
        };
        for lane in side_node.children().filter(|n| n.has_tag_name("lane")) {
            if lane.attribute("type") != Some("driving") {
                continue;
            }
            let Some(id) = lane.attribute("id").and_then(|s| s.parse::<i32>().ok()) else {
                continue;
            };
            let widths = parse_width_cubics(lane);
            if widths.is_empty() {
                continue; // a driving lane with no width can't be sampled
            }
            let (pred_link, succ_link) = links::lane_link(lane);
            let def = LaneDef {
                id,
                widths,
                pred_link,
                succ_link,
            };
            if id > 0 {
                left.push(def);
            } else if id < 0 {
                right.push(def);
            }
        }
    }
    // Order each side from the center outward, so cumulative width works.
    left.sort_by_key(|l| l.id); // 1, 2, 3, ...
    right.sort_by_key(|l| -l.id); // -1, -2, -3, ...

    let sample_s = sample_positions(s_start, s_end);
    for side in [&left, &right] {
        // Left lanes (positive id) offset toward +t and travel against +s;
        // right lanes (negative id) offset toward -t and travel with +s.
        let is_left = side.first().map(|l| l.id > 0).unwrap_or(false);
        let sign = if is_left { 1.0 } else { -1.0 };
        // Standard right-hand-traffic convention: negative-id (right) lanes run
        // with +s, positive-id (left) against it. OpenDRIVE itself encodes no
        // travel direction; a left-hand-traffic map would invert this.
        let direction = if is_left {
            Direction::Backward
        } else {
            Direction::Forward
        };
        // Lanes emitted on this side, center-outward, as (id, index in `out`) --
        // consecutive ones are lateral neighbors (lane-change edges).
        let mut emitted: Vec<(LaneId, usize)> = Vec::new();
        for (i, lane) in side.iter().enumerate() {
            let points = sample_lane(
                geoms,
                elevations,
                lane_offsets,
                s_start,
                lane,
                &side[..i], // inner lanes on the same side, closer to center
                sign,
                &sample_s,
            );
            let Some(center) = Polyline::try_new(points) else {
                continue;
            };
            let id = LaneId(topo.next_id);
            topo.next_id += 1;
            topo.registry
                .insert((road_id.to_string(), section_idx, lane.id), id);
            topo.metas.push(LaneMeta {
                id,
                road: road_id.to_string(),
                section: section_idx,
                od_id: lane.id,
                direction,
                succ_link: lane.succ_link,
                pred_link: lane.pred_link,
            });
            emitted.push((id, out.len()));
            out.push(Lane {
                id,
                kind: LaneKind::Driving,
                direction,
                center,
                width: width_at(lane, 0.0) as f32,
                // Filled by links::resolve once all lanes are registered.
                successors: Vec::new(),
                predecessors: Vec::new(),
                neighbors: Vec::new(),
            });
        }
        // Consecutive same-side lanes are each other's lane-change neighbors.
        for k in 0..emitted.len() {
            let mut nbrs = Vec::new();
            if k > 0 {
                nbrs.push(emitted[k - 1].0);
            }
            if k + 1 < emitted.len() {
                nbrs.push(emitted[k + 1].0);
            }
            out[emitted[k].1].neighbors = nbrs;
        }
    }
}

/// Sample one lane's centerline to points in our coordinate frame.
#[allow(clippy::too_many_arguments)]
fn sample_lane(
    geoms: &[GeomRec],
    elevations: &[Cubic],
    lane_offsets: &[Cubic],
    section_s: f64,
    lane: &LaneDef,
    inner: &[LaneDef],
    sign: f64,
    sample_s: &[f64],
) -> Vec<Vec3> {
    sample_s
        .iter()
        .map(|&s| {
            let s_lane = s - section_s;
            // Center offset: the road's laneOffset (shared by all lanes) plus
            // this lane's own: cumulative inner-lane widths + half its width.
            let base = active(lane_offsets, s).map(|o| o.eval(s)).unwrap_or(0.0);
            let inner_w: f64 = inner.iter().map(|l| width_at(l, s_lane)).sum();
            let t = base + sign * (inner_w + width_at(lane, s_lane) / 2.0);

            let g = geom_at(geoms, s);
            let (x, y, hdg) = g.pose(s);
            let elev = active(elevations, s).map(|e| e.eval(s)).unwrap_or(0.0);
            // ref -> our frame, then offset along the left-hand normal.
            // left_normal(our tangent) = (-sin hdg, 0, -cos hdg).
            Vec3::new(
                (x - t * hdg.sin()) as f32,
                elev as f32,
                (-y - t * hdg.cos()) as f32,
            )
        })
        .collect()
}

fn width_at(lane: &LaneDef, s_lane: f64) -> f64 {
    active(&lane.widths, s_lane)
        .map(|w| w.eval(s_lane).max(0.0))
        .unwrap_or(0.0)
}

fn geom_at(geoms: &[GeomRec], s: f64) -> &GeomRec {
    geoms
        .iter()
        .rev()
        .find(|g| g.s <= s + 1e-9)
        .unwrap_or(&geoms[0])
}

/// Arc-length sample positions over `[start, end]`, spaced `SAMPLE_STEP`,
/// always including the exact endpoints.
fn sample_positions(start: f64, end: f64) -> Vec<f64> {
    let mut ss = Vec::new();
    let mut s = start;
    while s < end - 1e-6 {
        ss.push(s);
        s += SAMPLE_STEP;
    }
    ss.push(end);
    ss
}

/// Parse every `<item>` child of `parent` as a cubic (elevation, laneOffset),
/// keyed on `start_attr`, sorted by start.
fn cubics_in(parent: roxmltree::Node, item: &str, start_attr: &str) -> Vec<Cubic> {
    let mut out: Vec<Cubic> = parent
        .children()
        .filter(|n| n.has_tag_name(item))
        .filter_map(|n| {
            Some(Cubic {
                start: attr_f64(n, start_attr)?,
                a: attr_f64(n, "a").unwrap_or(0.0),
                b: attr_f64(n, "b").unwrap_or(0.0),
                c: attr_f64(n, "c").unwrap_or(0.0),
                d: attr_f64(n, "d").unwrap_or(0.0),
            })
        })
        .collect();
    out.sort_by(|a, b| a.start.total_cmp(&b.start));
    out
}

fn parse_width_cubics(lane: roxmltree::Node) -> Vec<Cubic> {
    let mut out: Vec<Cubic> = lane
        .children()
        .filter(|n| n.has_tag_name("width"))
        .filter_map(|n| {
            Some(Cubic {
                start: attr_f64(n, "sOffset").unwrap_or(0.0),
                a: attr_f64(n, "a")?,
                b: attr_f64(n, "b").unwrap_or(0.0),
                c: attr_f64(n, "c").unwrap_or(0.0),
                d: attr_f64(n, "d").unwrap_or(0.0),
            })
        })
        .collect();
    out.sort_by(|a, b| a.start.total_cmp(&b.start));
    out
}

fn child<'a>(node: roxmltree::Node<'a, 'a>, tag: &str) -> Option<roxmltree::Node<'a, 'a>> {
    node.children().find(|n| n.has_tag_name(tag))
}

#[cfg(test)]
mod tests {
    use super::*;

    // A straight 40 m road climbing at 4%, one right (forward) driving lane.
    const STRAIGHT: &str = r#"<?xml version="1.0"?>
<OpenDRIVE>
  <road name="s" length="40.0" id="1" junction="-1">
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="40.0"><line/></geometry>
    </planView>
    <elevationProfile>
      <elevation s="0.0" a="0.0" b="0.04" c="0.0" d="0.0"/>
    </elevationProfile>
    <lanes>
      <laneSection s="0.0">
        <right>
          <lane id="-1" type="driving">
            <width sOffset="0.0" a="3.5" b="0.0" c="0.0" d="0.0"/>
          </lane>
        </right>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>"#;

    #[test]
    fn straight_one_forward_lane() {
        let net = load_str(STRAIGHT).expect("import");
        assert_eq!(net.driving_lanes().count(), 1);
        let lane = net.lanes.first().unwrap();
        assert_eq!(lane.direction, Direction::Forward);
        let start = lane.center.pose_at(0.0);
        let end = lane.center.pose_at(lane.center.length());
        // Heads +X, the right lane sits on the +Z side, and it climbs.
        assert!(start.heading.x > 0.9, "start heading {:?}", start.heading);
        assert!(
            (start.position.z - 1.75).abs() < 0.1,
            "z {}",
            start.position.z
        );
        assert!(end.position.y > start.position.y + 1.0, "no climb");
    }

    // A straight then a 90-degree left arc (radius 30), two opposing lanes --
    // the same shape as the hand-authored demo_road.
    const STRAIGHT_ARC: &str = r#"<?xml version="1.0"?>
<OpenDRIVE>
  <road name="sa" length="87.12" id="1" junction="-1">
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="40.0"><line/></geometry>
      <geometry s="40.0" x="40.0" y="0.0" hdg="0.0" length="47.12"><arc curvature="0.03333"/></geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <left>
          <lane id="1" type="driving"><width sOffset="0.0" a="3.5"/></lane>
        </left>
        <right>
          <lane id="-1" type="driving"><width sOffset="0.0" a="3.5"/></lane>
        </right>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>"#;

    #[test]
    fn straight_then_left_arc_two_lanes() {
        let net = load_str(STRAIGHT_ARC).expect("import");
        assert_eq!(net.driving_lanes().count(), 2);
        // Forward lane = the right (negative-id) lane.
        let fwd = net
            .lanes
            .iter()
            .find(|l| l.direction == Direction::Forward)
            .unwrap();
        let start = fwd.center.pose_at(0.0);
        let end = fwd.center.pose_at(fwd.center.length());
        assert!(start.heading.x > 0.9, "start {:?}", start.heading);
        // After a 90-degree left turn, heading points toward -Z.
        assert!(end.heading.z < -0.9, "end {:?}", end.heading);
    }

    #[test]
    fn lanes_sit_a_width_apart() {
        let net = load_str(STRAIGHT_ARC).expect("import");
        let fwd = net
            .lanes
            .iter()
            .find(|l| l.direction == Direction::Forward)
            .unwrap();
        let bwd = net
            .lanes
            .iter()
            .find(|l| l.direction == Direction::Backward)
            .unwrap();
        // On the straight, the two lane centers are ~one lane width apart.
        let a = fwd.center.point_at(10.0);
        let gap = (a - bwd.center.project(a).point).length();
        assert!((gap - 3.5).abs() < 0.2, "gap {gap}");
    }

    // A straight road with a constant laneOffset of +2.0 (shifts the whole
    // cross-section left, toward -Z in our frame).
    const STRAIGHT_OFFSET: &str = r#"<?xml version="1.0"?>
<OpenDRIVE>
  <road name="o" length="20.0" id="1" junction="-1">
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="20.0"><line/></geometry>
    </planView>
    <lanes>
      <laneOffset s="0.0" a="2.0" b="0.0" c="0.0" d="0.0"/>
      <laneSection s="0.0">
        <right>
          <lane id="-1" type="driving"><width sOffset="0.0" a="3.5"/></lane>
        </right>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>"#;

    #[test]
    fn lane_offset_shifts_the_cross_section() {
        // Right lane without offset sits at z = +1.75; laneOffset +2.0 shifts
        // it left by 2.0 -> z = -0.25.
        let net = load_str(STRAIGHT_OFFSET).expect("import");
        let z = net.lanes[0].center.pose_at(0.0).position.z;
        assert!((z - (-0.25)).abs() < 0.05, "z {z}");
    }

    // A straight road split into two lane sections at s=25. Each section's
    // lanes should become their own polyline spanning only that section.
    const TWO_SECTIONS: &str = r#"<?xml version="1.0"?>
<OpenDRIVE>
  <road name="ms" length="40.0" id="1" junction="-1">
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="40.0"><line/></geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <right><lane id="-1" type="driving"><width sOffset="0.0" a="3.5"/></lane></right>
      </laneSection>
      <laneSection s="25.0">
        <right><lane id="-1" type="driving"><width sOffset="0.0" a="3.5"/></lane></right>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>"#;

    #[test]
    fn each_lane_section_becomes_its_own_lane() {
        let net = load_str(TWO_SECTIONS).expect("import");
        assert_eq!(net.driving_lanes().count(), 2, "one lane per section");
        let mut lens: Vec<f32> = net.lanes.iter().map(|l| l.center.length()).collect();
        lens.sort_by(|a, b| a.total_cmp(b));
        // Sections span [0,25] and [25,40] -> ~25 and ~15 m.
        assert!((lens[0] - 15.0).abs() < 1.0, "short section {}", lens[0]);
        assert!((lens[1] - 25.0).abs() < 1.0, "long section {}", lens[1]);
    }

    // A pure clothoid: curvStart 0, curvEnd 0.1 over 10 m. End heading is the
    // closed form 0.5*c_dot*L^2 = 0.5*(0.01)*100 = 0.5 rad (a left turn -> -Z).
    const SPIRAL_ONLY: &str = r#"<?xml version="1.0"?>
<OpenDRIVE>
  <road name="sp" length="10.0" id="1" junction="-1">
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="10.0">
        <spiral curvStart="0.0" curvEnd="0.1"/>
      </geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <right><lane id="-1" type="driving"><width sOffset="0.0" a="3.5"/></lane></right>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>"#;

    #[test]
    fn spiral_heading_matches_closed_form() {
        // Sampled at s=5 (interior, where the polyline's interpolated tangent is
        // accurate -- the very endpoint tangent is a last-segment artifact).
        // Reference heading there = 0.5*c_dot*s^2 = 0.5*0.01*25 = 0.125 rad.
        let net = load_str(SPIRAL_ONLY).expect("import");
        let h = net.lanes[0].center.pose_at(5.0).heading;
        let theta = (-h.z).atan2(h.x);
        assert!((theta - 0.125).abs() < 0.03, "mid heading angle {theta}");
    }

    // A normalized paramPoly3: u(p)=10p, v(p)=5p^2 for p in [0,1], so the curve
    // runs from OD (0,0) to OD (10,5) -> our frame end near z=-5. The road is
    // longer than the curve (arc length ~11.5), so the end clamps to that point.
    // A straight line (v ignored) would end at z=0.
    const PARAM_POLY3: &str = r#"<?xml version="1.0"?>
<OpenDRIVE>
  <road name="pp" length="15.0" id="1" junction="-1">
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="15.0">
        <paramPoly3 pRange="normalized" aU="0" bU="10" cU="0" dU="0" aV="0" bV="0" cV="5" dV="0"/>
      </geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <right><lane id="-1" type="driving"><width sOffset="0.0" a="3.5"/></lane></right>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>"#;

    #[test]
    fn param_poly3_follows_the_v_deviation() {
        let net = load_str(PARAM_POLY3).expect("import");
        let end = net.lanes[0]
            .center
            .pose_at(net.lanes[0].center.length())
            .position;
        assert!(end.x > 8.0, "end x {}", end.x);
        assert!(end.z < -3.0, "end z {} (should follow v to ~-5)", end.z);
    }

    // poly3 with v(u)=0.05*u^2 over length 10 -- curves laterally toward -z
    // (like the arcLength paramPoly3 case; exact endpoint depends on the
    // arc-length reparametrization, so just assert a clear deviation).
    const POLY3: &str = r#"<?xml version="1.0"?>
<OpenDRIVE>
  <road name="p3" length="10.0" id="1" junction="-1">
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="10.0">
        <poly3 a="0" b="0" c="0.05" d="0"/>
      </geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <right><lane id="-1" type="driving"><width sOffset="0.0" a="3.5"/></lane></right>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>"#;

    #[test]
    fn poly3_curves_laterally() {
        let net = load_str(POLY3).expect("import");
        let end = net.lanes[0]
            .center
            .pose_at(net.lanes[0].center.length())
            .position;
        assert!(end.z < -2.0, "end z {} (poly3 should curve)", end.z);
    }

    // Two lane sections in one road, linked lane -1 -> lane -1.
    const TWO_SECTION_LINKED: &str = r#"<?xml version="1.0"?>
<OpenDRIVE>
  <road name="r" length="40.0" id="1" junction="-1">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="40.0"><line/></geometry></planView>
    <lanes>
      <laneSection s="0.0"><right><lane id="-1" type="driving">
        <link><successor id="-1"/></link><width sOffset="0.0" a="3.5"/>
      </lane></right></laneSection>
      <laneSection s="15.0"><right><lane id="-1" type="driving">
        <link><predecessor id="-1"/></link><width sOffset="0.0" a="3.5"/>
      </lane></right></laneSection>
    </lanes>
  </road>
</OpenDRIVE>"#;

    #[test]
    fn cross_section_link() {
        let net = load_str(TWO_SECTION_LINKED).expect("import");
        assert_eq!(net.lanes.len(), 2);
        assert_eq!(net.lanes[0].successors, vec![net.lanes[1].id]);
        assert_eq!(net.lanes[1].predecessors, vec![net.lanes[0].id]);
    }

    // Road 1 -> road 2 (its successor), each a single forward lane.
    const TWO_ROADS_LINKED: &str = r#"<?xml version="1.0"?>
<OpenDRIVE>
  <road name="a" length="20.0" id="1" junction="-1">
    <link><successor elementType="road" elementId="2" contactPoint="start"/></link>
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="20.0"><line/></geometry></planView>
    <lanes><laneSection s="0.0"><right><lane id="-1" type="driving">
      <link><successor id="-1"/></link><width sOffset="0.0" a="3.5"/>
    </lane></right></laneSection></lanes>
  </road>
  <road name="b" length="20.0" id="2" junction="-1">
    <link><predecessor elementType="road" elementId="1" contactPoint="end"/></link>
    <planView><geometry s="0.0" x="20.0" y="0.0" hdg="0.0" length="20.0"><line/></geometry></planView>
    <lanes><laneSection s="0.0"><right><lane id="-1" type="driving">
      <link><predecessor id="-1"/></link><width sOffset="0.0" a="3.5"/>
    </lane></right></laneSection></lanes>
  </road>
</OpenDRIVE>"#;

    #[test]
    fn road_to_road_link() {
        let net = load_str(TWO_ROADS_LINKED).expect("import");
        assert_eq!(net.lanes.len(), 2);
        assert_eq!(net.lanes[0].successors, vec![net.lanes[1].id], "A -> B");
        assert_eq!(net.lanes[1].predecessors, vec![net.lanes[0].id]);
    }

    // Road 1 -> junction 100 -> connecting road 2, via a laneLink.
    const JUNCTION: &str = r#"<?xml version="1.0"?>
<OpenDRIVE>
  <road name="in" length="20.0" id="1" junction="-1">
    <link><successor elementType="junction" elementId="100"/></link>
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="20.0"><line/></geometry></planView>
    <lanes><laneSection s="0.0"><right><lane id="-1" type="driving">
      <link><successor id="-1"/></link><width sOffset="0.0" a="3.5"/>
    </lane></right></laneSection></lanes>
  </road>
  <road name="conn" length="20.0" id="2" junction="100">
    <link><predecessor elementType="road" elementId="1" contactPoint="end"/></link>
    <planView><geometry s="0.0" x="20.0" y="0.0" hdg="0.0" length="20.0"><line/></geometry></planView>
    <lanes><laneSection s="0.0"><right><lane id="-1" type="driving">
      <width sOffset="0.0" a="3.5"/>
    </lane></right></laneSection></lanes>
  </road>
  <junction id="100">
    <connection id="0" incomingRoad="1" connectingRoad="2" contactPoint="start">
      <laneLink from="-1" to="-1"/>
    </connection>
  </junction>
</OpenDRIVE>"#;

    #[test]
    fn junction_link() {
        let net = load_str(JUNCTION).expect("import");
        assert_eq!(net.lanes.len(), 2);
        assert_eq!(
            net.lanes[0].successors,
            vec![net.lanes[1].id],
            "road -> junction -> connecting road"
        );
    }

    #[test]
    fn empty_or_junk_is_an_error() {
        assert!(load_str("<OpenDRIVE></OpenDRIVE>").is_err());
        assert!(load_str("not xml at all <<<").is_err());
    }
}
