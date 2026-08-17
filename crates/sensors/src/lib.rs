//! The reflex layer. Three parts, all pure logic the `server` feeds a
//! freshly-gathered world into: sensor **readings**
//! (`time_to_collision`/`distance_to`/`speed`) over a `SensorContext`; the
//! perception-**impairment** pipeline (`perceive`: range / FOV / line-of-sight
//! cull, then seeded Gaussian noise); and reflex-rule **evaluation**
//! (`evaluate`: resolve each rule's named device to a context, then apply the
//! threshold with hysteresis and priority). Depends on `protocol` for the rule
//! and message shapes.

mod context;
mod perceive;
mod reflex;
mod sensor;

pub use context::{Obstacle, SensorContext};
pub use perceive::{perceive, Detection, DetectionKind, PerceivedEntity, Rng};
pub use reflex::{evaluate, ActiveRule};
pub use sensor::{sensor_for, DistanceTo, Sensor, Speed, TimeToCollision};
