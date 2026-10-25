//! fastroute: the dense parts of a board (fine-pitch SMD parts such as QFN, QFP, TSSOP and
//! BGA), whose pins are hard to escape one at a time. The report uses them to tell how many
//! unrouted connections start at such a part; the escape planner works on them.

use fr_geom::{FloatPoint, IntBox};

use crate::board::{ItemKey, RoutingBoard};
use crate::structure::Unit;

/// A part counts as dense with at least this many SMD pins with nets ...
pub const DENSE_MIN_PINS: usize = 16;
/// ... and a pin pitch of at most this many µm.
pub const DENSE_MAX_PITCH_UM: f64 = 650.0;
/// An unrouted connection is counted at a dense part if one end is a pin of the part or lies
/// within this many µm of its pads.
pub const NEAR_UM: f64 = 2000.0;

/// A dense part of the board.
#[derive(Clone, Debug)]
pub struct DensePart {
    pub component_no: i32,
    pub name: String,
    /// The SMD pins of the part that have a net.
    pub pins: Vec<ItemKey>,
    /// The median distance from a pin centre to the nearest other pin centre, in board units.
    pub pitch: f64,
    /// The bounding box of the pads.
    pub bbox: IntBox,
}

/// The dense parts of `board`, by component number.
pub fn dense_parts(board: &RoutingBoard) -> Vec<DensePart> {
    let um = board.communication.get_resolution(Unit::Um);
    let mut by_component: std::collections::BTreeMap<i32, Vec<ItemKey>> = Default::default();
    for p in board.get_smd_pins() {
        let it = board.item(p);
        if it.component_no() > 0 && it.net_count() > 0 {
            by_component.entry(it.component_no()).or_default().push(p);
        }
    }
    let mut result = Vec::new();
    for (component_no, pins) in by_component {
        if pins.len() < DENSE_MIN_PINS {
            continue;
        }
        let pitch = median_pitch(&pins.iter().map(|&p| board.item(p).center(board).to_float()).collect::<Vec<_>>());
        if pitch > DENSE_MAX_PITCH_UM * um {
            continue;
        }
        result.push(DensePart {
            component_no,
            name: board.components.get(component_no).name.clone(),
            bbox: board.bounding_box_of(pins.iter().copied()),
            pins,
            pitch,
        });
    }
    result
}

/// The median nearest-neighbour distance of `points` (at least 2).
fn median_pitch(points: &[FloatPoint]) -> f64 {
    let mut nearest: Vec<f64> = points
        .iter()
        .enumerate()
        .map(|(i, a)| points.iter().enumerate().filter(|&(j, _)| j != i).map(|(_, b)| a.distance(b)).fold(f64::MAX, f64::min))
        .collect();
    nearest.sort_by(f64::total_cmp);
    nearest[nearest.len() / 2]
}

/// Where an unrouted connection meets a dense part.
#[derive(Clone, Copy, Debug)]
pub struct EscapeContact<'a> {
    pub part: &'a DensePart,
    /// One end is a pin of the part (otherwise an end only lies near its pads).
    pub at_pin: bool,
}

/// The dense part an unrouted connection between `from` (at `from_xy`) and `to` starts at:
/// a part one end is a pin of, else the nearest part an end lies within [`NEAR_UM`] of.
pub fn escape_contact<'a>(
    board: &RoutingBoard,
    parts: &'a [DensePart],
    ends: [(ItemKey, FloatPoint); 2],
) -> Option<EscapeContact<'a>> {
    for (key, _) in ends {
        let c = board.item(key).component_no();
        if let Some(part) = parts.iter().find(|p| p.component_no == c && p.pins.contains(&key)) {
            return Some(EscapeContact { part, at_pin: true });
        }
    }
    let near = NEAR_UM * board.communication.get_resolution(Unit::Um);
    parts
        .iter()
        .flat_map(|p| ends.iter().map(move |(_, xy)| (p, p.bbox.distance(xy))))
        .filter(|&(_, d)| d <= near)
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(part, _)| EscapeContact { part, at_pin: false })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn median_pitch_of_a_row() {
        let pts: Vec<FloatPoint> = (0..8).map(|i| FloatPoint::new(i as f64 * 500.0, 0.0)).collect();
        assert_eq!(median_pitch(&pts), 500.0);
        // one stray pin far away does not change the median
        let mut pts = pts;
        pts.push(FloatPoint::new(0.0, 10_000.0));
        assert_eq!(median_pitch(&pts), 500.0);
    }
}
