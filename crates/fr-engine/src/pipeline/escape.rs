//! fastroute: the dense parts of a board (fine-pitch SMD parts such as QFN, QFP, TSSOP and
//! BGA), whose pins are hard to escape one at a time. The report uses them to tell how many
//! unrouted connections start at such a part; the escape planner works on them.
//!
//! The fanout stage gives every SMD pin a via, one pin at a time. At a dense part those vias
//! fill the space around the pads, also for pins whose connection stays on the pad's layer
//! (a decoupling capacitor next to the part, a bus to a neighbour on the same side). The
//! escape planner ([`plan_escapes`]) decides for all pins of the dense parts together which
//! ones keep to their own layer: a pin whose nearest pin of the same net (on another part) is
//! reachable on the pad's layer stays there, unless its airline crosses the airline of
//! another such pin; of crossing pins, those with the most crossings get their via, until no
//! two airlines on a layer cross. Pins of plane nets and pins without such a target are fanned
//! out as before.
//!
//! A connection whose target is on another layer only needs one via, but the fanout gives
//! both ends one. When the target is not a pin of a dense part, the plan keeps the target on
//! its layer (the fanout skips it) and leaves the via at the dense part, where the connection
//! can then use either layer. (Moving the via to the target instead, and keeping the dense
//! pin on its layer, routed worse: Smart-Watch 37 → 47 unrouted in the first run.)

use std::collections::{BTreeMap, BTreeSet};

use fr_geom::{FloatPoint, IntBox};

use crate::board::{ItemKey, RoutingBoard};
use crate::ids::LayerNo;
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

/// The result of [`plan_escapes`].
#[derive(Clone, Debug, Default)]
pub struct EscapePlan {
    /// The pins (item ids) that keep to their own layer: the fanout stage leaves them alone.
    pub keep_on_layer: BTreeSet<i32>,
    pub dense_parts: usize,
    /// Pins of the dense parts the plan looked at (signal pins with a target on another part).
    pub planned: usize,
    /// Pins that would stay on their layer but got a via because their airlines crossed.
    pub via_for_crossings: usize,
    /// Planned pins whose target is on another layer only (their target keeps its layer
    /// unless it is a pin of a dense part) ...
    pub other_layer: usize,
    /// ... of these, the ones whose target is a pin of a dense part.
    pub other_layer_dense: usize,
}

/// One pin of a dense part with a target on the same layer.
struct Candidate {
    id: i32,
    layer: LayerNo,
    from: FloatPoint,
    to: FloatPoint,
    /// The item id of the target pin (airlines to the same pin do not cross).
    target: i32,
}

/// Plans the escapes of the dense parts of `board` (see the module comment).
pub fn plan_escapes(board: &RoutingBoard) -> EscapePlan {
    let parts = dense_parts(board);
    let mut plan = EscapePlan { dense_parts: parts.len(), ..Default::default() };
    if parts.is_empty() {
        return plan;
    }
    // nets with a plane: their pins want a via to the plane
    let plane_nets: BTreeSet<i32> = board
        .get_conduction_areas()
        .into_iter()
        .flat_map(|k| board.item(k).net_numbers().to_vec())
        .collect();
    // pins by net: (key, centre, component)
    let mut by_net: BTreeMap<i32, Vec<(ItemKey, FloatPoint, i32)>> = BTreeMap::new();
    for p in board.get_pins() {
        let it = board.item(p);
        if it.net_count() == 1 {
            by_net.entry(it.net_number(0)).or_default().push((p, it.center(board).to_float(), it.component_no()));
        }
    }
    let mut candidates = Vec::new();
    let mut targets_kept: BTreeSet<i32> = BTreeSet::new();
    for part in &parts {
        for &p in &part.pins {
            let it = board.item(p);
            let net = it.net_number(0);
            if it.net_count() != 1 || plane_nets.contains(&net) {
                continue;
            }
            let from = it.center(board).to_float();
            let Some(&(target, to, _)) = by_net
                .get(&net)
                .into_iter()
                .flatten()
                .filter(|(_, _, c)| *c != part.component_no)
                .min_by(|a, b| from.distance(&a.1).total_cmp(&from.distance(&b.1)))
            else {
                continue;
            };
            plan.planned += 1;
            let layer = it.first_layer(board);
            let t = board.item(target);
            if t.first_layer(board) <= layer && layer <= t.last_layer(board) {
                candidates.push(Candidate { id: it.id().0, layer, from, to, target: t.id().0 });
            } else {
                plan.other_layer += 1;
                // one via for the connection, at the dense part: the target keeps its layer
                if parts.iter().any(|q| q.pins.contains(&target)) {
                    plan.other_layer_dense += 1;
                } else if t.first_layer(board) == t.last_layer(board) {
                    targets_kept.insert(t.id().0);
                }
            }
        }
    }
    // the crossing graph of the airlines on each layer; then, greedily, the pin with the most
    // crossings (then the longest airline, then the lowest id) gets its via
    let n = candidates.len();
    let mut crossing: Vec<BTreeSet<usize>> = vec![BTreeSet::new(); n];
    for i in 0..n {
        for j in i + 1..n {
            let (a, b) = (&candidates[i], &candidates[j]);
            if a.layer == b.layer && a.target != b.target && segments_cross(a.from, a.to, b.from, b.to) {
                crossing[i].insert(j);
                crossing[j].insert(i);
            }
        }
    }
    let mut via = vec![false; n];
    loop {
        let worst = (0..n).filter(|&i| !via[i] && !crossing[i].is_empty()).max_by(|&i, &j| {
            let len = |k: usize| candidates[k].from.distance(&candidates[k].to);
            crossing[i].len().cmp(&crossing[j].len()).then(len(i).total_cmp(&len(j))).then(candidates[j].id.cmp(&candidates[i].id))
        });
        let Some(w) = worst else { break };
        via[w] = true;
        plan.via_for_crossings += 1;
        for k in std::mem::take(&mut crossing[w]) {
            crossing[k].remove(&w);
        }
    }
    plan.keep_on_layer = (0..n).filter(|&i| !via[i]).map(|i| candidates[i].id).collect();
    plan.keep_on_layer.extend(targets_kept);
    plan
}

/// Whether the segments `a1 a2` and `b1 b2` cross (touching ends do not count).
fn segments_cross(a1: FloatPoint, a2: FloatPoint, b1: FloatPoint, b2: FloatPoint) -> bool {
    // (not f64::signum: that is 1 for 0)
    let orient = |p: FloatPoint, q: FloatPoint, r: FloatPoint| {
        let c = (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
        (c > 0.0) as i32 - (c < 0.0) as i32
    };
    let (d1, d2) = (orient(b1, b2, a1), orient(b1, b2, a2));
    let (d3, d4) = (orient(a1, a2, b1), orient(a1, a2, b2));
    d1 * d2 < 0 && d3 * d4 < 0
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

    #[test]
    fn crossing_segments() {
        let p = FloatPoint::new;
        assert!(segments_cross(p(0.0, 0.0), p(2.0, 2.0), p(0.0, 2.0), p(2.0, 0.0)));
        assert!(!segments_cross(p(0.0, 0.0), p(2.0, 0.0), p(0.0, 1.0), p(2.0, 1.0)));
        // sharing an end is not a crossing
        assert!(!segments_cross(p(0.0, 0.0), p(2.0, 2.0), p(2.0, 2.0), p(4.0, 0.0)));
    }
}
