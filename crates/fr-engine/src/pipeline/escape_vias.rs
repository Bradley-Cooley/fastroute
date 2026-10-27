//! fastroute escape via sites (`--escape-vias`): before the fanout, the pins of a dense part
//! (see [`super::escape`]) whose connection leaves the part's surroundings get a via right at
//! the pad, alternately outside the pad row and inside it (under the part).
//!
//! The fanout gives one pin at a time the first via its search reaches. At a 0.5 mm pitch
//! part those vias end up in one row just outside the pads, where every second one blocks the
//! way of its neighbours, and the space under the part (inside the pad ring) stays empty. Here
//! the vias of a whole part are placed together: along each side, a pin that needs a via gets
//! it outside if its neighbours do not leave on the pad's layer or use the outside, and
//! inside otherwise, so that no trace has to pass between two vias. A site that is not free
//! is tried on the other side, and a pin without a free site is left to the fanout.

use std::collections::{BTreeMap, BTreeSet};

use fr_geom::{FloatPoint, IntPoint, Point};
use fr_settings::RouterSettings;

use crate::autoroute::control::AutorouteControl;
use crate::board::actions::forced_via_inserter;
use crate::board::{ItemKey, RoutingBoard};
use crate::ids::FixedState;
use crate::structure::Unit;

use super::escape::DensePart;

/// A pin needs a via if the nearest pin of its net on another part is further away than this
/// many µm (closer targets are reached on the pad's layer).
pub const LOCAL_TARGET_UM: f64 = 5000.0;
/// The gaps between the end of the pad and the via's copper tried, in µm (the nearest free
/// one is taken).
const PAD_GAPS_UM: [f64; 7] = [50.0, 100.0, 150.0, 200.0, 300.0, 400.0, 500.0];

/// What a pin of a side does.
#[derive(Clone, Copy, PartialEq, Debug)]
enum Exit {
    /// leaves on the pad's layer (or is not routed)
    Layer,
    Outside,
    Inside,
}

/// The result of [`place_escape_vias`].
#[derive(Clone, Debug, Default)]
pub struct EscapeVias {
    /// The pins (item ids) that got a via: the fanout leaves them alone.
    pub pins: BTreeSet<i32>,
    pub outside: usize,
    pub inside: usize,
    /// pins that needed a via but found no free site
    pub no_site: usize,
}

/// Places the escape vias of `parts` on `board` (see the module comment).
pub fn place_escape_vias(board: &mut RoutingBoard, settings: &RouterSettings, parts: &[DensePart], local_target_um: f64) -> EscapeVias {
    let um = board.communication.get_resolution(Unit::Um);
    let mut result = EscapeVias::default();
    let plane_nets: BTreeSet<i32> =
        board.get_conduction_areas().into_iter().flat_map(|k| board.item(k).net_numbers().to_vec()).collect();
    let mut by_net: BTreeMap<i32, Vec<(FloatPoint, i32)>> = BTreeMap::new();
    for p in board.get_pins() {
        let it = board.item(p);
        if it.net_count() == 1 {
            by_net.entry(it.net_number(0)).or_default().push((it.center(board).to_float(), it.component_no()));
        }
    }
    for part in parts {
        // the pins of each side, in order along the side: (position along the side, key,
        // outward direction, distance from the centre to the pad end, needs a via)
        let b = part.bbox;
        let mut sides: [Vec<(f64, ItemKey, (f64, f64), f64, bool)>; 4] = Default::default();
        for &p in &part.pins {
            let it = board.item(p);
            let c = it.center(board).to_float();
            let pad = it.bounding_box(board);
            let cand = [
                (c.x - b.ll.x as f64, (-1.0, 0.0), c.x - pad.ll.x as f64, c.y),
                (b.ur.x as f64 - c.x, (1.0, 0.0), pad.ur.x as f64 - c.x, c.y),
                (c.y - b.ll.y as f64, (0.0, -1.0), c.y - pad.ll.y as f64, c.x),
                (b.ur.y as f64 - c.y, (0.0, 1.0), pad.ur.y as f64 - c.y, c.x),
            ];
            let (side, &(_, dir, half, along)) = cand.iter().enumerate().min_by(|a, b| a.1 .0.total_cmp(&b.1 .0)).unwrap();
            let needs_via = it.net_count() == 1 && !plane_nets.contains(&it.net_number(0)) && {
                let others = by_net.get(&it.net_number(0)).into_iter().flatten().filter(|(_, comp)| *comp != part.component_no);
                others.map(|(q, _)| c.distance(q)).min_by(|a, b| a.total_cmp(b)).is_some_and(|d| d > local_target_um * um)
            };
            sides[side].push((along, p, dir, half, needs_via));
        }
        for side in sides.iter_mut() {
            side.sort_by(|a, b| a.0.total_cmp(&b.0));
            let n = side.len();
            let mut exits = vec![Exit::Layer; n];
            for i in 0..n {
                let (_, key, dir, half, needs_via) = side[i];
                if !needs_via {
                    continue;
                }
                // a neighbour leaving on the pad's layer, or with a via outside, blocks the
                // outside site
                let blocked = |j: Option<usize>, exits: &[Exit]| {
                    j.is_some_and(|j| j < n && (exits[j] == Exit::Outside || exits[j] == Exit::Layer && !side[j].4 && j < i))
                };
                let next_keeps_layer = i + 1 < n && !side[i + 1].4;
                let prefer_outside = !blocked(i.checked_sub(1), &exits) && !next_keeps_layer;
                let order = if prefer_outside { [Exit::Outside, Exit::Inside] } else { [Exit::Inside, Exit::Outside] };
                for exit in order {
                    if exit == Exit::Outside && blocked(i.checked_sub(1), &exits) {
                        continue;
                    }
                    if try_place(board, settings, key, dir, half, exit == Exit::Outside, um) {
                        exits[i] = exit;
                        result.pins.insert(board.item(key).id().0);
                        if exit == Exit::Outside {
                            result.outside += 1;
                        } else {
                            result.inside += 1;
                        }
                        break;
                    }
                }
                if exits[i] == Exit::Layer {
                    result.no_site += 1;
                }
            }
        }
    }
    result
}

/// Places a via for `pin` at the end of its pad, outside (away from the part) or inside, with
/// a trace from the pad centre, if both are free of other copper. Returns whether it did.
fn try_place(board: &mut RoutingBoard, settings: &RouterSettings, pin: ItemKey, dir: (f64, f64), half: f64, outside: bool, um: f64) -> bool {
    let it = board.item(pin);
    let net = it.net_number(0);
    let layer = it.first_layer(board);
    let c = it.center(board).to_float();
    let ctrl = AutorouteControl::new_default(board, net, settings);
    // the net class's vias, else the board's (as the fanout's fallback)
    let via = if ctrl.via_rule.via_count() > 0 {
        ctrl.via_rule.get_via(0)
    } else {
        match board.rules.via_rules.get_first() {
            Some(r) if board.rules.via_rules[r].via_count() > 0 => board.rules.via_rules[r].get_via(0),
            _ => return false,
        }
    };
    let via_info = board.rules.via_infos[via].clone();
    let padstack = via_info.get_padstack();
    let via_radius = match board.library.padstacks.get(padstack).and_then(|p| p.get_shape(layer)) {
        Some(s) => 0.5 * s.max_width(),
        None => return false,
    };
    let s = if outside { 1.0 } else { -1.0 };
    let nets = [net];
    let start: IntPoint = c.round();
    let half_width = ctrl.trace_half_width[layer as usize];
    for gap in PAD_GAPS_UM {
        let d = half + via_radius + gap * um;
        let location = Point::Int(FloatPoint::new(c.x + s * dir.0 * d, c.y + s * dir.1 * d).round());
        if !forced_via_inserter::check(board, &via_info, &location, &nets, 0, 0, Some(&ctrl.trace_half_width), ctrl.trace_clearance_class) {
            continue;
        }
        let points = [Point::Int(start), location.clone()];
        let poly = fr_geom::Polyline::from_points(&points);
        if !board.check_forced_trace_polyline(&poly, half_width, layer, &nets, ctrl.trace_clearance_class, 0, 0, 0) {
            continue;
        }
        let fixed = if std::env::var("FR_SFIX").is_ok() { FixedState::ShoveFixed } else { FixedState::Unfixed }; // TUNE
        board.insert_escape_via(padstack, location, &nets, via_info.get_clearance_class_index(), fixed, layer);
        board.insert_trace_points(&points, layer, half_width, &nets, ctrl.trace_clearance_class, fixed);
        return true;
    }
    false
}
