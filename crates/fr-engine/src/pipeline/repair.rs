//! fastroute: region repair ("ruin and recreate") after the routing stage.
//!
//! The batch autorouter routes one connection at a time; when the last few connections fail,
//! routing them usually needs several existing routes around them to move together, which a
//! single rip-up never does. For each unrouted connection this stage rips every unfixed trace
//! and via in a window around its airline, re-routes the nets involved in several variants
//! (pass orders and rip-up costs) on board clones in parallel, and keeps the best variant if
//! it has fewer unrouted connections (then fewer clearance violations) than the board before.
//! Growing windows are tried in turn. Variants are chosen by index on ties, so the result does
//! not depend on thread timing.

use std::collections::BTreeSet;
use std::time::{Duration, Instant};

use fr_geom::IntBox;
use fr_settings::RouterSettings;

use crate::board::{ItemKey, ItemSet, RoutingBoard, StopConnectionOption};
use crate::drc::net_incompletes::{AirLine, NetIncompletes};
use crate::ids::NetNo;

use super::autorouter::BatchAutorouter;
use super::stats::{self, StatsCache};
use super::PipelineContext;

/// Windows tried in this order: corridor half-width around the airline and the most
/// connections ripped in it (the nearest to the airline first).
const WINDOWS: [(f64, usize); 3] = [(1000.0, 4), (2000.0, 8), (3000.0, 16)];
/// Variants per window (pass order seed, rip-up cost factor).
const VARIANTS: [(Option<i64>, i32); 6] = [(None, 1), (Some(1), 1), (Some(2), 1), (None, 4), (Some(3), 4), (Some(4), 10)];
/// Autorouting passes per variant.
const VARIANT_PASSES: i32 = 10;

/// Options of the repair stage (`--repair`, `--repair-time`).
#[derive(Clone, Copy, Debug)]
pub struct RepairOptions {
    /// Wall clock budget of the stage; it stops after the window in progress.
    pub budget: Duration,
}

/// What the repair stage did.
#[derive(Clone, Copy, Debug, Default)]
pub struct RepairOutcome {
    pub windows_tried: usize,
    pub windows_improved: usize,
    pub incomplete_before: i32,
    pub incomplete_after: i32,
}

type Key = (i32, i32, i64);

fn key_of(s: &stats::Score) -> Key {
    (s.incomplete_count, s.clearance_violation_count, -((s.router_score as f64 * 1000.0).round() as i64))
}

/// The unrouted connections of the board, skipping nets of ignored classes (sorted by net,
/// then position, so the order is deterministic).
pub fn unrouted_airlines(board: &RoutingBoard) -> Vec<AirLine> {
    let max_net_no = board.rules.nets.max_net_number().max(0) as usize;
    let mut net_items: Vec<Vec<ItemKey>> = vec![Vec::new(); max_net_no];
    for key in board.items.iter() {
        let item = board.item(key);
        if item.is_connectable_class() {
            for &n in item.net_numbers() {
                if n >= 1 && (n as usize) <= max_net_no {
                    net_items[n as usize - 1].push(key);
                }
            }
        }
    }
    let mut result = Vec::new();
    for (i, list) in net_items.iter().enumerate() {
        let net_no = i as NetNo + 1;
        if list.len() < 2 || is_ignored_net(board, net_no) {
            continue;
        }
        result.extend(NetIncompletes::new(board, net_no, list).incompletes);
    }
    result
}

fn is_ignored_net(board: &RoutingBoard, net_no: NetNo) -> bool {
    match board.rules.nets.get(net_no) {
        Some(net) => board.rules.net_classes[net.get_net_class()].is_ignored_by_autorouter,
        None => false,
    }
}

/// Board units per micrometre.
fn um_to_board(board: &RoutingBoard, um: f64) -> i32 {
    let resolution = board.communication.resolution.max(1);
    fr_jcompat::math_round(crate::structure::Unit::scale(um * resolution as f64, crate::structure::Unit::Um, board.communication.unit)) as i32
}

/// Distance from the segment `a`-`b` to the box `bx` (0 if they touch).
fn segment_box_distance(a: (f64, f64), b: (f64, f64), bx: &IntBox) -> f64 {
    let (x0, y0, x1, y1) = (bx.ll.x as f64, bx.ll.y as f64, bx.ur.x as f64, bx.ur.y as f64);
    let point_box = |p: (f64, f64)| {
        let dx = (x0 - p.0).max(p.0 - x1).max(0.0);
        let dy = (y0 - p.1).max(p.1 - y1).max(0.0);
        dx.hypot(dy)
    };
    let point_segment = |p: (f64, f64)| {
        let (vx, vy) = (b.0 - a.0, b.1 - a.1);
        let len2 = vx * vx + vy * vy;
        let t = if len2 > 0.0 { (((p.0 - a.0) * vx + (p.1 - a.1) * vy) / len2).clamp(0.0, 1.0) } else { 0.0 };
        (p.0 - a.0 - t * vx).hypot(p.1 - a.1 - t * vy)
    };
    // the segment crosses the box if it is inside or cuts one of its sides
    let inside = |p: (f64, f64)| p.0 >= x0 && p.0 <= x1 && p.1 >= y0 && p.1 <= y1;
    if inside(a) || inside(b) {
        return 0.0;
    }
    let corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)];
    let cross = |p: (f64, f64), q: (f64, f64), r: (f64, f64)| (q.0 - p.0) * (r.1 - p.1) - (q.1 - p.1) * (r.0 - p.0);
    for i in 0..4 {
        let (c, d) = (corners[i], corners[(i + 1) % 4]);
        if cross(a, b, c) * cross(a, b, d) <= 0.0 && cross(c, d, a) * cross(c, d, b) <= 0.0 {
            return 0.0;
        }
    }
    let mut dist = point_box(a).min(point_box(b));
    for c in corners {
        dist = dist.min(point_segment(c));
    }
    dist
}

/// Rips the unfixed traces and vias within `margin_um` of the airline (whole connections, up
/// to their forks; at most `max_connections`, nearest first) and returns the nets to re-route
/// and the number of ripped connections, or `None` if nothing near the airline can be ripped.
fn rip_corridor(board: &mut RoutingBoard, a: &AirLine, margin_um: f64, max_connections: usize) -> Option<(BTreeSet<NetNo>, usize)> {
    let margin = um_to_board(board, margin_um) as f64;
    let (p, q) = ((a.from_corner.x, a.from_corner.y), (a.to_corner.x, a.to_corner.y));
    let mut near: Vec<(f64, i32, ItemKey)> = Vec::new();
    for key in board.items.iter() {
        let item = board.item(key);
        if !item.is_routable() {
            continue;
        }
        let d = segment_box_distance(p, q, &item.bounding_box(board));
        if d <= margin {
            near.push((d, item.id().0, key));
        }
    }
    near.sort_by(|x, y| x.0.total_cmp(&y.0).then(x.1.cmp(&y.1)));
    let mut ripped = ItemSet::new();
    let mut connections = 0;
    for &(_, _, k) in &near {
        if connections >= max_connections {
            break;
        }
        if ripped.iter().any(|r| r == k) {
            continue;
        }
        let items = board.get_connection_items(k, StopConnectionOption::None);
        if items.iter().any(|i| board.item(i).is_user_fixed()) {
            continue;
        }
        ripped.extend_from(&items);
        connections += 1;
    }
    if connections == 0 {
        return None;
    }
    let keys: Vec<ItemKey> = ripped.iter().collect();
    let mut nets: BTreeSet<NetNo> = BTreeSet::new();
    nets.insert(a.net_number);
    for &k in &keys {
        nets.extend(board.item(k).net_numbers().iter().copied());
    }
    board.remove_items(keys);
    for &n in &nets {
        board.combine_traces(n);
    }
    Some((nets, connections))
}

/// Routes one variant of a window on a clone of `board`.
fn run_variant(board: &RoutingBoard, settings: &RouterSettings, ctx: &PipelineContext, a: &AirLine, window: (f64, usize), variant: usize) -> Option<(RoutingBoard, stats::Score)> {
    let (seed, ripup_factor) = VARIANTS[variant];
    let mut b = board.clone();
    let net = a.net_number;
    let (nets, ripped) = rip_corridor(&mut b, a, window.0, window.1)?;
    let accuracy = settings.trace_pull_tight_accuracy.unwrap_or(500);
    let ripup = settings.get_start_ripup_costs().saturating_mul(ripup_factor);
    let mut router = BatchAutorouter::new(&b, settings, true, true, ripup, accuracy);
    router.net_filter = Some(nets);
    router.enhancements = true;
    router.order_seed = seed.map(|s| 0x7e9a_0000 + s);
    router.priority_nets = Some(std::iter::once(net).collect());
    let stop = ctx.stop.child();
    router.autoroute_passes_for_optimizing_item(&mut b, settings, VARIANT_PASSES, &stop);
    router.remove_tails(&mut b, StopConnectionOption::None, Some(&stop));
    let s = StatsCache::new().score(&b, settings);
    log::debug!(
        "repair variant {variant} (net {net}, {ripped} connections ripped): {} unrouted, {} violations, router score {:.2}",
        s.incomplete_count,
        s.clearance_violation_count,
        s.router_score as f64
    );
    Some((b, s))
}

/// The repair stage: see the module docs. Replaces `board` by an improved board if any
/// window improved it.
pub fn repair_unrouted(board: &mut RoutingBoard, settings: &RouterSettings, ctx: &PipelineContext, options: RepairOptions) -> RepairOutcome {
    use rayon::prelude::*;
    let start = Instant::now();
    let mut stats = StatsCache::new();
    let mut current = stats.score(board, settings);
    let mut outcome = RepairOutcome { incomplete_before: current.incomplete_count, incomplete_after: current.incomplete_count, ..Default::default() };
    // windows that failed on the current board are not retried until the board changes
    let mut tried: BTreeSet<(NetNo, i64, i64, i64, i64, usize)> = BTreeSet::new();
    'rounds: loop {
        let airlines = unrouted_airlines(board);
        if airlines.is_empty() {
            break;
        }
        for a in &airlines {
            for (m, &window) in WINDOWS.iter().enumerate() {
                if start.elapsed() >= options.budget || ctx.stop.is_stop_requested() {
                    break 'rounds;
                }
                let id = (a.net_number, a.from_corner.x as i64, a.from_corner.y as i64, a.to_corner.x as i64, a.to_corner.y as i64, m);
                if !tried.insert(id) {
                    continue;
                }
                outcome.windows_tried += 1;
                let base: &RoutingBoard = board;
                let results: Vec<Option<(RoutingBoard, stats::Score)>> =
                    (0..VARIANTS.len()).into_par_iter().map(|v| run_variant(base, settings, ctx, a, window, v)).collect();
                let mut best: Option<(usize, RoutingBoard, stats::Score)> = None;
                for (v, r) in results.into_iter().enumerate() {
                    let Some((b, s)) = r else { continue };
                    let better = match &best {
                        None => key_of(&s) < key_of(&current),
                        Some((_, _, bs)) => key_of(&s) < key_of(bs),
                    };
                    if better {
                        best = Some((v, b, s));
                    }
                }
                if let Some((v, b, s)) = best {
                    if s.incomplete_count < current.incomplete_count
                        || (s.incomplete_count == current.incomplete_count && s.clearance_violation_count < current.clearance_violation_count)
                    {
                        log::info!(
                            "Repair: window of {} connection(s) within {:.1} mm of net {} improved the board to {} unrouted, {} violations (was {}, {}; variant {v}).",
                            window.1,
                            window.0 / 1000.0,
                            a.net_number,
                            s.incomplete_count,
                            s.clearance_violation_count,
                            current.incomplete_count,
                            current.clearance_violation_count
                        );
                        *board = b;
                        current = s;
                        outcome.windows_improved += 1;
                        tried.clear();
                        ctx.checkpoint(board, super::CheckpointKey { incomplete: s.incomplete_count, violations: s.clearance_violation_count, stage: 0, score: s.router_score });
                        continue 'rounds;
                    }
                }
            }
        }
        // no window improved the board
        break;
    }
    outcome.incomplete_after = current.incomplete_count;
    log::info!(
        "Repair stage: {} window(s) tried, {} improved the board; unrouted {} -> {} in {:.1} s.",
        outcome.windows_tried,
        outcome.windows_improved,
        outcome.incomplete_before,
        outcome.incomplete_after,
        start.elapsed().as_secs_f64()
    );
    outcome
}
