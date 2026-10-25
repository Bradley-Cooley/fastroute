//! Java `autoroute/pipeline/BatchFanout`: fanout passes over the SMD pins of the components
//! (components with more SMD pins first, pins in the configured order).

use std::cmp::Ordering;
use std::time::{Duration, Instant};

use fr_geom::FloatPoint;
use fr_settings::RouterSettings;

use crate::board::{AutorouteAttemptState, ItemKey, RoutingBoard};
use crate::datastructures::IdGenerator;
use crate::structure::Unit;

use super::autorouter::enforce_strict_drc;
use super::history::board_hash;
use super::PipelineContext;

/// Java `BatchFanout.FanoutRunSummary` (the parts used by the port).
#[derive(Clone, Copy, Debug, Default)]
pub struct FanoutRunSummary {
    pub completed_passes: i32,
    pub timed_out: bool,
}

struct FanoutPin {
    key: ItemKey,
    pin_index: i32,
    distance_to_component_center: f64,
    distance_to_closest_on_net: f64,
    surroundings_density: i32,
}

struct FanoutComponent {
    component_no: i32,
    smd_pin_count: i32,
    pins: Vec<FanoutPin>,
}

/// Java `Component.Pin.compareTo`.
fn compare_pins(order: &str, a: &FanoutPin, b: &FanoutPin) -> Ordering {
    let mut result = 0i32;
    match order {
        "inner_first" => {
            let d = a.distance_to_component_center - b.distance_to_component_center;
            if d > 0.0 {
                result = 1;
            } else if d < 0.0 {
                result = -1;
            }
        }
        "outer_first" => {
            let d = a.distance_to_component_center - b.distance_to_component_center;
            if d > 0.0 {
                result = -1;
            } else if d < 0.0 {
                result = 1;
            }
        }
        "distanceToClosestOnNet" => {
            let d = a.distance_to_closest_on_net - b.distance_to_closest_on_net;
            if d > 0.0 {
                result = 1;
            } else if d < 0.0 {
                result = -1;
            }
        }
        "surroundingsDensity" => {
            let d = b.surroundings_density.wrapping_sub(a.surroundings_density);
            result = d.signum();
        }
        _ => {}
    }
    if result == 0 {
        result = a.pin_index.wrapping_sub(b.pin_index);
    }
    result.cmp(&0)
}

/// Java `new BatchFanout(board, settings, thread)`: the sorted components with SMD pins.
fn sorted_components(
    board: &RoutingBoard,
    settings: &RouterSettings,
    skip_ignored_nets: bool,
    skip_pins: Option<&std::collections::BTreeSet<i32>>,
) -> Vec<FanoutComponent> {
    let order = settings.fanout.pin_sorting_order.clone().unwrap_or_else(|| "outer_first".to_string());
    // fastroute: pins of net classes the autorouter ignores (router.autorouter.ignore_net_classes)
    // get no fanout either; Java fans them out, leaving stubs on nets that must not be routed.
    let ignored = |p: ItemKey| {
        skip_ignored_nets
            && board.item(p).net_numbers().iter().any(|&n| {
                board.rules.nets.get(n).map(|net| board.rules.net_classes[net.get_net_class()].is_ignored_by_autorouter).unwrap_or(false)
            })
    };
    let smd_pins_with_nets: Vec<ItemKey> =
        board
            .get_smd_pins()
            .into_iter()
            .filter(|&p| board.item(p).net_count() > 0 && !ignored(p))
            // fastroute escape planner: pins the plan keeps on their own layer
            .filter(|&p| !skip_pins.is_some_and(|s| s.contains(&board.item(p).id().0)))
            .collect();
    let mut result = Vec::new();
    for i in 1..=board.components.count() {
        let component_id = board.components.get(i).id;
        let pin_list: Vec<ItemKey> = smd_pins_with_nets.iter().copied().filter(|&p| board.item(p).component_no() == component_id).collect();
        let (mut x, mut y) = (0.0f64, 0.0f64);
        for &p in &pin_list {
            let c = board.item(p).center(board).to_float();
            x += c.x;
            y += c.y;
        }
        let n = pin_list.len() as i32;
        if n > 0 {
            x /= n as f64;
            y /= n as f64;
        }
        let gravity = FloatPoint::new(x, y);
        let need_closest = order == "distanceToClosestOnNet";
        let need_density = order == "surroundingsDensity";
        let mut pins: Vec<FanoutPin> = Vec::new();
        for &p in &pin_list {
            let it = board.item(p);
            let loc = it.center(board).to_float();
            let mut min_distance = f64::MAX;
            if need_closest {
                let net_no = if it.net_count() > 0 { it.net_number(0) } else { 0 };
                if net_no > 0 {
                    for other in board.get_pins() {
                        if other != p && board.item(other).contains_net(net_no) {
                            let d = loc.distance(&board.item(other).center(board).to_float());
                            if d < min_distance {
                                min_distance = d;
                            }
                        }
                    }
                }
            }
            let mut density = 0;
            if need_density {
                let max_dist = 20000.0 * board.communication.get_resolution(Unit::Um);
                for &other in &smd_pins_with_nets {
                    if other != p && loc.distance(&board.item(other).center(board).to_float()) <= max_dist {
                        density += 1;
                    }
                }
            }
            pins.push(FanoutPin {
                key: p,
                pin_index: it.as_pin().map(|x| x.pin_index).unwrap_or(0),
                distance_to_component_center: loc.distance(&gravity),
                distance_to_closest_on_net: min_distance,
                surroundings_density: density,
            });
        }
        // TreeSet<Pin> (pins of a component have distinct pin indices, no duplicates)
        pins.sort_by(|a, b| compare_pins(&order, a, b));
        if n > 0 {
            result.push(FanoutComponent { component_no: component_id, smd_pin_count: n, pins });
        }
    }
    // Component.compareTo: more SMD pins first, then by component id
    result.sort_by(|a, b| {
        let c = a.smd_pin_count - b.smd_pin_count;
        if c > 0 {
            Ordering::Less
        } else if c < 0 {
            Ordering::Greater
        } else {
            a.component_no.wrapping_sub(b.component_no).cmp(&0)
        }
    });
    result
}

/// Java `BatchFanout.fanoutBoard(board, settings, thread, listener)`; fastroute: `skip_pins`
/// (item ids) are not fanned out.
pub fn fanout_board(
    board: &mut RoutingBoard,
    settings: &RouterSettings,
    ctx: &PipelineContext,
    skip_pins: Option<&std::collections::BTreeSet<i32>>,
) -> FanoutRunSummary {
    let components = sorted_components(board, settings, ctx.enhancements, skip_pins);
    let total_smd_pin_count: i32 = components.iter().map(|c| c.smd_pin_count).sum();
    let start = Instant::now();
    let deadline = if ctx.wall_clock_limits {
        settings
            .fanout
            .timeout_string
            .as_deref()
            .and_then(fr_settings::parse_timespan_string)
            .map(|s| start + Duration::from_secs(s.max(0) as u64))
    } else {
        None
    };
    let max_passes = settings.fanout.max_passes.unwrap_or(20);
    let stagnation_pass_limit = 3;
    let mut state = FanoutState { total_items_fanouted: 0, timed_out: false, deadline };
    let mut completed_passes = 0;
    let mut previous_board_state = i64::MIN;
    let mut identical_passes = 0;
    let mut last_board_hash = board_hash(board);
    let mut previous_fanned: std::collections::BTreeSet<i32> = std::collections::BTreeSet::new();
    for i in 0..max_passes {
        if ctx.stop.is_stop_autorouter_requested() {
            break;
        }
        if let Some(d) = state.deadline {
            if Instant::now() >= d {
                state.timed_out = true;
                log::info!("Fanout stage timed out before starting pass #{}", i + 1);
                break;
            }
        }
        if let Some(m) = settings.fanout.max_items {
            if m > 0 && state.total_items_fanouted >= m {
                break;
            }
        }
        let mut fanned = std::collections::BTreeSet::new();
        let routed_count = fanout_pass(board, settings, ctx, &components, total_smd_pin_count, i, &mut state, &mut fanned);
        completed_passes += 1;
        if routed_count == 0 {
            break;
        }
        // fastroute: a pass that only fans out pins the previous pass fanned out as well
        // (ripped again in between) makes no progress; Java needs 3 such passes to notice.
        if ctx.enhancements && !fanned.is_empty() && fanned.is_subset(&previous_fanned) {
            log::info!(
                "Fanout stopped after {completed_passes} passes: the last pass only re-fanned {} pin{} of the previous pass.",
                fanned.len(),
                if fanned.len() == 1 { "" } else { "s" }
            );
            break;
        }
        previous_fanned = fanned;
        let board_state = ((routed_count as i64) << 32) ^ board.get_vias().len() as i64;
        if board_state == previous_board_state {
            identical_passes += 1;
            if identical_passes >= stagnation_pass_limit {
                log::info!("Fanout stopped after {completed_passes} passes: no progress for {stagnation_pass_limit} consecutive passes.");
                break;
            }
        } else {
            identical_passes = 0;
            previous_board_state = board_state;
        }
        if state.timed_out {
            break;
        }
        let current_board_hash = board_hash(board);
        if current_board_hash == last_board_hash {
            break;
        }
        last_board_hash = current_board_hash;
    }
    FanoutRunSummary { completed_passes, timed_out: state.timed_out }
}

struct FanoutState {
    total_items_fanouted: i32,
    timed_out: bool,
    deadline: Option<Instant>,
}

/// Java `fanoutPass(passNo, listener)`: returns the number of pins fanouted in this pass.
/// fastroute: progress line interval inside a long fanout pass.
const PROGRESS_LOG_INTERVAL_SECS: f64 = 30.0;
/// fastroute: a pin whose fanout takes this long gets its own log line.
const SLOW_PIN_SECS: f64 = 10.0;

/// "pin 12 of U5 (net GND)" for the log.
fn pin_label(board: &RoutingBoard, key: ItemKey) -> String {
    let it = board.item(key);
    let component = it.component_no();
    let component = if component > 0 { board.components.get(component).name.clone() } else { "?".to_string() };
    let pin_no = match &it.kind {
        crate::board::ItemKind::Pin(p) => p.pin_index + 1,
        _ => 0,
    };
    let net = board.rules.nets.get(it.net_number(0)).map(|n| n.name.clone()).unwrap_or_default();
    format!("pin {pin_no} of {component} (net {net})")
}

fn fanout_pass(
    board: &mut RoutingBoard,
    settings: &RouterSettings,
    ctx: &PipelineContext,
    components: &[FanoutComponent],
    total_smd_pin_count: i32,
    pass_no: i32,
    state: &mut FanoutState,
    fanned: &mut std::collections::BTreeSet<i32>,
) -> i32 {
    let pass_start = Instant::now();
    let mut pins_to_go = total_smd_pin_count;
    let (mut routed, mut not_routed, mut insert_errors) = (0, 0, 0);
    let vias_before = board.get_vias().len() as i32;
    let ripup_costs = settings.get_start_ripup_costs().wrapping_mul(pass_no + 1);
    let base_millis_per_pin = settings.fanout.max_milliseconds_per_pin.unwrap_or(10000);
    let ripup_allowed = settings.fanout.ripup_allowed.unwrap_or(true);
    let effective_ripup_costs = if ripup_allowed { ripup_costs } else { -1 };
    let stop = &ctx.stop;
    // fastroute: a progress line now and then (a pass over a big board takes many minutes)
    let mut last_report = Instant::now();
    'components: for component in components {
        for pin in &component.pins {
            if last_report.elapsed().as_secs_f64() >= PROGRESS_LOG_INTERVAL_SECS {
                last_report = Instant::now();
                log::info!(
                    "Fanout pass #{}: {} of {} SMD pins checked in {:.0} s ({} fanned out, {} not routed).",
                    pass_no + 1,
                    total_smd_pin_count - pins_to_go,
                    total_smd_pin_count,
                    pass_start.elapsed().as_secs_f64(),
                    routed,
                    not_routed
                );
            }
            if let Some(m) = settings.fanout.max_items {
                if m > 0 && state.total_items_fanouted >= m {
                    log::info!("Max items limit reached ({m}). Stopping fanout.");
                    break 'components;
                }
            }
            let max_milliseconds = (base_millis_per_pin as f64) * (pass_no + 1) as f64;
            let time_limit = board.time_limits.make(max_milliseconds as i32);
            let net_no = board.item(pin.key).net_number(0);
            if let Some(net) = board.rules.nets.get(net_no) {
                let net_class = &board.rules.net_classes[net.get_net_class()];
                let via_rule = net_class.get_via_rule();
                let has_board_vias = !board.rules.via_rules.is_empty() && board.rules.via_rules[board.rules.via_rules.get(0)].via_count() > 0;
                let fallback_allowed = settings.fanout.fallback_to_board_vias == Some(true) && has_board_vias;
                let can_use_vias = via_rule.map(|r| board.rules.via_rules[r].via_count() > 0).unwrap_or(false) || fallback_allowed;
                if !can_use_vias {
                    pins_to_go -= 1;
                    continue;
                }
            }
            let max_item_id_before = board.communication.id_generator.max_generated_id();
            board.start_marking_changed_area();
            let t_pin = Instant::now();
            let mut result = board.fanout(pin.key, settings, effective_ripup_costs, Some(stop), Some(time_limit));
            let route_secs = t_pin.elapsed().as_secs_f64();
            if result.state == AutorouteAttemptState::Routed {
                if let Some(rejection) = enforce_strict_drc(board, net_no, max_item_id_before) {
                    result = rejection;
                }
            }
            // fastroute: a pin that takes far longer than its time limit is worth a line (the
            // limit bounds the maze search only)
            let pin_secs = t_pin.elapsed().as_secs_f64();
            if pin_secs >= SLOW_PIN_SECS {
                log::info!(
                    "Fanout pass #{}: {} took {pin_secs:.0} s ({:?}; routing {route_secs:.0} s, check {:.0} s).",
                    pass_no + 1,
                    pin_label(board, pin.key),
                    result.state,
                    pin_secs - route_secs
                );
            }
            match result.state {
                AutorouteAttemptState::Routed => {
                    routed += 1;
                    state.total_items_fanouted += 1;
                    fanned.insert(board.item(pin.key).id().0);
                }
                AutorouteAttemptState::Failed => {
                    not_routed += 1;
                    state.total_items_fanouted += 1;
                }
                AutorouteAttemptState::InsertError => {
                    insert_errors += 1;
                    state.total_items_fanouted += 1;
                }
                _ => {}
            }
            pins_to_go -= 1;
            ctx.observe(board, &super::LiveEvent::Fanout { pass_no: pass_no + 1, done: total_smd_pin_count - pins_to_go, total: total_smd_pin_count, routed, not_routed });
            if let Some(d) = state.deadline {
                if Instant::now() >= d {
                    log::info!("Fanout stage timed out.");
                    state.timed_out = true;
                    return routed;
                }
            }
            if stop.is_stop_autorouter_requested() {
                return routed;
            }
        }
    }
    let extra_vias = 0.max(board.get_vias().len() as i32 - vias_before);
    log::info!(
        "Fanout pass #{} completed in {:.2} seconds with {} SMD pin{} fanouted, {} not routed, {} insert error{}, +{} extra via{} ({} SMD pin{} still to check in pass, ripup costs={}).",
        pass_no + 1,
        pass_start.elapsed().as_secs_f64(),
        routed,
        if routed == 1 { "" } else { "s" },
        not_routed,
        insert_errors,
        if insert_errors == 1 { "" } else { "s" },
        extra_vias,
        if extra_vias == 1 { "" } else { "s" },
        pins_to_go,
        if pins_to_go == 1 { "" } else { "s" },
        ripup_costs
    );
    routed
}
