//! The autoroute entry points of `RoutingBoard` (`initAutoroute`, `autoroute`, `fanout`) and of
//! `AutorouteEngine.autorouteConnection`, and the per connection routing of the batch autorouter
//! (`pipeline/AutorouteConnectionRouter.route` without the strict DRC check, see
//! [`route_connection`]).

use std::collections::{BTreeSet, HashMap};
use std::panic::{catch_unwind, AssertUnwindSafe};

use fr_geom::FloatPoint;
use fr_settings::{ExpansionCostFactor, RouterSettings};

use crate::board::{AutorouteAttemptState, AutorouteMaintenance, ItemKey, ItemSet, RoutingBoard, StopConnectionOption};
use crate::datastructures::{StopToken, TimeLimit};
use crate::ids::{ClearanceClassNo, NetNo};
use crate::rules::ViaRule;
use crate::structure::Unit;

use super::control::{AutorouteAttemptResult, AutorouteControl};
use super::engine::AutorouteEngine;
use super::inserter::FoundConnectionInserter;
use super::locator::FoundConnectionLocator;
use super::maze::{MazeSearchEngine, MazeSearchResult};

/// The autoroute engine stored in a [`RoutingBoard`] (Java `transient AutorouteEngine
/// autorouteEngine`): cloning a board does not clone the engine.
#[derive(Default)]
pub struct EngineSlot(pub Option<Box<AutorouteEngine>>);

impl Clone for EngineSlot {
    fn clone(&self) -> Self {
        EngineSlot(None)
    }
}

impl std::fmt::Debug for EngineSlot {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "EngineSlot({})", if self.0.is_some() { "engine" } else { "none" })
    }
}

impl EngineSlot {
    /// Takes the engine out of the slot.
    pub fn take(&mut self) -> Option<Box<AutorouteEngine>> {
        self.0.take()
    }
}

impl AutorouteEngine {
    /// Java `autorouteConnection(startSet, destSet, ctrl, rippedItemList, ripupCosts)`: routes a
    /// connection between `start_set` and `dest_set`. The ripped items are added to
    /// `ripped_item_list`, their ripup costs to `ripup_costs`.
    pub fn autoroute_connection(
        &mut self,
        board: &mut RoutingBoard,
        start_set: &ItemSet,
        dest_set: &ItemSet,
        ctrl: &AutorouteControl,
        ripped_item_list: &mut ItemSet,
        ripup_costs: Option<&mut HashMap<ItemKey, i32>>,
    ) -> AutorouteAttemptResult {
        self.process_board_changes(board);
        // (panics stand for Java exceptions, which are caught like in Java)
        let search_result = {
            // outer None: the maze search could not be created; inner: the found connection
            let r: Result<Option<Option<MazeSearchResult>>, _> = catch_unwind(AssertUnwindSafe(|| {
                let mut maze = MazeSearchEngine::get_instance(start_set, dest_set, &mut *self, &mut *board, ctrl)?;
                Some(catch_unwind(AssertUnwindSafe(|| maze.find_connection())).unwrap_or_else(|_| {
                    log::error!("AutorouteEngine.autoroute_connection: Exception in mazeSearchAlgo.find_connection");
                    None
                }))
            }));
            let Ok(Some(found)) = r else {
                // (Java returns without clearing the database)
                return AutorouteAttemptResult::with_details(
                    AutorouteAttemptState::Failed,
                    "Failed to route connection, because the maze search algorithm could not be created.",
                );
            };
            found
        };
        let angle = board.rules.get_trace_angle_restriction();
        let autoroute_result = match &search_result {
            Some(r) => catch_unwind(AssertUnwindSafe(|| {
                FoundConnectionLocator::get_instance(r, ctrl, &mut *self, &*board, angle, &mut *ripped_item_list, ripup_costs).ok()
            }))
            .unwrap_or_else(|_| {
                log::error!("AutorouteEngine.autoroute_connection: Exception in FoundConnectionLocator.get_instance");
                None
            }),
            None => None,
        };
        // Always clean up expansion rooms from the search tree, regardless of search outcome.
        if !self.maintain_database {
            self.clear(board);
        } else {
            self.reset_all_doors(board);
        }
        if search_result.is_none() {
            return AutorouteAttemptResult::with_details(
                AutorouteAttemptState::Failed,
                "Failed to route connection, because no connection was found between their nets.",
            );
        }
        let Some(autoroute_result) = autoroute_result else {
            return AutorouteAttemptResult::with_details(AutorouteAttemptState::Failed, "Failed to route connection.");
        };
        // fastroute: an end on an inactive plane layer is fine if it is the net's plane itself
        // (the connection is a via into the plane). Java rejects it, so plane nets only got
        // connected to their plane by the fanout stage.
        let end_ok = |layer: i32, item: Option<ItemKey>| {
            ctrl.layer_active[layer as usize]
                || (ctrl.connect_to_planes
                    && !board.layer_structure.layers[layer as usize].is_signal
                    && item.is_some_and(|k| board.item(k).is_conduction_area()))
        };
        if !end_ok(autoroute_result.start_layer, autoroute_result.start_item)
            || !end_ok(autoroute_result.target_layer, autoroute_result.target_item)
        {
            log::debug!(
                target: "fr_engine::pipeline::diag",
                "located connection on inactive layer: start {} target {} active {:?}",
                autoroute_result.start_layer,
                autoroute_result.target_layer,
                ctrl.layer_active
            );
            return AutorouteAttemptResult::with_details(
                AutorouteAttemptState::Failed,
                "Failed to route connection, because some of their layers are disabled.",
            );
        }
        // Delete the ripped connections.
        let stop_connection_option = if ctrl.remove_unconnected_vias { StopConnectionOption::None } else { StopConnectionOption::FanoutVia };
        let mut ripped_connections = ItemSet::new();
        let mut changed_nets: BTreeSet<NetNo> = BTreeSet::new();
        for ripped in ripped_item_list.iter() {
            ripped_connections.extend_from(&board.get_connection_items(ripped, stop_connection_option));
            for &n in board.item(ripped).net_numbers() {
                changed_nets.insert(n);
            }
        }
        board.remove_items(ripped_connections.iter().collect::<Vec<_>>());
        for net in changed_nets {
            board.remove_trace_tails(net, stop_connection_option);
        }
        if FoundConnectionInserter::get_instance(&autoroute_result, board, ctrl).is_none() {
            return AutorouteAttemptResult::with_details(
                AutorouteAttemptState::Failed,
                "Failed to route connection, because the new connection could not be inserted.",
            );
        }
        AutorouteAttemptResult::new(AutorouteAttemptState::Routed)
    }
}

impl RoutingBoard {
    /// Java `initAutoroute(netNumber, traceClearanceClassIndex, stoppableThread, timeLimit,
    /// retainAutorouteDatabase)`: creates (or, if the database is retained, reuses) the autoroute
    /// engine and initializes it for the net. Use [`Self::autoroute_connection`] afterwards.
    pub fn init_autoroute(
        &mut self,
        net_number: NetNo,
        trace_clearance_class: ClearanceClassNo,
        stop: Option<&StopToken>,
        time_limit: Option<TimeLimit>,
        retain_autoroute_database: bool,
    ) {
        let need_new = match &self.autoroute_engine.0 {
            None => true,
            Some(e) => !retain_autoroute_database || e.tree_clearance_class != trace_clearance_class,
        };
        if need_new {
            // Java replaces the engine without clearing it: objects of the old engine still
            // referenced by the search tree or the items stay alive.
            let old = self.autoroute_engine.take();
            let engine = AutorouteEngine::new_carry(self, trace_clearance_class, retain_autoroute_database, old);
            // (the record exists while the board holds an engine: the item autoroute infos are
            // dropped when Java would null them; rooms are only maintained if retained)
            self.basic.autoroute_maintenance = Some(AutorouteMaintenance {
                tree_clearance_class: engine.tree_clearance_class,
                maintain_database: retain_autoroute_database,
                ..AutorouteMaintenance::default()
            });
            self.autoroute_engine.0 = Some(Box::new(engine));
        }
        let mut engine = self.autoroute_engine.take().unwrap();
        engine.init_connection(self, net_number, stop.cloned(), time_limit);
        self.autoroute_engine.0 = Some(engine);
    }

    /// `initAutoroute(...).autorouteConnection(startSet, destSet, ctrl, rippedItemList,
    /// ripupCosts)` with the engine created by [`Self::init_autoroute`].
    pub fn autoroute_connection(
        &mut self,
        start_set: &ItemSet,
        dest_set: &ItemSet,
        ctrl: &AutorouteControl,
        ripped_item_list: &mut ItemSet,
        ripup_costs: Option<&mut HashMap<ItemKey, i32>>,
    ) -> AutorouteAttemptResult {
        let mut engine = self.autoroute_engine.take().expect("RoutingBoard.autoroute_connection: init_autoroute must be called first");
        let result = engine.autoroute_connection(self, start_set, dest_set, ctrl, ripped_item_list, ripup_costs);
        self.autoroute_engine.0 = Some(engine);
        result
    }

    /// Java `autoroute(item, routerSettings, viaCosts, stoppableThread, timeLimit)`: routes the
    /// item to another item of its net to which it is not yet connected.
    pub fn autoroute(
        &mut self,
        item: ItemKey,
        router_settings: &RouterSettings,
        via_costs: i32,
        stop: Option<&StopToken>,
        time_limit: Option<TimeLimit>,
    ) -> AutorouteAttemptResult {
        let it = self.item(item);
        if !it.is_connectable_class() || it.net_count() == 0 {
            return AutorouteAttemptResult::with_details(AutorouteAttemptState::NoConnections, "The item is not connectable.");
        }
        if it.net_count() > 1 {
            log::warn!("RoutingBoard.autoroute: netCount > 1 not yet implemented");
        }
        let route_net_no = it.net_number(0);
        let mut ctrl = AutorouteControl::new(self, route_net_no, router_settings, via_costs, router_settings.get_trace_costs());
        ctrl.remove_unconnected_vias = false;
        let route_start_set = self.connected_set(item, route_net_no, false);
        if self.rules.nets.get(route_net_no).map(|n| n.contains_plane()).unwrap_or(false) {
            for current in route_start_set.iter() {
                if self.item(current).is_conduction_area() {
                    return AutorouteAttemptResult::with_details(AutorouteAttemptState::ConnectedToPlane, "The item is connected to a plane.");
                }
            }
        }
        let route_dest_set = self.unconnected_set(item, route_net_no);
        if route_dest_set.is_empty() {
            return AutorouteAttemptResult::with_details(AutorouteAttemptState::AlreadyConnected, "The item is already connected.");
        }
        let mut ripped_item_list = ItemSet::new();
        self.init_autoroute(route_net_no, ctrl.trace_clearance_class, stop, time_limit, false);
        let result = self.autoroute_connection(&route_start_set, &route_dest_set, &ctrl, &mut ripped_item_list, None);
        if result.state == AutorouteAttemptState::Routed {
            let accuracy = router_settings.trace_pull_tight_accuracy.unwrap_or(500);
            self.opt_changed_area(&[route_net_no], None, accuracy, Some(&ctrl.trace_costs), stop, 1000);
        }
        result
    }

    /// Java `fanout(pin, routerSettings, ripupCosts, stoppableThread, timeLimit)`: routes from the
    /// pin to the first via, if the pin and its connected set have only 1 layer. Ripup is allowed
    /// if `ripup_costs >= 0`.
    pub fn fanout(
        &mut self,
        pin: ItemKey,
        router_settings: &RouterSettings,
        ripup_costs: i32,
        stop: Option<&StopToken>,
        time_limit: Option<TimeLimit>,
    ) -> AutorouteAttemptResult {
        let p = self.item(pin);
        if p.first_layer(self) != p.last_layer(self) || p.net_count() != 1 {
            return AutorouteAttemptResult::with_details(AutorouteAttemptState::AlreadyConnected, "The pin is already connected.");
        }
        let pin_net_no = p.net_number(0);
        let pin_layer = p.first_layer(self);
        let pin_connected_set = self.connected_set(pin, pin_net_no, false);
        for current in pin_connected_set.iter() {
            let c = self.item(current);
            if c.first_layer(self) != pin_layer || c.last_layer(self) != pin_layer {
                return AutorouteAttemptResult::with_details(AutorouteAttemptState::AlreadyConnected, "The pin is already connected.");
            }
        }
        let unconnected_set = self.unconnected_set(pin, pin_net_no);
        if unconnected_set.is_empty() {
            return AutorouteAttemptResult::with_details(AutorouteAttemptState::NoUnconnectedNets, "The pin is already connected.");
        }
        let pin_center = self.item(pin).center(self).to_float();
        let mut sorted_unconnected_list: Vec<ItemKey> = unconnected_set.iter().collect();
        let dist_sq = |board: &RoutingBoard, k: ItemKey, c: &FloatPoint| {
            let b = board.item(k).bounding_box(board);
            let cx = (b.ll.x as f64 + b.ur.x as f64) / 2.0;
            let cy = (b.ll.y as f64 + b.ur.y as f64) / 2.0;
            let dx = cx - c.x;
            let dy = cy - c.y;
            dx * dx + dy * dy
        };
        // List.sort (stable) with Double.compare
        let keys: Vec<f64> = sorted_unconnected_list.iter().map(|k| dist_sq(self, *k, &pin_center)).collect();
        let mut indexed: Vec<(usize, ItemKey)> = sorted_unconnected_list.iter().copied().enumerate().collect();
        indexed.sort_by(|a, b| java_double_compare(keys[a.0], keys[b.0]));
        sorted_unconnected_list = indexed.into_iter().map(|(_, k)| k).collect();
        let mut ctrl = AutorouteControl::new_default(self, pin_net_no, router_settings);
        ctrl.is_fanout = true;
        if router_settings.fanout.fallback_to_board_vias == Some(true) {
            let mut combined_via_rule = ViaRule::new(format!("{}_fallback", ctrl.via_rule.name));
            for i in 0..ctrl.via_rule.via_count() {
                combined_via_rule.append_via(ctrl.via_rule.get_via(i));
            }
            if let Some(default_rule) = self.rules.via_rules.get_first() {
                let default_via_rule = self.rules.via_rules[default_rule].clone();
                // fastroute: the default vias carry the default clearance class; for a net of a
                // wider class (KiCad applies the net's clearance to its vias) such a via ended
                // up too close to other copper (0.17 mm next to a 0.2 mm class)
                let own_class = (0..ctrl.via_rule.via_count())
                    .next()
                    .map(|i| self.rules.via_infos[ctrl.via_rule.get_via(i)].get_clearance_class_index());
                for i in 0..default_via_rule.via_count() {
                    let default_via = default_via_rule.get_via(i);
                    if self.basic.fallback_vias_own_class
                        && own_class.is_some_and(|c| self.rules.via_infos[default_via].get_clearance_class_index() != c)
                    {
                        continue;
                    }
                    if !combined_via_rule.contains(default_via) {
                        combined_via_rule.append_via(default_via);
                    }
                }
            }
            ctrl.via_rule = combined_via_rule;
            ctrl.rebuild_via_info(self, router_settings.get_via_costs(), pin_net_no);
        }
        // (Java `fanoutStartPinName` is only used for diagnostics)
        let component_no = self.item(pin).component_no();
        ctrl.fanout_start_pin_name = if component_no > 0 { Some(format!("{}-{}", self.components.get(component_no).name, self.item(pin).id().0)) } else { None };
        ctrl.fanout_start_pin_center = Some(self.item(pin).center(self));
        ctrl.fanout_start_pin_layer = pin_layer;
        ctrl.remove_unconnected_vias = false;
        if ripup_costs >= 0 {
            ctrl.ripup_allowed = true;
            ctrl.ripup_costs = ripup_costs;
        }
        let mut ripped_item_list = ItemSet::new();
        self.init_autoroute(pin_net_no, ctrl.trace_clearance_class, stop, time_limit, false);
        let mut result: Option<AutorouteAttemptResult> = None;
        if sorted_unconnected_list.len() <= 4 {
            if let Some(&closest_target) = sorted_unconnected_list.first() {
                // 1. Try to route to the closest target first
                let mut closest = ItemSet::new();
                closest.insert(self.item(closest_target).id(), closest_target);
                let r = self.autoroute_connection(&pin_connected_set, &closest, &ctrl, &mut ripped_item_list, None);
                // 2. If that fails and we have other targets, search the entire unconnected set
                if r.state != AutorouteAttemptState::Routed && r.state != AutorouteAttemptState::AlreadyConnected && sorted_unconnected_list.len() > 1 {
                    result = Some(self.autoroute_connection(&pin_connected_set, &unconnected_set, &ctrl, &mut ripped_item_list, None));
                } else {
                    result = Some(r);
                }
            }
        } else {
            // For large nets route to the entire unconnected set at once
            result = Some(self.autoroute_connection(&pin_connected_set, &unconnected_set, &ctrl, &mut ripped_item_list, None));
        }
        let result = result.unwrap_or_else(|| AutorouteAttemptResult::with_details(AutorouteAttemptState::Failed, "No target items to route connection."));
        if result.state == AutorouteAttemptState::Routed {
            let accuracy = router_settings.trace_pull_tight_accuracy.unwrap_or(500);
            self.opt_changed_area(&[pin_net_no], None, accuracy, Some(&ctrl.trace_costs), stop, 1000);
        }
        result
    }
}

/// Java `Double.compare(a, b)`.
fn java_double_compare(a: f64, b: f64) -> std::cmp::Ordering {
    use std::cmp::Ordering;
    if a < b {
        return Ordering::Less;
    }
    if a > b {
        return Ordering::Greater;
    }
    let ab = if a == 0.0 { a.to_bits() as i64 } else { fr_jcompat::double_to_long_bits(a) };
    let bb = if b == 0.0 { b.to_bits() as i64 } else { fr_jcompat::double_to_long_bits(b) };
    ab.cmp(&bb)
}

/// The parameters of [`route_connection`] taken from the batch autorouter (Java
/// `BatchAutorouter` getters).
#[derive(Clone, Debug)]
pub struct ConnectionRouterParams {
    /// `router.getTraceCosts()`.
    pub trace_costs: Vec<ExpansionCostFactor>,
    /// `router.getStartRipupCosts()`.
    pub start_ripup_costs: i32,
    /// `router.isRemoveUnconnectedVias()`.
    pub remove_unconnected_vias: bool,
    /// `router.isRetainAutorouteDatabase()` (false unless the benchmark system property is set).
    pub retain_autoroute_database: bool,
    /// `router.getTracePullTightAccuracy()`.
    pub trace_pull_tight_accuracy: i32,
    /// fastroute: a connection may end in a via into the net's plane (see
    /// [`AutorouteControl::connect_to_planes`]).
    pub connect_to_planes: bool,
    /// fastroute failure-only history (see [`super::history`]).
    pub history: Option<std::sync::Arc<super::history::FailureHistory>>,
}

/// The result of [`route_connection`].
#[derive(Clone, Debug)]
pub struct ConnectionRouteOutcome {
    pub result: AutorouteAttemptResult,
    /// `communication.idGenerator.maxGeneratedId()` before routing (Java `maxItemIdBeforeRoute`,
    /// input of the strict DRC check); `None` if the routing was not attempted.
    pub max_item_id_before_route: Option<i32>,
}

/// Java `AutorouteConnectionRouter.TIME_LIMIT_TO_PREVENT_ENDLESS_LOOP`.
const TIME_LIMIT_TO_PREVENT_ENDLESS_LOOP: i32 = 1000;

/// Java `AutorouteConnectionRouter.route(item, routeNetNo, rippedItemList, ripupCosts,
/// ripupPassNo)` without the strict DRC check (`applyStrictDrcAfterRoute`), which the caller
/// (porting unit U9) applies when the returned state is `Routed`: Java calls it exactly then,
/// before returning (after the regular route or after a successful necked retry), with
/// [`ConnectionRouteOutcome::max_item_id_before_route`]. The board snapshot Java takes for the
/// strict DRC rollback (if `settings.isStrictDrc()`) can be taken by the caller before calling
/// this function (the snapshot contains no search tree data).
#[allow(clippy::too_many_arguments)]
pub fn route_connection(
    board: &mut RoutingBoard,
    settings: &RouterSettings,
    params: &ConnectionRouterParams,
    item: ItemKey,
    route_net_no: NetNo,
    ripped_item_list: &mut ItemSet,
    ripup_costs: Option<&mut HashMap<ItemKey, i32>>,
    ripup_pass_no: i32,
    stop: Option<&StopToken>,
) -> ConnectionRouteOutcome {
    // Java catches every exception of the routing (`catch (Exception e)` -> FAILED); a panic
    // stands for such an exception.
    let r = catch_unwind(AssertUnwindSafe(|| {
        route_connection_impl(board, settings, params, item, route_net_no, ripped_item_list, ripup_costs, ripup_pass_no, stop)
    }));
    match r {
        Ok(o) => o,
        Err(_) => {
            log::error!("Error during routing passes");
            ConnectionRouteOutcome { result: AutorouteAttemptResult::new(AutorouteAttemptState::Failed), max_item_id_before_route: None }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn route_connection_impl(
    board: &mut RoutingBoard,
    settings: &RouterSettings,
    params: &ConnectionRouterParams,
    item: ItemKey,
    route_net_no: NetNo,
    ripped_item_list: &mut ItemSet,
    mut ripup_costs: Option<&mut HashMap<ItemKey, i32>>,
    ripup_pass_no: i32,
    stop: Option<&StopToken>,
) -> ConnectionRouteOutcome {
    let contains_plane = board.rules.nets.get(route_net_no).map(|n| n.contains_plane()).unwrap_or(false);
    let current_via_costs = if contains_plane { settings.get_plane_via_costs() } else { settings.get_via_costs() };
    let mut ctrl = AutorouteControl::new(board, route_net_no, settings, current_via_costs, params.trace_costs.clone());
    ctrl.ripup_allowed = true;
    ctrl.ripup_costs = params.start_ripup_costs.wrapping_mul(ripup_pass_no);
    ctrl.remove_unconnected_vias = params.remove_unconnected_vias;
    ctrl.connect_to_planes = params.connect_to_planes;
    ctrl.history = params.history.clone();
    let unconnected_set = board.unconnected_set(item, route_net_no);
    if unconnected_set.is_empty() {
        return ConnectionRouteOutcome { result: AutorouteAttemptResult::new(AutorouteAttemptState::NoUnconnectedNets), max_item_id_before_route: None };
    }
    let connected_set = board.connected_set(item, route_net_no, false);
    let (route_start_set, route_dest_set) = if contains_plane {
        for current in connected_set.iter() {
            if board.item(current).is_conduction_area() {
                return ConnectionRouteOutcome {
                    result: AutorouteAttemptResult::new(AutorouteAttemptState::ConnectedToPlane),
                    max_item_id_before_route: None,
                };
            }
        }
        (connected_set, unconnected_set)
    } else {
        (unconnected_set, connected_set)
    };
    let mut max_milliseconds = 100000.0 * 2f64.powi(ripup_pass_no - 1);
    max_milliseconds = super::control::jmin(max_milliseconds, i32::MAX as f64);
    let time_limit = board.time_limits.make(max_milliseconds as i32);
    board.init_autoroute(route_net_no, ctrl.trace_clearance_class, stop, Some(time_limit.clone()), params.retain_autoroute_database);
    let max_item_id_before_route = crate::datastructures::IdGenerator::max_generated_id(&board.communication.id_generator);
    let autoroute_result = board.autoroute_connection(&route_start_set, &route_dest_set, &ctrl, ripped_item_list, ripup_costs.as_deref_mut());
    if autoroute_result.state == AutorouteAttemptState::Routed {
        board.opt_changed_area(&[], None, params.trace_pull_tight_accuracy, Some(&ctrl.trace_costs), stop, TIME_LIMIT_TO_PREVENT_ENDLESS_LOOP);
    }
    if (autoroute_result.state == AutorouteAttemptState::Failed || autoroute_result.state == AutorouteAttemptState::InsertError) && settings.get_neck_width_um() > 0.0 {
        if let Some(necked) = retry_connection_necked(
            board,
            settings,
            params,
            route_net_no,
            &ctrl,
            current_via_costs,
            &route_start_set,
            &route_dest_set,
            ripped_item_list,
            ripup_costs,
            ripup_pass_no,
            time_limit,
            stop,
        ) {
            return ConnectionRouteOutcome { result: necked, max_item_id_before_route: Some(max_item_id_before_route) };
        }
    }
    ConnectionRouteOutcome { result: autoroute_result, max_item_id_before_route: Some(max_item_id_before_route) }
}

/// Java `AutorouteConnectionRouter.retryConnectionNecked(...)`.
#[allow(clippy::too_many_arguments)]
fn retry_connection_necked(
    board: &mut RoutingBoard,
    settings: &RouterSettings,
    params: &ConnectionRouterParams,
    route_net_no: NetNo,
    original_control: &AutorouteControl,
    via_costs: i32,
    route_start_set: &ItemSet,
    route_dest_set: &ItemSet,
    ripped_item_list: &mut ItemSet,
    ripup_costs: Option<&mut HashMap<ItemKey, i32>>,
    ripup_pass_no: i32,
    time_limit: TimeLimit,
    stop: Option<&StopToken>,
) -> Option<AutorouteAttemptResult> {
    if let Some(net) = board.rules.nets.get(route_net_no) {
        if board.rules.net_classes[net.get_net_class()].no_neckdown {
            return None;
        }
    }
    let board_resolution = board.communication.resolution.max(1);
    let neck_width = fr_jcompat::math_round(Unit::scale(settings.get_neck_width_um() * board_resolution as f64, Unit::Um, board.communication.unit)) as i32;
    let neck_half_width = (neck_width / 2).max(1);
    let mut narrower_somewhere = false;
    for i in 0..original_control.layer_count as usize {
        if original_control.layer_active[i] && original_control.trace_half_width[i] > neck_half_width {
            narrower_somewhere = true;
            break;
        }
    }
    if !narrower_somewhere {
        return None;
    }
    let mut neck_control = AutorouteControl::new(board, route_net_no, settings, via_costs, params.trace_costs.clone());
    neck_control.ripup_allowed = true;
    neck_control.ripup_costs = params.start_ripup_costs.wrapping_mul(ripup_pass_no);
    neck_control.remove_unconnected_vias = params.remove_unconnected_vias;
    for i in 0..neck_control.layer_count as usize {
        let compensation = neck_control.compensated_trace_half_width[i] - neck_control.trace_half_width[i];
        neck_control.trace_half_width[i] = neck_control.trace_half_width[i].min(neck_half_width);
        neck_control.compensated_trace_half_width[i] = neck_control.trace_half_width[i] + compensation;
    }
    board.init_autoroute(route_net_no, neck_control.trace_clearance_class, stop, Some(time_limit), params.retain_autoroute_database);
    let neck_result = board.autoroute_connection(route_start_set, route_dest_set, &neck_control, ripped_item_list, ripup_costs);
    if neck_result.state != AutorouteAttemptState::Routed {
        return None;
    }
    board.opt_changed_area(&[], None, params.trace_pull_tight_accuracy, Some(&neck_control.trace_costs), stop, TIME_LIMIT_TO_PREVENT_ENDLESS_LOOP);
    Some(neck_result)
}
