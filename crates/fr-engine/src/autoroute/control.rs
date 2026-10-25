//! Port of `autoroute/maze/AutorouteControl.java` and `autoroute/AutorouteAttemptResult.java`.

use fr_geom::Point;
use fr_settings::{ExpansionCostFactor, RouterSettings};

use crate::board::{AutorouteAttemptState, RoutingBoard};
use crate::ids::{ClearanceClassNo, LayerNo, NetNo};
use crate::rules::ViaRule;

/// Java `AutorouteAttemptResult`: the state and a detail message.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AutorouteAttemptResult {
    pub state: AutorouteAttemptState,
    pub details: String,
}

impl AutorouteAttemptResult {
    /// Java `new AutorouteAttemptResult(state)`.
    pub fn new(state: AutorouteAttemptState) -> Self {
        AutorouteAttemptResult { state, details: String::new() }
    }

    /// Java `new AutorouteAttemptResult(state, details)`.
    pub fn with_details(state: AutorouteAttemptState, details: impl Into<String>) -> Self {
        AutorouteAttemptResult { state, details: details.into() }
    }
}

/// Java `AutorouteControl.ViaMask`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ViaMask {
    pub from_layer: LayerNo,
    pub to_layer: LayerNo,
    pub attach_smd_allowed: bool,
}

/// Java `AutorouteControl`: the parameters of one autoroute connection.
///
/// The `RouterSettings` reference of Java is replaced by the few values the autorouter reads
/// from it ([`Self::start_ripup_costs`], the fanout escape lengths).
#[derive(Clone, Debug)]
pub struct AutorouteControl {
    /// The horizontal and vertical trace costs on each layer.
    pub trace_costs: Vec<ExpansionCostFactor>,
    pub bend_costs: Vec<f64>,
    pub with_neckdown: bool,
    /// fastroute extension: the net's class forbids any neck-down, including
    /// the fanout micro neck-down (controlled-impedance classes).
    pub no_neckdown: bool,
    /// fastroute extension: neck-down never goes below this half width
    /// (`router.min_trace_width_um`; 0 = Java behaviour).
    pub min_trace_half_width: i32,
    /// Defines for each layer, if it may be used for routing.
    pub layer_active: Vec<bool>,
    pub layer_count: i32,
    /// The currently used trace half widths on each layer.
    pub trace_half_width: Vec<i32>,
    /// The compensated trace half widths (equal to `trace_half_width` without clearance
    /// compensation).
    pub compensated_trace_half_width: Vec<i32>,
    pub via_radii: Vec<f64>,
    /// Java `addViaCosts[from].toLayer[to]` (always 0).
    pub add_via_costs: Vec<Vec<i32>>,
    /// The clearance class of the traces.
    pub trace_clearance_class: ClearanceClassNo,
    /// True, if layer changes by vias are allowed.
    pub vias_allowed: bool,
    /// True, if vias may drill to the pad of SMD pins.
    pub attach_smd_allowed: bool,
    /// The minimum cost value of all normal vias.
    pub min_normal_via_cost: f64,
    pub ripup_allowed: bool,
    pub ripup_costs: i32,
    pub ripup_pass_no: i32,
    /// If true, the autoroute algorithm completes after the first drill.
    pub is_fanout: bool,
    pub fanout_start_pin_name: Option<String>,
    pub fanout_start_pin_center: Option<Point>,
    pub fanout_start_pin_layer: LayerNo,
    /// Normally true, if the autorouter contains no fanout pass.
    pub remove_unconnected_vias: bool,
    /// fastroute: accept connections that end in a via into the net's plane on a power layer.
    pub connect_to_planes: bool,
    /// fastroute failure-only history (see [`super::history`]).
    pub history: Option<std::sync::Arc<super::history::FailureHistory>>,
    /// The possible (partial) vias (Java `viaRule`, a copy of the net class rule).
    pub via_rule: ViaRule,
    pub net_number: NetNo,
    /// The clearance class of the vias.
    pub via_clearance_class: ClearanceClassNo,
    /// The via ranges usable by the autorouter.
    pub via_infos: Vec<ViaMask>,
    pub via_lower_bound: LayerNo,
    pub via_upper_bound: LayerNo,
    pub max_via_radius: f64,
    /// The width of the region around changed traces, where traces are pulled tight.
    pub tidy_region_width: i32,
    pub pull_tight_accuracy: i32,
    pub max_shove_trace_recursion_depth: i32,
    pub max_shove_via_recursion_depth: i32,
    pub max_spring_over_recursion_depth: i32,
    /// The minimal cost value of all cheap vias.
    pub min_cheap_via_cost: f64,
    /// `settings.getStartRipupCosts()`.
    pub start_ripup_costs: i32,
    /// `settings.fanout.minEscapeLengthMm`.
    pub fanout_min_escape_length_mm: Option<f64>,
    /// `settings.fanout.maxEscapeLengthMm`.
    pub fanout_max_escape_length_mm: Option<f64>,
}

impl AutorouteControl {
    /// fastroute extension: a neck-down half width raised to the board minimum
    /// (`router.min_trace_width_um`); unchanged when no minimum is set.
    pub fn clamp_neckdown_half_width(&self, half_width: i32) -> i32 {
        if self.min_trace_half_width > 0 && half_width > 0 {
            half_width.max(self.min_trace_half_width)
        } else {
            half_width
        }
    }

    /// Java `new AutorouteControl(board, netNumber, settings)`.
    pub fn new_default(board: &RoutingBoard, net_number: NetNo, settings: &RouterSettings) -> Self {
        let mut c = Self::base(board, settings, settings.get_trace_costs());
        c.init_net(net_number, board, settings.get_via_costs());
        c
    }

    /// Java `new AutorouteControl(board, netNumber, settings, viaCosts, traceCosts)`.
    pub fn new(board: &RoutingBoard, net_number: NetNo, settings: &RouterSettings, via_costs: i32, trace_costs: Vec<ExpansionCostFactor>) -> Self {
        let mut c = Self::base(board, settings, trace_costs);
        c.init_net(net_number, board, via_costs);
        c
    }

    fn base(board: &RoutingBoard, settings: &RouterSettings, trace_costs: Vec<ExpansionCostFactor>) -> Self {
        let layer_count = board.layer_count();
        let n = layer_count.max(0) as usize;
        let bend_costs = (0..n).map(|i| settings.get_bend_cost(i)).collect();
        let mut layer_active = vec![false; n];
        for (i, active) in layer_active.iter_mut().enumerate() {
            let active_setting = settings.get_layer_active(i);
            if !board.layer_structure.layers[i].is_signal && active_setting {
                log::warn!(
                    "Layer '{}' is a dedicated power plane and cannot be routed. Forcing active state to false.",
                    board.layer_structure.layers[i].name
                );
                *active = false;
            } else {
                *active = active_setting;
            }
        }
        AutorouteControl {
            trace_costs,
            bend_costs,
            with_neckdown: settings.get_automatic_neckdown(),
            no_neckdown: false,
            min_trace_half_width: min_trace_half_width(board, settings),
            layer_active,
            layer_count,
            trace_half_width: vec![0; n],
            compensated_trace_half_width: vec![0; n],
            via_radii: vec![0.0; n],
            add_via_costs: vec![vec![0; n]; n],
            trace_clearance_class: 0,
            vias_allowed: settings.get_vias_allowed(),
            attach_smd_allowed: false,
            min_normal_via_cost: 0.0,
            ripup_allowed: false,
            ripup_costs: 1000,
            ripup_pass_no: 1,
            is_fanout: false,
            fanout_start_pin_name: None,
            fanout_start_pin_center: None,
            fanout_start_pin_layer: -1,
            remove_unconnected_vias: true,
            connect_to_planes: false,
            history: None,
            via_rule: ViaRule::empty(),
            net_number: 0,
            via_clearance_class: 0,
            via_infos: Vec::new(),
            via_lower_bound: 0,
            via_upper_bound: layer_count,
            max_via_radius: 0.0,
            tidy_region_width: i32::MAX,
            pull_tight_accuracy: 500,
            max_shove_trace_recursion_depth: 20,
            max_shove_via_recursion_depth: 8,
            max_spring_over_recursion_depth: 8,
            min_cheap_via_cost: 0.0,
            start_ripup_costs: settings.get_start_ripup_costs(),
            fanout_min_escape_length_mm: settings.fanout.min_escape_length_mm,
            fanout_max_escape_length_mm: settings.fanout.max_escape_length_mm,
        }
    }

    fn is_pure_smd_net(board: &RoutingBoard, net_number: NetNo) -> bool {
        let net_items = board.get_connectable_items(net_number);
        if net_items.is_empty() {
            return false;
        }
        for key in net_items {
            let item = board.item(key);
            if !item.is_pin() || item.first_layer(board) != item.last_layer(board) {
                return false;
            }
        }
        true
    }

    fn has_smd_pin(board: &RoutingBoard, net_number: NetNo) -> bool {
        board.get_connectable_items(net_number).into_iter().any(|key| {
            let item = board.item(key);
            item.is_pin() && item.first_layer(board) == item.last_layer(board)
        })
    }

    fn init_net(&mut self, net_number: NetNo, board: &RoutingBoard, via_costs: i32) {
        self.net_number = net_number;
        let rules = &board.rules;
        let net_class = rules.nets.get(net_number).map(|n| &rules.net_classes[n.get_net_class()]);
        match net_class {
            Some(nc) => {
                self.trace_clearance_class = nc.get_trace_clearance_class();
                self.via_rule = match nc.get_via_rule() {
                    Some(r) => rules.via_rules[r].clone(),
                    None => {
                        log::warn!("AutorouteControl: net class without via rule");
                        ViaRule::empty()
                    }
                };
            }
            None => {
                self.trace_clearance_class = 1;
                self.via_rule = match rules.via_rules.get_first() {
                    Some(r) => rules.via_rules[r].clone(),
                    None => ViaRule::empty(),
                };
            }
        }
        for i in 0..self.layer_count {
            let iu = i as usize;
            self.trace_half_width[iu] =
                if net_number > 0 { rules.get_trace_half_width(net_number, i) } else { rules.get_trace_half_width(1, i) };
            self.compensated_trace_half_width[iu] = self.trace_half_width[iu]
                .wrapping_add(rules.clearance_matrix.clearance_compensation_value(self.trace_clearance_class, i));
            if let Some(nc) = net_class {
                if !nc.is_active_routing_layer(i) {
                    self.layer_active[iu] = false;
                }
                if nc.no_neckdown {
                    self.with_neckdown = false;
                    self.no_neckdown = true;
                }
            }
        }
        self.rebuild_via_info(board, via_costs, net_number);
    }

    /// Java `rebuildViaInfo(board, viaCosts, netNumber)`.
    pub fn rebuild_via_info(&mut self, board: &RoutingBoard, via_costs: i32, net_number: NetNo) {
        let rules = &board.rules;
        let padstacks = &board.library.padstacks;
        if self.via_rule.via_count() > 0 {
            self.via_clearance_class = rules.via_infos[self.via_rule.get_via(0)].get_clearance_class_index();
        } else {
            self.via_clearance_class = 1;
        }
        self.via_infos = Vec::with_capacity(self.via_rule.via_count().max(0) as usize);
        self.attach_smd_allowed = false;
        for i in 0..self.via_rule.via_count() {
            let current_via = &rules.via_infos[self.via_rule.get_via(i)];
            if current_via.attach_smd_allowed() {
                self.attach_smd_allowed = true;
            }
            let padstack = padstacks.get(current_via.get_padstack()).expect("AutorouteControl: via padstack");
            let from_layer = padstack.from_layer();
            let to_layer = padstack.to_layer();
            for j in from_layer..=to_layer {
                let current_radius = match padstack.get_shape(j) {
                    Some(s) => 0.5 * s.max_width(),
                    None => 0.0,
                };
                let ju = j as usize;
                self.via_radii[ju] = jmax(self.via_radii[ju], current_radius);
            }
            self.via_infos.push(ViaMask { from_layer, to_layer, attach_smd_allowed: current_via.attach_smd_allowed() });
        }
        let pure_smd_net = Self::is_pure_smd_net(board, net_number);
        let has_smd = Self::has_smd_pin(board, net_number);
        if !self.attach_smd_allowed && self.layer_count > 1 && has_smd {
            self.attach_smd_allowed = true;
            for v in self.via_infos.iter_mut() {
                v.attach_smd_allowed = true;
            }
        }
        for j in 0..self.layer_count as usize {
            self.via_radii[j] = jmax(self.via_radii[j], self.trace_half_width[j] as f64);
            self.max_via_radius = jmax(self.max_via_radius, self.via_radii[j]);
        }
        let mut via_cost_factor = self.max_via_radius;
        via_cost_factor = jmax(via_cost_factor, 1.0);
        if pure_smd_net {
            via_cost_factor *= 0.1;
        }
        self.min_normal_via_cost = via_costs as f64 * via_cost_factor;
        self.min_cheap_via_cost = 0.8 * self.min_normal_via_cost;
    }
}

/// Java `Math.max(double, double)`.
#[inline]
pub(crate) fn jmax(a: f64, b: f64) -> f64 {
    if a.is_nan() || b.is_nan() {
        f64::NAN
    } else if a == 0.0 && b == 0.0 {
        if a.is_sign_negative() && b.is_sign_negative() {
            -0.0
        } else {
            0.0
        }
    } else if a > b {
        a
    } else {
        b
    }
}

/// Java `Math.min(double, double)`.
#[inline]
pub(crate) fn jmin(a: f64, b: f64) -> f64 {
    if a.is_nan() || b.is_nan() {
        f64::NAN
    } else if a == 0.0 && b == 0.0 {
        if a.is_sign_negative() || b.is_sign_negative() {
            -0.0
        } else {
            0.0
        }
    } else if a < b {
        a
    } else {
        b
    }
}

/// Board-unit half width for `router.min_trace_width_um` (0 if unset); converted
/// like the necked-retry width in `router.rs`.
fn min_trace_half_width(board: &RoutingBoard, settings: &RouterSettings) -> i32 {
    let um = settings.get_min_trace_width_um();
    if um <= 0.0 {
        return 0;
    }
    let resolution = board.communication.resolution.max(1);
    let width = fr_jcompat::math_round(crate::structure::Unit::scale(
        um * resolution as f64,
        crate::structure::Unit::Um,
        board.communication.unit,
    )) as i32;
    // Round the half width up so that 2 * half >= the requested width.
    (width + 1) / 2
}
