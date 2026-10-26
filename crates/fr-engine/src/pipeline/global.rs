//! fastroute global routing with negotiated congestion (PathFinder, McMurchie & Ebeling 1995).
//!
//! The board is cut into square tiles on every routing layer. A [`CapacityGraph`] says how
//! many traces can cross each tile boundary (the free length of the boundary divided by the
//! trace pitch, after pads, keepouts and existing copper) and how many vias fit into each
//! tile. [`global_route`] then routes every unrouted connection on this graph, again and
//! again: the cost of a boundary grows with its present overuse and with a history of its
//! overuse, so the nets negotiate who gets the scarce boundaries, until none is overused (or
//! the iterations run out). The result is a corridor of tiles for every net, which the maze
//! search then prefers (see [`GlobalCorridors`]).
//!
//! Everything here is deterministic: connections are routed in a fixed order, ties are broken
//! by node index.

use std::cmp::Reverse;
use std::collections::{BTreeMap, BinaryHeap, HashMap};

use fr_geom::FloatPoint;
use fr_settings::RouterSettings;

use crate::autoroute::AutorouteControl;
use crate::board::item::{ItemKind, ObstacleKind};
use crate::board::{ItemKey, RoutingBoard};
use crate::drc::DesignRulesChecker;
use crate::ids::{LayerNo, NetNo};

/// Tile side in trace pitches (trace width + clearance).
pub const TILE_PITCHES: f64 = 6.0;
/// At most this many PathFinder iterations.
pub const MAX_ITERATIONS: usize = 40;
/// The present-congestion factor starts here and grows by [`PRES_GROWTH`] per iteration.
const PRES_START: f64 = 0.5;
const PRES_GROWTH: f64 = 1.6;
/// Added to the history of an overused boundary per unit of overuse and iteration.
const HIST_STEP: f64 = 0.3;
/// A boundary the net already uses costs this fraction of its base cost (Steiner sharing).
const SHARED_FACTOR: f64 = 0.2;

/// The capacities of the tiles of a board.
#[derive(Clone, Debug)]
pub struct CapacityGraph {
    /// the routing layers (signal layers that are active), by index `li`
    pub layers: Vec<LayerNo>,
    pub x0: f64,
    pub y0: f64,
    /// tile side in board units
    pub tile: f64,
    pub nx: usize,
    pub ny: usize,
    /// traces across the boundary between tile (x, y) and (x + 1, y), per layer index
    pub h_cap: Vec<Vec<u16>>,
    /// traces across the boundary between tile (x, y) and (x, y + 1), per layer index
    pub v_cap: Vec<Vec<u16>>,
    /// vias that fit into each tile
    pub via_cap: Vec<u16>,
    /// trace pitch in board units
    pub pitch: f64,
}

impl CapacityGraph {
    pub fn tile_of(&self, p: &FloatPoint) -> (usize, usize) {
        let x = ((p.x - self.x0) / self.tile).floor().clamp(0.0, (self.nx - 1) as f64) as usize;
        let y = ((p.y - self.y0) / self.tile).floor().clamp(0.0, (self.ny - 1) as f64) as usize;
        (x, y)
    }

    pub fn layer_index(&self, layer: LayerNo) -> Option<usize> {
        self.layers.iter().position(|&l| l == layer)
    }

    /// Builds the graph of `board` for the trace and via sizes of `ctrl` (a net of the default
    /// class).
    pub fn build(board: &RoutingBoard, ctrl: &AutorouteControl) -> Option<CapacityGraph> {
        let layers: Vec<LayerNo> =
            (0..board.layer_count()).filter(|&l| ctrl.layer_active.get(l as usize).copied().unwrap_or(false)).collect();
        if layers.is_empty() {
            return None;
        }
        let l0 = layers[0];
        let half_width = ctrl.trace_half_width[l0 as usize].max(1) as f64;
        let clearance = board.rules.clearance_matrix.get_value(ctrl.trace_clearance_class, ctrl.trace_clearance_class, l0, false) as f64;
        let pitch = 2.0 * half_width + clearance;
        let via_radius = layers.iter().map(|&l| ctrl.via_radii[l as usize]).fold(half_width, f64::max);
        let via_pitch = 2.0 * via_radius + clearance;
        let bb = board.bounding_box();
        let (w, h) = ((bb.ur.x - bb.ll.x) as f64, (bb.ur.y - bb.ll.y) as f64);
        if w <= 0.0 || h <= 0.0 {
            return None;
        }
        // the fine grid: half a pitch per cell, a whole number of cells per tile
        let res = pitch / 2.0;
        let k = (TILE_PITCHES * 2.0).round() as usize;
        let tile = res * k as f64;
        let (nx, ny) = (((w / tile).ceil() as usize).max(1), ((h / tile).ceil() as usize).max(1));
        let (fx, fy) = (nx * k, ny * k);
        let (x0, y0) = (bb.ll.x as f64, bb.ll.y as f64);
        // blocked fine cells per layer: for a trace centre (margin: clearance + half width)
        // and for a via centre (margin: clearance + via radius)
        let mut trace_blocked = vec![vec![false; fx * fy]; layers.len()];
        let mut via_blocked = vec![false; fx * fy];
        let trace_margin = clearance + half_width;
        let via_margin = clearance + via_radius;
        for key in board.get_items() {
            let it = board.item(key);
            let (blocks_traces, blocks_vias) = match &it.kind {
                ItemKind::Pin(_) | ItemKind::Via(_) | ItemKind::Trace(_) | ItemKind::BoardOutline(_) => (true, true),
                ItemKind::ObstacleArea(a) => match a.kind {
                    ObstacleKind::ViaKeepout => (false, true),
                    ObstacleKind::WireKeepout => (true, board.layer_count() <= 2),
                    _ => (true, true),
                },
                ItemKind::ConductionArea(c) => (c.is_obstacle, c.is_obstacle),
                ItemKind::ComponentOutline(_) => (false, false),
            };
            if !blocks_traces && !blocks_vias {
                continue;
            }
            for i in 0..board.tile_shape_count(key) {
                let Some(shape) = board.tile_shape(key, i) else { continue };
                let layer = board.shape_layer(key, i);
                let li = layers.iter().position(|&l| l == layer);
                let sb = shape.bounding_box();
                let margin = trace_margin.max(via_margin);
                let cx0 = (((sb.ll.x as f64 - margin - x0) / res).floor().max(0.0)) as usize;
                let cy0 = (((sb.ll.y as f64 - margin - y0) / res).floor().max(0.0)) as usize;
                let cx1 = ((((sb.ur.x as f64 + margin - x0) / res).ceil()) as usize).min(fx);
                let cy1 = ((((sb.ur.y as f64 + margin - y0) / res).ceil()) as usize).min(fy);
                for cy in cy0..cy1 {
                    for cx in cx0..cx1 {
                        let p = FloatPoint::new(x0 + (cx as f64 + 0.5) * res, y0 + (cy as f64 + 0.5) * res);
                        let d = shape.distance(&p);
                        if blocks_traces && d < trace_margin {
                            if let Some(li) = li {
                                trace_blocked[li][cy * fx + cx] = true;
                            }
                        }
                        if blocks_vias && d < via_margin {
                            via_blocked[cy * fx + cx] = true;
                        }
                    }
                }
            }
        }
        // traces across a run of free cells along a boundary
        let tracks = |run: usize| if run == 0 { 0 } else { (run - 1) / 2 + 1 };
        let mut h_cap = vec![vec![0u16; nx * ny]; layers.len()];
        let mut v_cap = vec![vec![0u16; nx * ny]; layers.len()];
        for li in 0..layers.len() {
            let b = &trace_blocked[li];
            for ty in 0..ny {
                for tx in 0..nx {
                    // boundary to the right: fine column (tx + 1) * k, rows of the tile
                    if tx + 1 < nx {
                        let cx = (tx + 1) * k;
                        let (mut cap, mut run) = (0, 0);
                        for cy in ty * k..(ty + 1) * k {
                            if b[cy * fx + cx] || b[cy * fx + cx - 1] {
                                cap += tracks(run);
                                run = 0;
                            } else {
                                run += 1;
                            }
                        }
                        cap += tracks(run);
                        h_cap[li][ty * nx + tx] = cap as u16;
                    }
                    if ty + 1 < ny {
                        let cy = (ty + 1) * k;
                        let (mut cap, mut run) = (0, 0);
                        for cx in tx * k..(tx + 1) * k {
                            if b[cy * fx + cx] || b[(cy - 1) * fx + cx] {
                                cap += tracks(run);
                                run = 0;
                            } else {
                                run += 1;
                            }
                        }
                        cap += tracks(run);
                        v_cap[li][ty * nx + tx] = cap as u16;
                    }
                }
            }
        }
        // via sites: a grid of via pitch in each tile, free on the via grid
        let step = ((via_pitch / res).ceil() as usize).max(1);
        let mut via_cap = vec![0u16; nx * ny];
        for ty in 0..ny {
            for tx in 0..nx {
                let mut n = 0;
                let mut cy = ty * k + step / 2;
                while cy < (ty + 1) * k {
                    let mut cx = tx * k + step / 2;
                    while cx < (tx + 1) * k {
                        if !via_blocked[cy * fx + cx] {
                            n += 1;
                        }
                        cx += step;
                    }
                    cy += step;
                }
                via_cap[ty * nx + tx] = n;
            }
        }
        Some(CapacityGraph { layers, x0, y0, tile, nx, ny, h_cap, v_cap, via_cap, pitch })
    }
}

/// A connection to route globally: a net and the tiles (layer index, x, y) of its two ends.
#[derive(Clone, Debug)]
pub struct GlobalConnection {
    pub net: NetNo,
    pub from: Vec<usize>,
    pub to: Vec<usize>,
}

/// The result of [`global_route`].
#[derive(Clone, Debug, Default)]
pub struct GlobalResult {
    pub connections: usize,
    pub iterations: usize,
    /// total overuse of the boundaries and via sites after the last iteration
    pub overflow: u32,
    /// boundaries and tiles that are still overused
    pub overused: usize,
    /// the tiles (node indices) of each net's routes
    pub corridors: BTreeMap<NetNo, Vec<usize>>,
}

/// The routing graph: nodes are (layer index, x, y); an edge is a tile boundary on a layer or
/// the via resource of a tile.
struct Graph<'a> {
    g: &'a CapacityGraph,
    nl: usize,
    /// edges: h boundaries (li, x, y), then v boundaries, then the via sites of each tile
    cap: Vec<u16>,
    base: Vec<f64>,
}

impl<'a> Graph<'a> {
    fn new(g: &'a CapacityGraph, settings: &RouterSettings, via_cost: f64) -> Self {
        let nl = g.layers.len();
        let nt = g.nx * g.ny;
        let costs = settings.get_trace_costs();
        let mut cap = Vec::with_capacity(2 * nl * nt + nt);
        let mut base = Vec::with_capacity(2 * nl * nt + nt);
        for li in 0..nl {
            let c = costs.get(g.layers[li] as usize).map(|c| c.horizontal).unwrap_or(1.0);
            cap.extend(&g.h_cap[li]);
            base.extend(std::iter::repeat(g.tile * c).take(nt));
        }
        for li in 0..nl {
            let c = costs.get(g.layers[li] as usize).map(|c| c.vertical).unwrap_or(1.0);
            cap.extend(&g.v_cap[li]);
            base.extend(std::iter::repeat(g.tile * c).take(nt));
        }
        cap.extend(&g.via_cap);
        base.extend(std::iter::repeat(via_cost).take(nt));
        Graph { g, nl, cap, base }
    }

    fn node(&self, li: usize, x: usize, y: usize) -> usize {
        (li * self.g.ny + y) * self.g.nx + x
    }

    fn unnode(&self, n: usize) -> (usize, usize, usize) {
        let nt = self.g.nx * self.g.ny;
        (n / nt, (n % nt) / self.g.nx, n % self.g.nx)
    }

    /// The neighbours of node `n` with the edge between them.
    fn neighbours(&self, n: usize, out: &mut Vec<(usize, usize)>) {
        out.clear();
        let (li, y, x) = self.unnode(n);
        let (nx, ny) = (self.g.nx, self.g.ny);
        let nt = nx * ny;
        let t = y * nx + x;
        let h = |li: usize, t: usize| li * nt + t;
        let v = |li: usize, t: usize| self.nl * nt + li * nt + t;
        if x + 1 < nx {
            out.push((self.node(li, x + 1, y), h(li, t)));
        }
        if x > 0 {
            out.push((self.node(li, x - 1, y), h(li, t - 1)));
        }
        if y + 1 < ny {
            out.push((self.node(li, x, y + 1), v(li, t)));
        }
        if y > 0 {
            out.push((self.node(li, x, y - 1), v(li, t - nx)));
        }
        let via = 2 * self.nl * nt + t;
        for l2 in 0..self.nl {
            if l2 != li {
                out.push((self.node(l2, x, y), via));
            }
        }
    }
}

/// Routes `connections` on `g` with negotiated congestion (see the module comment).
pub fn global_route(g: &CapacityGraph, connections: &[GlobalConnection], settings: &RouterSettings, via_cost: f64) -> GlobalResult {
    let graph = Graph::new(g, settings, via_cost);
    let ne = graph.cap.len();
    let nn = graph.nl * g.nx * g.ny;
    let mut hist = vec![0.0f64; ne];
    // per edge: the nets using it (with a count of their connections)
    let mut users: Vec<Vec<(NetNo, u16)>> = vec![Vec::new(); ne];
    let mut routes: Vec<Vec<usize>> = vec![Vec::new(); connections.len()];
    let mut nodes_of: Vec<Vec<usize>> = vec![Vec::new(); connections.len()];
    let min_base = graph.base.iter().copied().filter(|&b| b > 0.0).fold(f64::MAX, f64::min).min(g.tile);
    let mut pres = PRES_START;
    let mut result = GlobalResult { connections: connections.len(), ..Default::default() };
    // A* scratch
    let mut dist = vec![f64::INFINITY; nn];
    let mut prev: Vec<(usize, usize)> = vec![(usize::MAX, usize::MAX); nn];
    let mut touched: Vec<usize> = Vec::new();
    let mut nb = Vec::new();
    let occupancy = |users: &Vec<(NetNo, u16)>| users.len();
    for iteration in 1..=MAX_ITERATIONS {
        result.iterations = iteration;
        for (ci, c) in connections.iter().enumerate() {
            // only connections that use an overused edge are rerouted after the first iteration
            if iteration > 1 && !routes[ci].iter().any(|&e| occupancy(&users[e]) > graph.cap[e] as usize) {
                continue;
            }
            // rip up
            for &e in &routes[ci] {
                if let Some(pos) = users[e].iter().position(|u| u.0 == c.net) {
                    users[e][pos].1 -= 1;
                    if users[e][pos].1 == 0 {
                        users[e].remove(pos);
                    }
                }
            }
            routes[ci].clear();
            // A* from all `from` nodes to any `to` node
            let targets: Vec<usize> = c.to.clone();
            let (tx, ty) = {
                let (_, y, x) = graph.unnode(targets[0]);
                (x as f64, y as f64)
            };
            let heur = |n: usize| {
                let (_, y, x) = graph.unnode(n);
                ((x as f64 - tx).abs() + (y as f64 - ty).abs()) * min_base * 0.5
            };
            for &n in &touched {
                dist[n] = f64::INFINITY;
                prev[n] = (usize::MAX, usize::MAX);
            }
            touched.clear();
            let mut heap: BinaryHeap<Reverse<(u64, usize)>> = BinaryHeap::new();
            for &s in &c.from {
                dist[s] = 0.0;
                touched.push(s);
                heap.push(Reverse(((heur(s) * 16.0) as u64, s)));
            }
            let mut reached = None;
            while let Some(Reverse((_, n))) = heap.pop() {
                if targets.contains(&n) {
                    reached = Some(n);
                    break;
                }
                let dn = dist[n];
                graph.neighbours(n, &mut nb);
                let at_end = c.from.contains(&n) || targets.contains(&n);
                for &(m, e) in &nb {
                    let own = users[e].iter().any(|u| u.0 == c.net);
                    // the exits of the connection's own end tiles are free of congestion: the
                    // pin's own pads block them in the graph, and its fanout via sits there
                    let cost = if at_end || c.from.contains(&m) || targets.contains(&m) {
                        graph.base[e]
                    } else if own {
                        graph.base[e] * SHARED_FACTOR
                    } else {
                        let over = (occupancy(&users[e]) + 1).saturating_sub(graph.cap[e] as usize) as f64;
                        graph.base[e] * (1.0 + hist[e]) * (1.0 + pres * over)
                    };
                    let d = dn + cost;
                    if d < dist[m] {
                        if dist[m].is_infinite() {
                            touched.push(m);
                        }
                        dist[m] = d;
                        prev[m] = (n, e);
                        heap.push(Reverse((((d + heur(m)) * 16.0) as u64, m)));
                    }
                }
            }
            let Some(mut n) = reached else { continue };
            let mut nodes = vec![n];
            while prev[n].0 != usize::MAX {
                let (p, e) = prev[n];
                let end = |x: usize| c.from.contains(&x) || targets.contains(&x);
                if !end(p) && !end(n) {
                    routes[ci].push(e);
                }
                nodes.push(p);
                n = p;
            }
            for &e in &routes[ci] {
                match users[e].iter().position(|u| u.0 == c.net) {
                    Some(pos) => users[e][pos].1 += 1,
                    None => users[e].push((c.net, 1)),
                }
            }
            nodes_of[ci] = nodes;
        }
        let mut overflow = 0u32;
        let mut overused = 0usize;
        for e in 0..ne {
            let over = occupancy(&users[e]).saturating_sub(graph.cap[e] as usize);
            if over > 0 {
                overflow += over as u32;
                overused += 1;
                hist[e] += HIST_STEP * over as f64;
            }
        }
        result.overflow = overflow;
        result.overused = overused;
        log::debug!(target: "fr_engine::pipeline::diag", "global routing iteration {iteration}: overflow {overflow} on {overused} edges");
        if overflow == 0 {
            break;
        }
        pres *= PRES_GROWTH;
    }
    let nt = g.nx * g.ny;
    for e in 0..ne {
        let occ = occupancy(&users[e]);
        if occ > graph.cap[e] as usize {
            let (kind, li, t) = if e < graph.nl * nt { ("h", e / nt, e % nt) } else if e < 2 * graph.nl * nt { ("v", (e - graph.nl * nt) / nt, e % nt) } else { ("via", 0, e % nt) };
            log::debug!(target: "fr_engine::pipeline::diag", "global overflow: {kind} layer {li} tile ({}, {}) at ({:.1}, {:.1}) mm: {occ} nets, capacity {}", t % g.nx, t / g.nx, (g.x0 + (t % g.nx) as f64 * g.tile) / 10000.0, (g.y0 + (t / g.nx) as f64 * g.tile) / 10000.0, graph.cap[e]);
        }
    }
    for (ci, c) in connections.iter().enumerate() {
        result.corridors.entry(c.net).or_default().extend(&nodes_of[ci]);
    }
    for v in result.corridors.values_mut() {
        v.sort_unstable();
        v.dedup();
    }
    result
}

/// The connections of `board` to route globally: its unrouted connections (airlines), without
/// plane nets and nets the autorouter ignores, with the tiles of their ends.
pub fn board_connections(board: &RoutingBoard, g: &CapacityGraph) -> Vec<GlobalConnection> {
    let basic: &crate::board::BasicBoard = board;
    let graph_node = |li: usize, (x, y): (usize, usize)| (li * g.ny + y) * g.nx + x;
    let ends = |key: ItemKey, p: &FloatPoint| -> Vec<usize> {
        let it = board.item(key);
        let t = g.tile_of(p);
        (it.first_layer(basic)..=it.last_layer(basic)).filter_map(|l| g.layer_index(l)).map(|li| graph_node(li, t)).collect()
    };
    let mut out = Vec::new();
    for a in DesignRulesChecker::new().get_all_airlines(basic) {
        let Some(net) = basic.rules.nets.get(a.net_number) else { continue };
        if net.contains_plane() || basic.rules.net_classes[net.get_net_class()].is_ignored_by_autorouter {
            continue;
        }
        let (from, to) = (ends(a.from_item, &a.from_corner), ends(a.to_item, &a.to_corner));
        if !from.is_empty() && !to.is_empty() {
            out.push(GlobalConnection { net: a.net_number, from, to });
        }
    }
    out
}

/// The corridors of the global routes, for the maze search: routing a net outside the tiles
/// of its corridor (and their neighbours) costs more.
#[derive(Clone, Debug)]
pub struct GlobalCorridors {
    graph: CapacityGraph,
    /// per net: the tiles (layer index, x, y as node indices) of its corridor, widened by one
    /// tile in the plane
    nets: HashMap<NetNo, std::collections::HashSet<usize>>,
    /// the cost factor outside the corridor
    pub factor: f64,
}

impl GlobalCorridors {
    pub fn new(graph: CapacityGraph, result: &GlobalResult, factor: f64) -> Self {
        let (nx, ny) = (graph.nx, graph.ny);
        let mut nets = HashMap::new();
        for (&net, nodes) in &result.corridors {
            let mut set = std::collections::HashSet::new();
            for &n in nodes {
                let nt = nx * ny;
                let (li, t) = (n / nt, n % nt);
                let (x, y) = (t % nx, t / nx);
                for dy in -1i64..=1 {
                    for dx in -1i64..=1 {
                        let (xx, yy) = (x as i64 + dx, y as i64 + dy);
                        if xx >= 0 && yy >= 0 && (xx as usize) < nx && (yy as usize) < ny {
                            set.insert(li * nt + yy as usize * nx + xx as usize);
                        }
                    }
                }
            }
            nets.insert(net, set);
        }
        GlobalCorridors { graph, nets, factor }
    }

    /// The factor on the routing cost of `net` at `p` on `layer`.
    pub fn route_factor(&self, layer: LayerNo, p: &FloatPoint, net: NetNo) -> f64 {
        let Some(set) = self.nets.get(&net) else { return 1.0 };
        let Some(li) = self.graph.layer_index(layer) else { return 1.0 };
        let (x, y) = self.graph.tile_of(p);
        if set.contains(&((li * self.graph.ny + y) * self.graph.nx + x)) {
            1.0
        } else {
            self.factor
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn open_graph(nx: usize, ny: usize, cap: u16) -> CapacityGraph {
        CapacityGraph {
            layers: vec![0],
            x0: 0.0,
            y0: 0.0,
            tile: 10.0,
            nx,
            ny,
            h_cap: vec![vec![cap; nx * ny]],
            v_cap: vec![vec![cap; nx * ny]],
            via_cap: vec![0; nx * ny],
            pitch: 1.0,
        }
    }

    #[test]
    fn negotiation_spreads_nets() {
        // a 5 x 3 board, capacity 1 per boundary: two nets from the left to the right edge in
        // the middle row cannot both take the straight way (the exits of their end tiles are
        // free)
        let g = open_graph(5, 3, 1);
        let node = |x: usize, y: usize| y * 5 + x;
        let conns = vec![
            GlobalConnection { net: 1, from: vec![node(0, 1)], to: vec![node(4, 1)] },
            GlobalConnection { net: 2, from: vec![node(0, 1)], to: vec![node(4, 1)] },
        ];
        let r = global_route(&g, &conns, &RouterSettings::default(), 100.0);
        assert_eq!(r.overflow, 0, "one net must take the detour through another row");
        assert!(r.iterations > 1);
        let conns = vec![
            GlobalConnection { net: 1, from: vec![node(0, 0)], to: vec![node(2, 1)] },
            GlobalConnection { net: 2, from: vec![node(0, 1)], to: vec![node(2, 1)] },
            GlobalConnection { net: 3, from: vec![node(0, 2)], to: vec![node(2, 2)] },
        ];
        let r = global_route(&g, &conns, &RouterSettings::default(), 100.0);
        assert!(r.iterations >= 1);
        assert_eq!(r.corridors.len(), 3);
    }
}
