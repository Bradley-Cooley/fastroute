//! fastroute global routing with negotiated congestion (PathFinder, McMurchie & Ebeling 1995).
//!
//! A coarse [`RouteGraph`] of the board says how many traces can cross each boundary between
//! its regions and how many vias fit into each region. [`global_route`] routes every unrouted
//! connection on this graph, again and again: the cost of a boundary grows with its present
//! overuse and with a history of its overuse, so the nets negotiate who gets the scarce
//! boundaries, until none is overused (or the iterations run out). The result is a corridor of
//! regions for every net, which the maze search then prefers (see [`GlobalCorridors`]).
//!
//! Two graphs:
//! * [`GraphKind::Squares`]: square tiles per layer; a boundary's capacity is its free length
//!   divided by the trace pitch. The gaps between the pads of a fine-pitch part lie inside the
//!   tiles, so this graph does not see the escape bottlenecks; the exits of a connection's own
//!   end tiles are free of congestion (its own pads block them in the graph).
//! * [`GraphKind::Triangles`]: a Delaunay triangulation per layer over the pad centres (plus
//!   keepout corners, points on the board edge and fill points in open areas), as in the
//!   topological routers (SURF, Dai, Dayan & Staepelaere). The regions are the triangles; a
//!   triangle side between two pads is the real gap between them, and its capacity is that gap
//!   divided by the pitch. Routes start in the triangles around their pins.
//!
//! Everything here is deterministic: connections are routed in a fixed order, ties are broken
//! by node index.

use std::cmp::Reverse;
use std::collections::{BTreeMap, BinaryHeap, HashMap, HashSet};

use fr_geom::{FloatPoint, IntPoint, TileShape};
use fr_settings::RouterSettings;

use crate::autoroute::AutorouteControl;
use crate::board::item::{ItemKind, ObstacleKind};
use crate::board::{ItemKey, RoutingBoard};
use crate::datastructures::planar_delaunay_triangulation::PlanarDelaunayTriangulation;
use crate::drc::DesignRulesChecker;
use crate::ids::{LayerNo, NetNo};

/// Tile side of the square graph in trace pitches (trace width + clearance).
pub const TILE_PITCHES: f64 = 6.0;
/// Spacing of the fill points of the triangle graph in trace pitches.
pub const FILL_PITCHES: f64 = 8.0;
/// At most this many PathFinder iterations.
pub const MAX_ITERATIONS: usize = 40;
/// The present-congestion factor starts here and grows by [`PRES_GROWTH`] per iteration.
const PRES_START: f64 = 0.5;
const PRES_GROWTH: f64 = 1.6;
/// Added to the history of an overused boundary per unit of overuse and iteration.
const HIST_STEP: f64 = 0.3;
/// A boundary the net already uses costs this fraction of its base cost (Steiner sharing).
const SHARED_FACTOR: f64 = 0.2;

/// Which graph [`build_graph`] builds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GraphKind {
    Squares,
    Triangles,
}

/// The sizes the graphs are built for (a net of the default class).
#[derive(Clone, Copy, Debug)]
struct Sizes {
    half_width: f64,
    clearance: f64,
    /// trace pitch (width + clearance)
    pitch: f64,
    via_radius: f64,
    via_pitch: f64,
}

/// The blocked cells of a fine grid (half a pitch per cell) over the board: for a trace centre
/// on each routing layer and for a via centre.
struct Raster {
    x0: f64,
    y0: f64,
    res: f64,
    fx: usize,
    fy: usize,
    trace_blocked: Vec<Vec<bool>>,
    via_blocked: Vec<bool>,
}

impl Raster {
    fn cell(&self, p: &FloatPoint) -> Option<usize> {
        let cx = ((p.x - self.x0) / self.res).floor();
        let cy = ((p.y - self.y0) / self.res).floor();
        (cx >= 0.0 && cy >= 0.0 && (cx as usize) < self.fx && (cy as usize) < self.fy).then(|| cy as usize * self.fx + cx as usize)
    }

    fn centre(&self, cx: usize, cy: usize) -> FloatPoint {
        FloatPoint::new(self.x0 + (cx as f64 + 0.5) * self.res, self.y0 + (cy as f64 + 0.5) * self.res)
    }
}

/// Whether an item blocks traces and vias (for the global graph).
fn blocks(board: &RoutingBoard, kind: &ItemKind) -> (bool, bool) {
    match kind {
        ItemKind::Pin(_) | ItemKind::Via(_) | ItemKind::Trace(_) | ItemKind::BoardOutline(_) => (true, true),
        ItemKind::ObstacleArea(a) => match a.kind {
            ObstacleKind::ViaKeepout => (false, true),
            ObstacleKind::WireKeepout => (true, board.layer_count() <= 2),
            _ => (true, true),
        },
        ItemKind::ConductionArea(c) => (c.is_obstacle, c.is_obstacle),
        ItemKind::ComponentOutline(_) => (false, false),
    }
}

fn rasterize(board: &RoutingBoard, layers: &[LayerNo], sizes: &Sizes, cells_per_tile: usize) -> Option<Raster> {
    let bb = board.bounding_box();
    let (w, h) = ((bb.ur.x - bb.ll.x) as f64, (bb.ur.y - bb.ll.y) as f64);
    if w <= 0.0 || h <= 0.0 {
        return None;
    }
    let res = sizes.pitch / 2.0;
    let tile = res * cells_per_tile as f64;
    let fx = ((w / tile).ceil() as usize).max(1) * cells_per_tile;
    let fy = ((h / tile).ceil() as usize).max(1) * cells_per_tile;
    let (x0, y0) = (bb.ll.x as f64, bb.ll.y as f64);
    let mut trace_blocked = vec![vec![false; fx * fy]; layers.len()];
    let mut via_blocked = vec![false; fx * fy];
    let trace_margin = sizes.clearance + sizes.half_width;
    let via_margin = sizes.clearance + sizes.via_radius;
    let margin = trace_margin.max(via_margin);
    for key in board.get_items() {
        let (blocks_traces, blocks_vias) = blocks(board, &board.item(key).kind);
        if !blocks_traces && !blocks_vias {
            continue;
        }
        for i in 0..board.tile_shape_count(key) {
            let Some(shape) = board.tile_shape(key, i) else { continue };
            let li = layers.iter().position(|&l| l == board.shape_layer(key, i));
            let sb = shape.bounding_box();
            let cx0 = ((sb.ll.x as f64 - margin - x0) / res).floor().max(0.0) as usize;
            let cy0 = ((sb.ll.y as f64 - margin - y0) / res).floor().max(0.0) as usize;
            let cx1 = (((sb.ur.x as f64 + margin - x0) / res).ceil().max(0.0) as usize).min(fx);
            let cy1 = (((sb.ur.y as f64 + margin - y0) / res).ceil().max(0.0) as usize).min(fy);
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
    Some(Raster { x0, y0, res, fx, fy, trace_blocked, via_blocked })
}

/// The via sites of the raster: a grid of via pitch, the free ones.
fn via_sites(r: &Raster, sizes: &Sizes) -> Vec<FloatPoint> {
    let step = ((sizes.via_pitch / r.res).ceil() as usize).max(1);
    let mut out = Vec::new();
    let mut cy = step / 2;
    while cy < r.fy {
        let mut cx = step / 2;
        while cx < r.fx {
            if !r.via_blocked[cy * r.fx + cx] {
                out.push(r.centre(cx, cy));
            }
            cx += step;
        }
        cy += step;
    }
    out
}

/// How a point is located in the graph (for the corridors and the connection ends).
#[derive(Clone, Debug)]
enum Locator {
    Grid { x0: f64, y0: f64, tile: f64, nx: usize, ny: usize },
    Tri(Vec<TriLayer>),
}

/// The triangles of one layer with a bucket grid for point location.
#[derive(Clone, Debug)]
struct TriLayer {
    /// node index of triangle 0 of the layer
    first_node: usize,
    tris: Vec<[FloatPoint; 3]>,
    x0: f64,
    y0: f64,
    cell: f64,
    nx: usize,
    ny: usize,
    buckets: Vec<Vec<u32>>,
}

impl TriLayer {
    fn new(first_node: usize, tris: Vec<[FloatPoint; 3]>, x0: f64, y0: f64, w: f64, h: f64, cell: f64) -> Self {
        let nx = ((w / cell).ceil() as usize).max(1);
        let ny = ((h / cell).ceil() as usize).max(1);
        let mut buckets = vec![Vec::new(); nx * ny];
        for (i, t) in tris.iter().enumerate() {
            let (minx, maxx) = (t.iter().map(|p| p.x).fold(f64::MAX, f64::min), t.iter().map(|p| p.x).fold(f64::MIN, f64::max));
            let (miny, maxy) = (t.iter().map(|p| p.y).fold(f64::MAX, f64::min), t.iter().map(|p| p.y).fold(f64::MIN, f64::max));
            let bx = |x: f64| (((x - x0) / cell).floor().max(0.0) as usize).min(nx - 1);
            let by = |y: f64| (((y - y0) / cell).floor().max(0.0) as usize).min(ny - 1);
            for y in by(miny)..=by(maxy) {
                for x in bx(minx)..=bx(maxx) {
                    buckets[y * nx + x].push(i as u32);
                }
            }
        }
        TriLayer { first_node, tris, x0, y0, cell, nx, ny, buckets }
    }

    fn locate(&self, p: &FloatPoint) -> Option<usize> {
        let x = (((p.x - self.x0) / self.cell).floor().max(0.0) as usize).min(self.nx - 1);
        let y = (((p.y - self.y0) / self.cell).floor().max(0.0) as usize).min(self.ny - 1);
        let bucket = &self.buckets[y * self.nx + x];
        let inside = |t: &[FloatPoint; 3]| {
            let s = |a: &FloatPoint, b: &FloatPoint| (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
            let (d1, d2, d3) = (s(&t[0], &t[1]), s(&t[1], &t[2]), s(&t[2], &t[0]));
            !((d1 < 0.0 || d2 < 0.0 || d3 < 0.0) && (d1 > 0.0 || d2 > 0.0 || d3 > 0.0))
        };
        if let Some(&i) = bucket.iter().find(|&&i| inside(&self.tris[i as usize])) {
            return Some(self.first_node + i as usize);
        }
        // outside every triangle (rounding, or outside the hull): the nearest centroid
        bucket
            .iter()
            .min_by(|&&a, &&b| centroid(&self.tris[a as usize]).distance(p).total_cmp(&centroid(&self.tris[b as usize]).distance(p)))
            .map(|&i| self.first_node + i as usize)
    }
}

fn centroid(t: &[FloatPoint; 3]) -> FloatPoint {
    FloatPoint::new((t[0].x + t[1].x + t[2].x) / 3.0, (t[0].y + t[1].y + t[2].y) / 3.0)
}

/// The global routing graph (see the module comment).
#[derive(Clone, Debug)]
pub struct RouteGraph {
    pub kind: GraphKind,
    /// the routing layers (signal layers that are active), by index
    pub layers: Vec<LayerNo>,
    /// per node: its layer index and centre
    pub node_layer: Vec<usize>,
    pub pos: Vec<FloatPoint>,
    /// per node: (neighbour, edge)
    pub adj: Vec<Vec<(usize, usize)>>,
    /// per edge: capacity, base cost, whether it is a via resource
    pub cap: Vec<u16>,
    pub base: Vec<f64>,
    pub is_via: Vec<bool>,
    /// the exits of a connection's end nodes are free of congestion
    pub free_end_exits: bool,
    /// the smallest cost per board unit of trace (for the A* estimate)
    pub unit_cost: f64,
    /// the trace pitch in board units
    pub pitch: f64,
    locator: Locator,
    /// triangle graph: the nodes around each pin (by item id) on each of its layers
    pin_nodes: HashMap<i32, Vec<usize>>,
}

impl RouteGraph {
    pub fn node_count(&self) -> usize {
        self.pos.len()
    }

    pub fn layer_index(&self, layer: LayerNo) -> Option<usize> {
        self.layers.iter().position(|&l| l == layer)
    }

    /// The node containing `p` on `layer`.
    pub fn locate(&self, layer: LayerNo, p: &FloatPoint) -> Option<usize> {
        let li = self.layer_index(layer)?;
        match &self.locator {
            Locator::Grid { x0, y0, tile, nx, ny } => {
                let x = ((p.x - x0) / tile).floor().clamp(0.0, (*nx - 1) as f64) as usize;
                let y = ((p.y - y0) / tile).floor().clamp(0.0, (*ny - 1) as f64) as usize;
                Some((li * ny + y) * nx + x)
            }
            Locator::Tri(layers) => layers[li].locate(p),
        }
    }

    /// The nodes a connection end at item `key` (at `p`) starts in.
    fn end_nodes(&self, board: &RoutingBoard, key: ItemKey, p: &FloatPoint) -> Vec<usize> {
        let it = board.item(key);
        if let Some(nodes) = self.pin_nodes.get(&it.id().0) {
            return nodes.clone();
        }
        let basic: &crate::board::BasicBoard = board;
        let mut out: Vec<usize> = (it.first_layer(basic)..=it.last_layer(basic)).filter_map(|l| self.locate(l, p)).collect();
        out.sort_unstable();
        out.dedup();
        out
    }
}

/// Builds the global graph of `board` for the trace and via sizes of `ctrl` (a net of the
/// default class).
pub fn build_graph(board: &RoutingBoard, ctrl: &AutorouteControl, settings: &RouterSettings, kind: GraphKind) -> Option<RouteGraph> {
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
    let sizes = Sizes { half_width, clearance, pitch, via_radius, via_pitch: 2.0 * via_radius + clearance };
    let costs = settings.get_trace_costs();
    let layer_cost = |li: usize, horizontal: bool| {
        costs.get(layers[li] as usize).map(|c| if horizontal { c.horizontal } else { c.vertical }).unwrap_or(1.0)
    };
    let unit_cost = (0..layers.len()).flat_map(|li| [layer_cost(li, true), layer_cost(li, false)]).fold(f64::MAX, f64::min).max(0.1);
    let via_cost = ctrl.min_normal_via_cost.max(pitch);
    match kind {
        GraphKind::Squares => {
            let k = (TILE_PITCHES * 2.0).round() as usize;
            let r = rasterize(board, &layers, &sizes, k)?;
            Some(square_graph(&r, &sizes, layers.clone(), k, &layer_cost, unit_cost, via_cost))
        }
        GraphKind::Triangles => {
            let r = rasterize(board, &layers, &sizes, 1)?;
            Some(triangle_graph(board, &r, &sizes, layers.clone(), &layer_cost, unit_cost, via_cost))
        }
    }
}

fn square_graph(
    r: &Raster,
    sizes: &Sizes,
    layers: Vec<LayerNo>,
    k: usize,
    layer_cost: &dyn Fn(usize, bool) -> f64,
    unit_cost: f64,
    via_cost: f64,
) -> RouteGraph {
    let (nx, ny) = (r.fx / k, r.fy / k);
    let nl = layers.len();
    let nt = nx * ny;
    let tile = r.res * k as f64;
    let fx = r.fx;
    // traces across a run of free cells along a boundary
    let tracks = |run: usize| if run == 0 { 0 } else { (run - 1) / 2 + 1 };
    let boundary = |b: &Vec<bool>, cells: &mut dyn Iterator<Item = (usize, usize)>| {
        let (mut cap, mut run) = (0, 0);
        for (c1, c2) in cells {
            if b[c1] || b[c2] {
                cap += tracks(run);
                run = 0;
            } else {
                run += 1;
            }
        }
        (cap + tracks(run)) as u16
    };
    let node = |li: usize, x: usize, y: usize| (li * ny + y) * nx + x;
    let mut g = RouteGraph {
        kind: GraphKind::Squares,
        node_layer: (0..nl * nt).map(|n| n / nt).collect(),
        pos: (0..nl * nt)
            .map(|n| {
                let t = n % nt;
                FloatPoint::new(r.x0 + ((t % nx) as f64 + 0.5) * tile, r.y0 + ((t / nx) as f64 + 0.5) * tile)
            })
            .collect(),
        adj: vec![Vec::new(); nl * nt],
        cap: Vec::new(),
        base: Vec::new(),
        is_via: Vec::new(),
        free_end_exits: true,
        unit_cost,
        pitch: sizes.pitch,
        locator: Locator::Grid { x0: r.x0, y0: r.y0, tile, nx, ny },
        pin_nodes: HashMap::new(),
        layers,
    };
    let add_edge = |g: &mut RouteGraph, a: usize, b: usize, cap: u16, base: f64, via: bool| {
        let e = g.cap.len();
        g.cap.push(cap);
        g.base.push(base);
        g.is_via.push(via);
        g.adj[a].push((b, e));
        g.adj[b].push((a, e));
    };
    for li in 0..nl {
        let b = &r.trace_blocked[li];
        for ty in 0..ny {
            for tx in 0..nx {
                if tx + 1 < nx {
                    let cx = (tx + 1) * k;
                    let cap = boundary(b, &mut (ty * k..(ty + 1) * k).map(|cy| (cy * fx + cx, cy * fx + cx - 1)));
                    add_edge(&mut g, node(li, tx, ty), node(li, tx + 1, ty), cap, tile * layer_cost(li, true), false);
                }
                if ty + 1 < ny {
                    let cy = (ty + 1) * k;
                    let cap = boundary(b, &mut (tx * k..(tx + 1) * k).map(|cx| (cy * fx + cx, (cy - 1) * fx + cx)));
                    add_edge(&mut g, node(li, tx, ty), node(li, tx, ty + 1), cap, tile * layer_cost(li, false), false);
                }
            }
        }
    }
    // vias: one resource per tile, shared by all layer changes in it
    let mut via_cap = vec![0u16; nt];
    for p in via_sites(r, sizes) {
        let x = (((p.x - r.x0) / tile) as usize).min(nx - 1);
        let y = (((p.y - r.y0) / tile) as usize).min(ny - 1);
        via_cap[y * nx + x] += 1;
    }
    for t in 0..nt {
        let e = g.cap.len();
        g.cap.push(via_cap[t]);
        g.base.push(via_cost);
        g.is_via.push(true);
        for la in 0..nl {
            for lb in 0..nl {
                if la != lb {
                    g.adj[la * nt + t].push((lb * nt + t, e));
                }
            }
        }
    }
    g
}

/// A vertex of the triangulation: a point with the obstacle shapes it stands for.
struct Vertex {
    p: IntPoint,
    shapes: Vec<TileShape>,
    /// item id of the pin
    pin: Option<i32>,
}

fn triangle_graph(
    board: &RoutingBoard,
    r: &Raster,
    sizes: &Sizes,
    layers: Vec<LayerNo>,
    layer_cost: &dyn Fn(usize, bool) -> f64,
    unit_cost: f64,
    via_cost: f64,
) -> RouteGraph {
    let nl = layers.len();
    let bb = board.bounding_box();
    let (w, h) = ((bb.ur.x - bb.ll.x) as f64, (bb.ur.y - bb.ll.y) as f64);
    let mut g = RouteGraph {
        kind: GraphKind::Triangles,
        layers: layers.clone(),
        node_layer: Vec::new(),
        pos: Vec::new(),
        adj: Vec::new(),
        cap: Vec::new(),
        base: Vec::new(),
        is_via: Vec::new(),
        free_end_exits: false,
        unit_cost,
        pitch: sizes.pitch,
        locator: Locator::Tri(Vec::new()),
        pin_nodes: HashMap::new(),
    };
    let mut tri_layers = Vec::new();
    // the obstacles of each layer
    let mut per_layer: Vec<Vec<Vertex>> = (0..nl).map(|_| Vec::new()).collect();
    for key in board.get_items() {
        let it = board.item(key);
        let (blocks_traces, _) = blocks(board, &it.kind);
        if !blocks_traces || matches!(it.kind, ItemKind::BoardOutline(_)) {
            continue;
        }
        let mut shapes: Vec<Vec<TileShape>> = vec![Vec::new(); nl];
        for i in 0..board.tile_shape_count(key) {
            if let (Some(shape), Some(li)) = (board.tile_shape(key, i), layers.iter().position(|&l| l == board.shape_layer(key, i))) {
                shapes[li].push(shape);
            }
        }
        for (li, s) in shapes.into_iter().enumerate() {
            if s.is_empty() {
                continue;
            }
            match &it.kind {
                ItemKind::Pin(_) | ItemKind::Via(_) => {
                    let c = it.center(board).to_float();
                    per_layer[li].push(Vertex {
                        p: IntPoint::new(c.x.round() as i32, c.y.round() as i32),
                        shapes: s,
                        pin: matches!(it.kind, ItemKind::Pin(_)).then(|| it.id().0),
                    });
                }
                _ => {
                    // keepouts, traces, areas: the corners of their bounding boxes
                    for shape in &s {
                        let b = shape.bounding_box();
                        for (x, y) in [(b.ll.x, b.ll.y), (b.ur.x, b.ll.y), (b.ur.x, b.ur.y), (b.ll.x, b.ur.y)] {
                            per_layer[li].push(Vertex { p: IntPoint::new(x, y), shapes: vec![shape.clone()], pin: None });
                        }
                    }
                }
            }
        }
    }
    let fill = FILL_PITCHES * sizes.pitch;
    for (li, verts) in per_layer.iter_mut().enumerate() {
        // points on the board edge and fill points in open areas
        let (nfx, nfy) = (((w / fill).ceil() as usize).max(1), ((h / fill).ceil() as usize).max(1));
        let mut occupied = vec![false; (nfx + 1) * (nfy + 1)];
        let fcell = |p: &IntPoint| {
            let x = (((p.x - bb.ll.x) as f64 / fill).round().max(0.0) as usize).min(nfx);
            let y = (((p.y - bb.ll.y) as f64 / fill).round().max(0.0) as usize).min(nfy);
            y * (nfx + 1) + x
        };
        for v in verts.iter() {
            occupied[fcell(&v.p)] = true;
        }
        for iy in 0..=nfy {
            for ix in 0..=nfx {
                let x = (bb.ll.x as f64 + (ix as f64 * fill).min(w)).round() as i32;
                let y = (bb.ll.y as f64 + (iy as f64 * fill).min(h)).round() as i32;
                let p = IntPoint::new(x, y);
                let edge = ix == 0 || iy == 0 || ix == nfx || iy == nfy;
                let free = r.cell(&FloatPoint::new(x as f64, y as f64)).is_some_and(|c| !r.trace_blocked[li][c]);
                if edge || (free && !occupied[fcell(&p)]) {
                    verts.push(Vertex { p, shapes: Vec::new(), pin: None });
                }
            }
        }
        // one vertex per point (the first one)
        let mut seen = HashSet::new();
        verts.retain(|v| seen.insert((v.p.x, v.p.y)));
        let first_node = g.pos.len();
        let tris = triangulate(verts);
        // nodes
        for t in &tris {
            let pts = t.map(|i| FloatPoint::new(verts[i].p.x as f64, verts[i].p.y as f64));
            g.node_layer.push(li);
            g.pos.push(centroid(&pts));
            g.adj.push(Vec::new());
        }
        // sides shared by two triangles
        let mut sides: BTreeMap<(usize, usize), Vec<usize>> = BTreeMap::new();
        for (ti, t) in tris.iter().enumerate() {
            for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                sides.entry((a.min(b), a.max(b))).or_default().push(ti);
            }
            for &v in t {
                if let Some(pin) = verts[v].pin {
                    g.pin_nodes.entry(pin).or_default().push(first_node + ti);
                }
            }
        }
        let cost = 0.5 * (layer_cost(li, true) + layer_cost(li, false));
        for ((a, b), ts) in sides {
            if ts.len() != 2 {
                continue;
            }
            let cap = side_capacity(&verts[a], &verts[b], sizes);
            let (na, nb) = (first_node + ts[0], first_node + ts[1]);
            let e = g.cap.len();
            g.cap.push(cap);
            g.base.push(g.pos[na].distance(&g.pos[nb]).max(sizes.pitch) * cost);
            g.is_via.push(false);
            g.adj[na].push((nb, e));
            g.adj[nb].push((na, e));
        }
        let tri_pts: Vec<[FloatPoint; 3]> = tris.iter().map(|t| t.map(|i| FloatPoint::new(verts[i].p.x as f64, verts[i].p.y as f64))).collect();
        tri_layers.push(TriLayer::new(first_node, tri_pts, bb.ll.x as f64, bb.ll.y as f64, w, h, fill));
    }
    for v in g.pin_nodes.values_mut() {
        v.sort_unstable();
        v.dedup();
    }
    // vias: the via sites in each triangle; a via edge leads to the triangle of every other
    // layer that contains the centroid
    let mut via_cap = vec![0u16; g.pos.len()];
    for p in via_sites(r, sizes) {
        for tl in &tri_layers {
            if let Some(n) = tl.locate(&p) {
                via_cap[n] = via_cap[n].saturating_add(1);
            }
        }
    }
    for n in 0..g.pos.len() {
        let e = g.cap.len();
        g.cap.push(via_cap[n]);
        g.base.push(via_cost);
        g.is_via.push(true);
        let li = g.node_layer[n];
        for (lj, tl) in tri_layers.iter().enumerate() {
            if lj != li {
                if let Some(m) = tl.locate(&g.pos[n]) {
                    g.adj[n].push((m, e));
                }
            }
        }
    }
    g.locator = Locator::Tri(tri_layers);
    g
}

/// Traces between two vertices: the free gap between their obstacles divided by the pitch.
fn side_capacity(a: &Vertex, b: &Vertex, sizes: &Sizes) -> u16 {
    let (pa, pb) = (FloatPoint::new(a.p.x as f64, a.p.y as f64), FloatPoint::new(b.p.x as f64, b.p.y as f64));
    let len = pa.distance(&pb);
    // how far an obstacle reaches from its vertex towards the other vertex
    let extent = |v: &Vertex, other: &FloatPoint| {
        v.shapes.iter().map(|s| (len - s.distance(other)).clamp(0.0, len)).fold(0.0, f64::max)
    };
    let gap = len - extent(a, &pb) - extent(b, &pa);
    if gap <= sizes.clearance {
        return 0;
    }
    (((gap - sizes.clearance) / sizes.pitch).floor() as u32).min(u16::MAX as u32) as u16
}

/// The Delaunay triangles of `verts` (indices), counter-clockwise.
fn triangulate(verts: &[Vertex]) -> Vec<[usize; 3]> {
    if verts.len() < 3 {
        return Vec::new();
    }
    let dt = PlanarDelaunayTriangulation::<u32, IntPoint>::new_with_own_random(verts.iter().enumerate().map(|(i, v)| (i as u32, [v.p])));
    let mut nbrs: Vec<Vec<usize>> = vec![Vec::new(); verts.len()];
    for e in dt.get_edge_lines() {
        if let (Some(a), Some(b)) = (e.start_object, e.end_object) {
            let (a, b) = (a as usize, b as usize);
            if a != b && !nbrs[a].contains(&b) {
                nbrs[a].push(b);
                nbrs[b].push(a);
            }
        }
    }
    let pt = |i: usize| (verts[i].p.x as f64, verts[i].p.y as f64);
    for (v, ns) in nbrs.iter_mut().enumerate() {
        let (vx, vy) = pt(v);
        ns.sort_by(|&a, &b| {
            let (ax, ay) = pt(a);
            let (bx, by) = pt(b);
            (ay - vy).atan2(ax - vx).total_cmp(&(by - vy).atan2(bx - vx)).then(a.cmp(&b))
        });
    }
    let orient = |a: usize, b: usize, c: usize| {
        let ((ax, ay), (bx, by), (cx, cy)) = (pt(a), pt(b), pt(c));
        (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
    };
    let mut out = Vec::new();
    let mut seen = HashSet::new();
    for a in 0..verts.len() {
        for &b in &nbrs[a] {
            // the face left of a -> b: at b, the neighbour before a in counter-clockwise order
            let nb = &nbrs[b];
            let Some(pos) = nb.iter().position(|&x| x == a) else { continue };
            let c = nb[(pos + nb.len() - 1) % nb.len()];
            if c == a || !nbrs[c].contains(&a) || orient(a, b, c) <= 0.0 {
                continue;
            }
            let mut key = [a, b, c];
            key.sort_unstable();
            if seen.insert(key) {
                out.push([a, b, c]);
            }
        }
    }
    out.sort_unstable();
    out
}

/// A connection to route globally: a net and the nodes of its two ends.
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
    /// boundaries and via resources that are still overused
    pub overused: usize,
    /// the nodes of each net's routes
    pub corridors: BTreeMap<NetNo, Vec<usize>>,
}

/// Routes `connections` on `graph` with negotiated congestion (see the module comment).
pub fn global_route(graph: &RouteGraph, connections: &[GlobalConnection]) -> GlobalResult {
    let ne = graph.cap.len();
    let nn = graph.node_count();
    let mut hist = vec![0.0f64; ne];
    // per edge: the nets using it (with a count of their connections)
    let mut users: Vec<Vec<(NetNo, u16)>> = vec![Vec::new(); ne];
    let mut routes: Vec<Vec<usize>> = vec![Vec::new(); connections.len()];
    let mut nodes_of: Vec<Vec<usize>> = vec![Vec::new(); connections.len()];
    let mut pres = PRES_START;
    let mut result = GlobalResult { connections: connections.len(), ..Default::default() };
    // A* scratch
    let mut dist = vec![f64::INFINITY; nn];
    let mut prev: Vec<(usize, usize)> = vec![(usize::MAX, usize::MAX); nn];
    let mut touched: Vec<usize> = Vec::new();
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
            let is_end = |x: usize| c.from.contains(&x) || c.to.contains(&x);
            let target = graph.pos[c.to[0]];
            let heur = |n: usize| graph.pos[n].distance(&target) * graph.unit_cost * 0.5;
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
                if c.to.contains(&n) {
                    reached = Some(n);
                    break;
                }
                let dn = dist[n];
                for &(m, e) in &graph.adj[n] {
                    let own = users[e].iter().any(|u| u.0 == c.net);
                    let cost = if graph.free_end_exits && (is_end(n) || is_end(m)) {
                        // the pin's own pads block its tile's exits in the square graph
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
                if !(graph.free_end_exits && (is_end(p) || is_end(n))) {
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
    for e in 0..ne {
        let occ = occupancy(&users[e]);
        if occ > graph.cap[e] as usize {
            let n = graph.adj.iter().position(|a| a.iter().any(|x| x.1 == e)).unwrap_or(0);
            log::debug!(
                target: "fr_engine::pipeline::diag",
                "global overflow: {} on layer {} near ({:.0}, {:.0}): {occ} nets, capacity {}",
                if graph.is_via[e] { "vias" } else { "boundary" },
                graph.layers[graph.node_layer[n]],
                graph.pos[n].x,
                graph.pos[n].y,
                graph.cap[e]
            );
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
/// plane nets and nets the autorouter ignores, with the nodes of their ends.
pub fn board_connections(board: &RoutingBoard, g: &RouteGraph) -> Vec<GlobalConnection> {
    let basic: &crate::board::BasicBoard = board;
    let mut out = Vec::new();
    for a in DesignRulesChecker::new().get_all_airlines(basic) {
        let Some(net) = basic.rules.nets.get(a.net_number) else { continue };
        if net.contains_plane() || basic.rules.net_classes[net.get_net_class()].is_ignored_by_autorouter {
            continue;
        }
        let (from, to) = (g.end_nodes(board, a.from_item, &a.from_corner), g.end_nodes(board, a.to_item, &a.to_corner));
        if !from.is_empty() && !to.is_empty() {
            out.push(GlobalConnection { net: a.net_number, from, to });
        }
    }
    out
}

/// The corridors of the global routes, for the maze search: routing a net outside the nodes
/// of its corridor (and their neighbours on the same layer) costs more.
#[derive(Clone, Debug)]
pub struct GlobalCorridors {
    graph: RouteGraph,
    nets: HashMap<NetNo, HashSet<usize>>,
    /// the cost factor outside the corridor
    pub factor: f64,
}

impl GlobalCorridors {
    pub fn new(graph: RouteGraph, result: &GlobalResult, factor: f64) -> Self {
        let mut nets = HashMap::new();
        for (&net, nodes) in &result.corridors {
            let mut set = HashSet::new();
            for &n in nodes {
                set.insert(n);
                for &(m, e) in &graph.adj[n] {
                    if !graph.is_via[e] {
                        set.insert(m);
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
        match self.graph.locate(layer, p) {
            Some(n) if !set.contains(&n) => self.factor,
            _ => 1.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn open_graph(nx: usize, ny: usize, cap: u16) -> RouteGraph {
        let r = Raster { x0: 0.0, y0: 0.0, res: 5.0, fx: nx * 2, fy: ny * 2, trace_blocked: vec![vec![false; nx * ny * 4]], via_blocked: vec![true; nx * ny * 4] };
        let sizes = Sizes { half_width: 1.0, clearance: 1.0, pitch: 10.0 / (cap as f64 * 2.0 - 1.0).max(1.0), via_radius: 1.0, via_pitch: 3.0 };
        let mut g = square_graph(&r, &sizes, vec![0], 2, &|_, _| 1.0, 1.0, 100.0);
        for (e, c) in g.cap.iter_mut().enumerate() {
            if !g.is_via[e] {
                *c = cap;
            }
        }
        g
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
        let r = global_route(&g, &conns);
        assert_eq!(r.overflow, 0, "one net must take the detour through another row");
        assert!(r.iterations > 1);
        assert_eq!(r.corridors.len(), 2);
    }

    #[test]
    fn triangles_of_a_square_with_centre() {
        let v = |x, y| Vertex { p: IntPoint::new(x, y), shapes: Vec::new(), pin: None };
        let verts = vec![v(0, 0), v(10, 0), v(10, 10), v(0, 10), v(5, 5)];
        let tris = triangulate(&verts);
        assert_eq!(tris.len(), 4);
        assert!(tris.iter().all(|t| t.contains(&4)));
    }
}
