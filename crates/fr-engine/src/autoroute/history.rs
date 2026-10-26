//! fastroute failure-only history (`--failure-history`): a cost history on the board, raised
//! only where connections that keep failing want to go.
//!
//! PathFinder (McMurchie & Ebeling 1995) raises the cost of every congested resource between
//! iterations. An earlier attempt at that here (every ripped region, on a 1 mm grid) made the
//! quick suite worse: it also penalised regions that are crowded but passable. This history is
//! narrower. When a connection fails for the second time, the batch autorouter routes it alone
//! on the pristine board (to find connections that cannot be routed at all); the path it finds
//! there is the corridor the connection wants. Each pass, the corridors of the connections that
//! failed in it add to the history of their cells, and the history of the earlier passes
//! decays. In the maze search the history then
//! * makes routing through a corridor cell more expensive for every other net, so traces move
//!   out of the way, and
//! * makes ripping items in the corridor cheaper for the net the corridor belongs to.
//!
//! Each run builds its own history. Starting the multi-start variants from the first run's
//! history instead routed worse (the corridors of the first run's late passes then steer the
//! variants' first passes).
//!
//! The history only changes between passes (the parallel pass reads a snapshot), so the
//! routing stays deterministic.

use std::collections::HashMap;

use fr_geom::FloatPoint;

use crate::ids::{LayerNo, NetNo};

/// Cell size of the history grid, in µm.
pub const CELL_UM: f64 = 500.0;
/// The history of the earlier passes is multiplied by this before a pass adds its failures.
pub const DECAY: f32 = 0.5;
/// Cells below this weight are dropped.
const MIN_WEIGHT: f32 = 0.05;
/// The routing cost through a cell of another net's corridor is multiplied by
/// `1 + ROUTE_FACTOR * weight`.
pub const ROUTE_FACTOR: f64 = 0.5;
/// The rip-up cost of an item in the net's own corridor is divided by `1 + RIP_FACTOR * weight`.
pub const RIP_FACTOR: f64 = 1.0;

/// The corridor of a connection: the cells (layer, x, y) of its path on the pristine board.
pub type Corridor = Vec<(LayerNo, i32, i32)>;

#[derive(Clone, Debug, Default)]
struct Cell {
    weight: f32,
    /// the nets whose corridors pass the cell
    nets: Vec<NetNo>,
}

/// See the module comment.
#[derive(Clone, Debug)]
pub struct FailureHistory {
    /// cell size in board units
    cell: f64,
    cells: HashMap<(LayerNo, i32, i32), Cell>,
}

impl FailureHistory {
    /// An empty history for a board with `units_per_um` board units per µm.
    pub fn new(units_per_um: f64) -> Self {
        FailureHistory { cell: CELL_UM * units_per_um, cells: HashMap::new() }
    }

    pub fn is_empty(&self) -> bool {
        self.cells.is_empty()
    }

    pub fn cell_count(&self) -> usize {
        self.cells.len()
    }

    fn key(&self, layer: LayerNo, p: &FloatPoint) -> (LayerNo, i32, i32) {
        (layer, (p.x / self.cell).floor() as i32, (p.y / self.cell).floor() as i32)
    }

    /// The cells of a path given as polylines (layer, corners), each cell once.
    pub fn corridor(&self, paths: &[(LayerNo, Vec<FloatPoint>)]) -> Corridor {
        let mut out: Vec<(LayerNo, i32, i32)> = Vec::new();
        let step = self.cell / 2.0;
        for (layer, corners) in paths {
            for w in corners.windows(2) {
                let (a, b) = (w[0], w[1]);
                let len = a.distance(&b);
                let n = (len / step).ceil().max(1.0) as usize;
                for i in 0..=n {
                    let t = i as f64 / n as f64;
                    let p = FloatPoint::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
                    out.push(self.key(*layer, &p));
                }
            }
        }
        out.sort_unstable();
        out.dedup();
        out
    }

    /// Ends a pass: decays the history, then adds the corridors of the connections (by net)
    /// that failed in the pass.
    pub fn update<'a>(&mut self, failed: impl IntoIterator<Item = (NetNo, &'a Corridor)>) {
        self.cells.retain(|_, c| {
            c.weight *= DECAY;
            c.weight >= MIN_WEIGHT
        });
        for (net, corridor) in failed {
            for &k in corridor {
                let c = self.cells.entry(k).or_default();
                c.weight += 1.0;
                if !c.nets.contains(&net) {
                    c.nets.push(net);
                }
            }
        }
    }

    /// The factor on the routing cost of `net` at `p` on `layer` (1 outside other nets'
    /// corridors).
    pub fn route_factor(&self, layer: LayerNo, p: &FloatPoint, net: NetNo) -> f64 {
        match self.cells.get(&self.key(layer, p)) {
            Some(c) if c.weight >= MIN_WEIGHT && !c.nets.contains(&net) => 1.0 + ROUTE_FACTOR * c.weight as f64,
            _ => 1.0,
        }
    }

    /// The divisor of the rip-up cost for `net` at `p` on `layer` (1 outside its own corridor).
    pub fn rip_divisor(&self, layer: LayerNo, p: &FloatPoint, net: NetNo) -> f64 {
        match self.cells.get(&self.key(layer, p)) {
            Some(c) if c.weight >= MIN_WEIGHT && c.nets.contains(&net) => 1.0 + RIP_FACTOR * c.weight as f64,
            _ => 1.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn corridor_decay_and_owner() {
        let mut h = FailureHistory::new(1.0);
        let c = h.corridor(&[(0, vec![FloatPoint::new(0.0, 0.0), FloatPoint::new(1200.0, 0.0)])]);
        assert_eq!(c, vec![(0, 0, 0), (0, 1, 0), (0, 2, 0)]);
        h.update([(7, &c)]);
        let p = FloatPoint::new(600.0, 100.0);
        assert_eq!(h.route_factor(0, &p, 3), 1.0 + ROUTE_FACTOR);
        assert_eq!(h.route_factor(0, &p, 7), 1.0);
        assert_eq!(h.route_factor(1, &p, 3), 1.0);
        assert_eq!(h.rip_divisor(0, &p, 7), 1.0 + RIP_FACTOR);
        h.update([]);
        assert_eq!(h.route_factor(0, &p, 3), 1.0 + ROUTE_FACTOR * 0.5);
        for _ in 0..4 {
            h.update([]);
        }
        assert!(h.is_empty());
    }
}
