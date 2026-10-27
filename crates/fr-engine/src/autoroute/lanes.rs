//! fastroute escape lanes (`--escape-lanes`): the strip in front of each pad of a dense part
//! (see [`crate::pipeline::escape`]) is reserved for the pad's own net.
//!
//! At a fine-pitch part the room to escape is the strip right outside the pad row. A trace
//! that runs along the row, or cuts across it to reach a pin further on, takes that room from
//! every pad it passes, and those pads then fail. Here the routing cost of every other net
//! through a lane cell is multiplied by a factor, so traces leave the pad row straight and
//! keep their distance, and a pad keeps its way out.

use std::collections::HashMap;

use fr_geom::{FloatPoint, IntBox};

use crate::board::RoutingBoard;
use crate::ids::{LayerNo, NetNo};
use crate::pipeline::escape::DensePart;

/// Cell size of the lane grid, in µm.
pub const CELL_UM: f64 = 250.0;

/// See the module comment.
#[derive(Clone, Debug)]
pub struct EscapeLanes {
    /// cell size in board units
    cell: f64,
    /// the routing cost of other nets in a lane cell is multiplied by this
    factor: f64,
    /// the nets whose lanes cover the cell
    cells: HashMap<(LayerNo, i32, i32), Vec<NetNo>>,
    /// the box around all lanes (a quick test for steps far from every part)
    bounds: IntBox,
}

impl EscapeLanes {
    /// The lanes of the pins of `parts`: from the pad centre outwards to `length_um` beyond
    /// the pad, as wide as the pin pitch.
    pub fn new(board: &RoutingBoard, parts: &[DensePart], length_um: f64, factor: f64) -> Self {
        let um = board.communication.get_resolution(crate::structure::Unit::Um);
        let mut lanes = EscapeLanes { cell: CELL_UM * um, factor, cells: HashMap::new(), bounds: IntBox::EMPTY };
        for part in parts {
            let b = &part.bbox;
            for &p in &part.pins {
                let it = board.item(p);
                let Some(&net) = it.net_numbers().first() else { continue };
                let layer = it.first_layer(board);
                let c = it.center(board).to_float();
                let pad = it.bounding_box(board);
                // the side of the part the pad is on: the nearest edge of the pads' box
                let sides = [
                    (c.x - b.ll.x as f64, (-1.0, 0.0), (c.x - pad.ll.x as f64)),
                    (b.ur.x as f64 - c.x, (1.0, 0.0), (pad.ur.x as f64 - c.x)),
                    (c.y - b.ll.y as f64, (0.0, -1.0), (c.y - pad.ll.y as f64)),
                    (b.ur.y as f64 - c.y, (0.0, 1.0), (pad.ur.y as f64 - c.y)),
                ];
                let &(_, (dx, dy), half) = sides.iter().min_by(|a, b| a.0.total_cmp(&b.0)).unwrap();
                let len = half + length_um * um;
                let w = 0.5 * part.pitch;
                let (nx, ny) = (-dy, dx);
                let step = lanes.cell / 2.0;
                let n_len = (len / step).ceil() as i32;
                let n_w = (w / step).ceil() as i32;
                for i in 0..=n_len {
                    for j in -n_w..=n_w {
                        let t = i as f64 * step;
                        let s = (j as f64 * step).clamp(-w, w);
                        let q = FloatPoint::new(c.x + dx * t + nx * s, c.y + dy * t + ny * s);
                        let k = lanes.key(layer, &q);
                        let nets = lanes.cells.entry(k).or_default();
                        if !nets.contains(&net) {
                            nets.push(net);
                        }
                    }
                }
            }
        }
        let reach = (length_um * um + 4.0 * lanes.cell) as i32;
        for part in parts {
            let b = &part.bbox;
            let grown = IntBox::new(b.ll.x - reach, b.ll.y - reach, b.ur.x + reach, b.ur.y + reach);
            lanes.bounds = lanes.bounds.union_int_box(&grown);
        }
        lanes
    }

    pub fn cell_count(&self) -> usize {
        self.cells.len()
    }

    fn key(&self, layer: LayerNo, p: &FloatPoint) -> (LayerNo, i32, i32) {
        (layer, (p.x / self.cell).floor() as i32, (p.y / self.cell).floor() as i32)
    }

    /// The mean factor on the routing cost of `net` along the step from `a` to `b` on `layer`
    /// (1 outside other nets' lanes).
    pub fn route_factor(&self, layer: LayerNo, a: &FloatPoint, b: &FloatPoint, net: NetNo) -> f64 {
        let bounds = &self.bounds;
        let outside = a.x.max(b.x) < bounds.ll.x as f64
            || a.x.min(b.x) > bounds.ur.x as f64
            || a.y.max(b.y) < bounds.ll.y as f64
            || a.y.min(b.y) > bounds.ur.y as f64;
        if outside || self.cells.is_empty() {
            return 1.0;
        }
        let n = (a.distance(b) / (self.cell / 2.0)).ceil().max(1.0) as usize;
        let mut sum = 0.0;
        for i in 0..n {
            let t = (i as f64 + 0.5) / n as f64;
            let p = FloatPoint::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
            sum += match self.cells.get(&self.key(layer, &p)) {
                Some(nets) if !nets.contains(&net) => self.factor,
                _ => 1.0,
            };
        }
        sum / n as f64
    }
}
