//! fastroute hardest-first ordering (`--hard-first`): the first pass routes the connections
//! that are expected to be hardest first, while the board is still empty around them.
//!
//! Freerouting routes the first pass in board order (the stock multi-start variants shuffle
//! it); later passes put the nets that failed in the pass before first. A connection's
//! difficulty is estimated from its airlines on the unrouted board, see [`OrderKey`].

use std::collections::HashMap;

use fr_geom::FloatPoint;

use crate::board::RoutingBoard;
use crate::drc::checker::DesignRulesChecker;
use crate::ids::NetNo;

/// Radius around an airline end in which [`OrderKey::Density`] counts other nets' pins, in µm.
const DENSITY_RADIUS_UM: f64 = 1000.0;

/// How the difficulty of a net is estimated.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OrderKey {
    /// the airlines of other nets crossing the net's airlines
    Crossings,
    /// the other nets' airline ends near the net's airline ends (dense parts)
    Density,
    /// the total airline length, short nets first
    Short,
    /// the total airline length, long nets first
    Long,
}

impl OrderKey {
    /// The keys of `--order-variants`, best first on the tuning boards.
    pub const ALL: [OrderKey; 4] = [Self::Short, Self::Density, Self::Crossings, Self::Long];

    pub fn parse(s: &str) -> Option<Self> {
        match s {
            "crossings" => Some(Self::Crossings),
            "density" => Some(Self::Density),
            "short" => Some(Self::Short),
            "long" => Some(Self::Long),
            _ => None,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::Crossings => "crossings",
            Self::Density => "density",
            Self::Short => "short",
            Self::Long => "long",
        }
    }
}

/// The rank of each net with airlines: higher ranks are routed first.
pub fn net_ranks(board: &RoutingBoard, key: OrderKey) -> HashMap<NetNo, i64> {
    let basic: &crate::board::BasicBoard = board;
    let lines: Vec<(NetNo, FloatPoint, FloatPoint)> = DesignRulesChecker::new()
        .get_all_airlines(basic)
        .into_iter()
        .map(|a| (a.net_number, a.from_corner, a.to_corner))
        .collect();
    let mut ranks: HashMap<NetNo, i64> = HashMap::new();
    match key {
        OrderKey::Crossings => {
            for (i, &(net, a, b)) in lines.iter().enumerate() {
                let n = lines
                    .iter()
                    .enumerate()
                    .filter(|&(j, &(other, c, d))| j != i && other != net && segments_cross(a, b, c, d))
                    .count();
                *ranks.entry(net).or_default() += n as i64;
            }
        }
        OrderKey::Density => {
            let r = DENSITY_RADIUS_UM * board.communication.get_resolution(crate::structure::Unit::Um);
            let ends: Vec<(NetNo, FloatPoint)> = lines.iter().flat_map(|&(net, a, b)| [(net, a), (net, b)]).collect();
            for &(net, p) in &ends {
                let n = ends.iter().filter(|&&(other, q)| other != net && p.distance(&q) <= r).count();
                *ranks.entry(net).or_default() += n as i64;
            }
        }
        OrderKey::Short | OrderKey::Long => {
            for &(net, a, b) in &lines {
                *ranks.entry(net).or_default() += a.distance(&b).round() as i64;
            }
            if key == OrderKey::Short {
                ranks.values_mut().for_each(|v| *v = -*v);
            }
        }
    }
    ranks
}

/// Whether the segments a-b and c-d cross in their interiors.
fn segments_cross(a: FloatPoint, b: FloatPoint, c: FloatPoint, d: FloatPoint) -> bool {
    let side = |p: FloatPoint, q: FloatPoint, r: FloatPoint| ((q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x)).signum();
    let (d1, d2) = (side(a, b, c), side(a, b, d));
    let (d3, d4) = (side(c, d, a), side(c, d, b));
    d1 * d2 < 0.0 && d3 * d4 < 0.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crossing_segments() {
        let p = FloatPoint::new;
        assert!(segments_cross(p(0.0, 0.0), p(2.0, 2.0), p(0.0, 2.0), p(2.0, 0.0)));
        assert!(!segments_cross(p(0.0, 0.0), p(2.0, 0.0), p(0.0, 1.0), p(2.0, 1.0)));
        // touching at an end is not a crossing
        assert!(!segments_cross(p(0.0, 0.0), p(2.0, 0.0), p(2.0, 0.0), p(2.0, 2.0)));
    }
}
