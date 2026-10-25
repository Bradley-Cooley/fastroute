//! Port of `autoroute/path/Connection.java`: a routing connection ending at the next fork or
//! terminal item.

use fr_geom::Point;

use crate::board::{BasicBoard, ItemKey, ItemSet};
use crate::ids::LayerNo;

use super::engine::AutorouteEngine;

const DETOUR_ADD: f64 = 100.0;
const DETOUR_ITEM_COST: f64 = 0.1;

/// Java `Connection`.
#[derive(Clone, Debug)]
pub struct Connection {
    /// `None` if the connection ends in empty space.
    pub start_point: Option<Point>,
    pub start_layer: LayerNo,
    pub end_point: Option<Point>,
    pub end_layer: LayerNo,
    pub item_list: ItemSet,
}

impl Connection {
    /// Java `Connection.get(item)`: the connection the item belongs to (cached in the item's
    /// autoroute info). `None` if the item is not a route item.
    pub fn get(eng: &mut AutorouteEngine, board: &BasicBoard, item: ItemKey) -> Option<usize> {
        if !board.item(item).is_routable() {
            return None;
        }
        if let Some(c) = eng.item_info_mut(item).precalculated_connection {
            return Some(c);
        }
        let contacts = board.normal_contacts(item);
        let mut connection_items = ItemSet::new();
        connection_items.insert(board.item(item).id(), item);
        let mut start_point: Option<Point> = None;
        let mut start_layer = 0;
        let mut end_point: Option<Point> = None;
        let mut end_layer = 0;
        for contact in contacts.iter() {
            let mut current_item = contact;
            let Some(mut prev_contact_point) = board.normal_contact_point(item, current_item) else {
                // no unique contact point
                continue;
            };
            let mut prev_contact_layer = board.item(item).first_common_layer(board.item(current_item), board);
            let mut fork_found = false;
            if board.item(item).is_trace() {
                // Check, that there is only 1 contact at this location.
                let check_contacts = board.trace_normal_contacts_at(item, &prev_contact_point, false);
                if check_contacts.len() != 1 {
                    fork_found = true;
                }
            }
            // Search from currentItem along the contacts until the next fork or nonroute item.
            loop {
                if !board.item(current_item).is_routable() || fork_found {
                    // connection ends
                    match &start_point {
                        None => {
                            start_point = Some(prev_contact_point.clone());
                            start_layer = prev_contact_layer;
                        }
                        Some(s) => {
                            if prev_contact_point != *s {
                                end_point = Some(prev_contact_point.clone());
                                end_layer = prev_contact_layer;
                            }
                        }
                    }
                    break;
                }
                // fastroute: around a closed ring of routable items without a fork the Java
                // loop never ends; stop where the walk has been before
                if !connection_items.insert(board.item(current_item).id(), current_item) {
                    break;
                }
                let current_item_contacts = board.normal_contacts(current_item);
                // filter the contacts at the previous contact point, because we were already
                // there. If then there is not exactly 1 new contact left, there is a stub or a
                // fork.
                let mut next_contact_point: Option<Point> = None;
                let mut next_contact_layer = -1;
                let mut next_contact: Option<ItemKey> = None;
                for tmp_contact in current_item_contacts.iter() {
                    let tmp_contact_layer = board.item(current_item).first_common_layer(board.item(tmp_contact), board);
                    if tmp_contact_layer >= 0 {
                        let Some(tmp_contact_point) = board.normal_contact_point(current_item, tmp_contact) else {
                            // no unique contact point
                            fork_found = true;
                            break;
                        };
                        if prev_contact_layer != tmp_contact_layer || prev_contact_point != tmp_contact_point {
                            next_contact_point = Some(tmp_contact_point);
                            next_contact_layer = tmp_contact_layer;
                            if next_contact.is_some() {
                                // second new contact found
                                fork_found = true;
                                break;
                            }
                            next_contact = Some(tmp_contact);
                        }
                    }
                }
                let Some(nc) = next_contact else {
                    break;
                };
                current_item = nc;
                prev_contact_point = next_contact_point.unwrap();
                prev_contact_layer = next_contact_layer;
            }
        }
        let result = Connection { start_point, start_layer, end_point, end_layer, item_list: connection_items };
        let index = eng.connections.len();
        let keys: Vec<ItemKey> = result.item_list.iter().collect();
        eng.connections.push(result);
        for k in keys {
            eng.item_info_mut(k).precalculated_connection = Some(index);
        }
        Some(index)
    }

    /// Java `traceLength()`.
    pub fn trace_length(&self, board: &BasicBoard) -> f64 {
        let mut result = 0.0;
        for key in self.item_list.iter() {
            if let Some(t) = board.item(key).as_trace() {
                result += t.length();
            }
        }
        result
    }

    /// Java `getDetour()`.
    pub fn get_detour(&self, board: &BasicBoard) -> f64 {
        let (Some(s), Some(e)) = (&self.start_point, &self.end_point) else {
            return i32::MAX as f64;
        };
        let min_trace_length = s.to_float().distance(&e.to_float());
        (self.trace_length(board) + DETOUR_ADD) / (min_trace_length + DETOUR_ADD) + DETOUR_ITEM_COST * (self.item_list.len() as f64 - 1.0)
    }
}
