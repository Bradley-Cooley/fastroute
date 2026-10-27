//! Port of `completeShape`, `restrainShape` and `divideLargeRoom` of `ShapeSearchTree`,
//! `ShapeSearchTree45Degree` and `ShapeSearchTree90Degree`.
//!
//! The base class collects all overlapping leaves, sorts them (`Leaf.compareTo`) and processes
//! them in that order. The 45 and 90 degree trees process the leaves while walking the tree
//! ([`TreeCursor`]) and shrink the query shape on the way, so their result depends on the tree
//! structure (which is why the tree reproduces Java's exactly).

use fr_geom::prelude::*;
use fr_geom::{IntBox, IntOctagon, Line, LineSegment, Side, Simplex, TileShape};

use crate::autoroute::rooms::IncompleteFreeSpaceExpansionRoom;
use crate::datastructures::TreeCursor;
use crate::ids::NetNo;

use super::basic_board::BasicBoard;
use super::search_tree::{layer_bit, sort_leaves_canonical, TreeKind, TreeObject};

impl BasicBoard {
    /// Java `ShapeSearchTree.completeShape(room, netNumber, ignoreObject, ignoreShape)` of tree
    /// `t` (dispatching on the tree class).
    pub fn complete_shape(
        &self,
        t: usize,
        room: &IncompleteFreeSpaceExpansionRoom,
        net_number: NetNo,
        ignore_object: Option<TreeObject>,
        ignore_shape: Option<&TileShape>,
    ) -> Vec<IncompleteFreeSpaceExpansionRoom> {
        if super::connectivity::verify_caches() {
            thread_local!(static CALLS: std::cell::Cell<u64> = const { std::cell::Cell::new(0) });
            if CALLS.with(|c| {
                c.set(c.get() + 1);
                c.get() % 256 == 1
            }) {
                self.verify_leaf_masks(t);
            }
        }
        match self.search_tree(t).kind {
            TreeKind::Default => self.complete_shape_default(t, room, net_number, ignore_object, ignore_shape),
            TreeKind::FortyfiveDegree => self.complete_shape_45(t, room, net_number, ignore_object, ignore_shape),
            TreeKind::NinetyDegree => self.complete_shape_90(t, room, net_number, ignore_object, ignore_shape),
        }
    }

    fn is_ignored(object: TreeObject, ignore_object: Option<TreeObject>) -> bool {
        match ignore_object {
            Some(o) => match (o, object) {
                (TreeObject::Item { key: a, .. }, TreeObject::Item { key: b, .. }) => a == b,
                (TreeObject::Room { key: a, .. }, TreeObject::Room { key: b, .. }) => a == b,
                _ => false,
            },
            None => false,
        }
    }

    // ------------------------------------------------------------------------------------------
    // ShapeSearchTree

    fn complete_shape_default(
        &self,
        t: usize,
        room: &IncompleteFreeSpaceExpansionRoom,
        net_number: NetNo,
        ignore_object: Option<TreeObject>,
        ignore_shape: Option<&TileShape>,
    ) -> Vec<IncompleteFreeSpaceExpansionRoom> {
        let Some(contained) = room.contained_shape.clone() else {
            log::warn!("ShapeSearchTree.completeShape: shapeToBeContained != null expected");
            return Vec::new();
        };
        let tree = self.search_tree(t);
        if tree.tree.is_empty() {
            return Vec::new();
        }
        let dirs = tree.bounding_directions();
        let mut start_shape = TileShape::IntBox(self.bounding_box);
        if let Some(s) = &room.shape {
            start_shape = start_shape.intersection(s);
        }
        let bounding_shape = start_shape.bounding_shape(&dirs).expect("completeShape: bounding shape");
        let mut result: Vec<IncompleteFreeSpaceExpansionRoom> = Vec::new();
        if start_shape.dimension() == 2 {
            result.push(IncompleteFreeSpaceExpansionRoom::new(Some(start_shape.clone()), room.layer, Some(contained)));
        }
        // collect in traversal order, then sort (Java sorts with Leaf.compareTo)
        let room_layer = room.layer;
        // Only leaves on the room layer are processed below: skip the others (leaf masks) and
        // sort canonically, unless the order would depend on the traversal order.
        let mut leaves = Vec::new();
        tree.tree.overlapping_leaves_masked(&bounding_shape, layer_bit(room_layer), &mut Vec::new(), &mut leaves);
        if !sort_leaves_canonical(&tree.tree, &mut leaves) {
            let mut cursor = TreeCursor::new();
            cursor.start(&tree.tree);
            leaves.clear();
            while let Some(leaf) = cursor.next_leaf(&tree.tree, &bounding_shape) {
                leaves.push(leaf);
            }
            leaves.sort_by(|a, b| tree.tree.compare_leaves(*a, *b, |x, y| x.java_cmp(y)));
        }
        for leaf in leaves {
            let l = tree.tree.leaf(leaf).unwrap();
            let object = l.object;
            let shape_index = l.shape_index_in_object;
            if self.object_is_trace_obstacle(object, net_number)
                && self.object_shape_layer(t, object, shape_index) == room_layer
                && !Self::is_ignored(object, ignore_object)
            {
                let object_shape = self.object_tree_shape(t, object, shape_index).expect("tree shape");
                let mut new_result = Vec::new();
                for current in result {
                    let mut something_changed = false;
                    let intersection = current.shape.as_ref().unwrap().intersection(&object_shape);
                    if intersection.dimension() == 2 {
                        let ignore_room = matches!(object, TreeObject::Room { .. })
                            && ignore_shape.map(|s| s.contains_tile_shape(&intersection)).unwrap_or(false);
                        if !ignore_room {
                            something_changed = true;
                            new_result.extend(Self::restrain_shape_default(&current, &object_shape));
                        }
                    }
                    if !something_changed {
                        new_result.push(current);
                    }
                }
                result = new_result;
                // (Java also updates a bounding shape here that is not used afterwards.)
            }
        }
        divide_large_room(TreeKind::Default, result, &self.bounding_box)
    }

    fn restrain_shape_default(incomplete_room: &IncompleteFreeSpaceExpansionRoom, obstacle_shape: &TileShape) -> Vec<IncompleteFreeSpaceExpansionRoom> {
        let mut result = Vec::new();
        // Always convert to Simplex (border lines of length 0 of octagons)
        let obstacle_simplex = TileShape::Simplex(obstacle_shape.to_simplex());
        let Some(contained) = &incomplete_room.contained_shape else {
            log::trace!("ShapeSearchTree.restrain_shape: shapeToBeContained is empty");
            return result;
        };
        let shape_to_be_contained = TileShape::Simplex(contained.to_simplex());
        if shape_to_be_contained.is_empty() {
            log::trace!("ShapeSearchTree.restrain_shape: shapeToBeContained is empty");
            return result;
        }
        let room_shape = incomplete_room.shape.as_ref().expect("restrain_shape: room shape");
        let layer = incomplete_room.layer;
        let mut cut_line: Option<Line> = None;
        let mut cut_line_distance = -1.0;
        let simplex_lines = obstacle_simplex.as_simplex().unwrap().lines().to_vec();
        let border_count = obstacle_simplex.border_line_count();
        for i in 0..border_count {
            let segment = simplex_segment(&obstacle_simplex, i);
            if room_shape.is_intersected_interior_by(&segment) {
                let current_line = simplex_lines[i as usize].clone();
                let current_min_distance = shape_to_be_contained.distance_to_the_left(&current_line);
                if current_min_distance > cut_line_distance {
                    cut_line_distance = current_min_distance;
                    cut_line = Some(current_line.opposite());
                }
            }
        }
        if let Some(cut) = cut_line {
            let result_piece = room_shape.intersection(&TileShape::get_instance_line(&cut));
            if result_piece.dimension() >= 2 {
                result.push(IncompleteFreeSpaceExpansionRoom::new(Some(result_piece), layer, Some(shape_to_be_contained)));
            }
            return result;
        }
        // There is no cut line, so that all shapeToBeContained is completely on the right side
        // of that line. Search a cut line, so that at least part of shapeToBeContained is on the
        // right side.
        if shape_to_be_contained.dimension() < 1 {
            // There is already a completed expansion room around shapeToBeContained.
            return result;
        }
        for i in 0..border_count {
            let segment = simplex_segment(&obstacle_simplex, i);
            if room_shape.is_intersected_interior_by(&segment) {
                let current_line = simplex_lines[i as usize].clone();
                if shape_to_be_contained.side_of(&current_line) == Side::Collinear {
                    cut_line = Some(current_line.opposite());
                    break;
                }
            }
        }
        let Some(cut) = cut_line else {
            return result;
        };
        let cut_half_plane = TileShape::get_instance_line(&cut);
        let new_shape_to_be_contained = shape_to_be_contained.intersection(&cut_half_plane);
        let result_piece = room_shape.intersection(&cut_half_plane);
        if result_piece.dimension() >= 2 {
            result.push(IncompleteFreeSpaceExpansionRoom::new(Some(result_piece), layer, Some(new_shape_to_be_contained)));
        }
        let opposite_half_plane = TileShape::get_instance_line(&cut.opposite());
        let rest_piece = room_shape.intersection(&opposite_half_plane);
        if rest_piece.dimension() >= 2 {
            let rest_contained = shape_to_be_contained.intersection(&opposite_half_plane);
            let rest_room = IncompleteFreeSpaceExpansionRoom::new(Some(rest_piece), layer, Some(rest_contained));
            result.extend(Self::restrain_shape_default(&rest_room, obstacle_shape));
        }
        result
    }

    // ------------------------------------------------------------------------------------------
    // ShapeSearchTree45Degree

    fn complete_shape_45(
        &self,
        t: usize,
        room: &IncompleteFreeSpaceExpansionRoom,
        net_number: NetNo,
        ignore_object: Option<TreeObject>,
        ignore_shape: Option<&TileShape>,
    ) -> Vec<IncompleteFreeSpaceExpansionRoom> {
        let Some(contained_raw) = &room.contained_shape else {
            log::warn!("ShapeSearchTree45Degree.completeShape: contained shape is null, skipping expansion room");
            return Vec::new();
        };
        let Some(shape_to_be_contained) = contained_raw.bounding_octagon() else {
            return Vec::new();
        };
        let tree = self.search_tree(t);
        if tree.tree.is_empty() {
            return Vec::new();
        }
        let mut start_shape = self.bounding_box.bounding_octagon();
        if let Some(s) = &room.shape {
            if !matches!(s, TileShape::IntOctagon(_)) {
                log::warn!("ShapeSearchTree45Degree.complete_shape: startShape of type IntOctagon expected");
                return Vec::new();
            }
            start_shape = s.bounding_octagon().unwrap().intersection_int_octagon(&start_shape);
        }
        let mut bounding_shape = start_shape;
        let room_layer = room.layer;
        // (room shape, shape to be contained); all shapes of this tree class are octagons
        let mut result: Vec<(IntOctagon, IntOctagon)> = vec![(start_shape, shape_to_be_contained)];
        let mut new_result: Vec<(IntOctagon, IntOctagon)> = Vec::new();
        let mut cursor = TreeCursor::new();
        cursor.start(&tree.tree);
        let layer_mask = layer_bit(room_layer);
        while let Some(leaf) = cursor.next_leaf_masked(&tree.tree, &fr_geom::RegularTileShape::IntOctagon(bounding_shape), layer_mask) {
            let l = tree.tree.leaf(leaf).unwrap();
            let object = l.object;
            let shape_index = l.shape_index_in_object;
            let is_obstacle = self.object_is_trace_obstacle(object, net_number);
            let same_layer = self.object_shape_layer(t, object, shape_index) == room_layer;
            if !(is_obstacle && same_layer && !Self::is_ignored(object, ignore_object)) {
                continue;
            }
            let object_shape = self
                .object_tree_shape(t, object, shape_index)
                .expect("tree shape")
                .bounding_octagon()
                .expect("bounding octagon");
            // fastroute: an obstacle that overlaps none of the rooms leaves the rooms and their
            // bounding shape (the union of the rooms) as they are; the walk only knows that it
            // overlaps the bounding shape
            if !result.iter().any(|(current_shape, _)| current_shape.overlaps(&object_shape)) {
                continue;
            }
            new_result.clear();
            let mut new_bounding_shape = IntOctagon::EMPTY;
            for &(current_shape, current_contained) in &result {
                let overlaps = current_shape.overlaps(&object_shape);
                if overlaps {
                    if matches!(object, TreeObject::Room { .. }) {
                        if let Some(ignore) = ignore_shape {
                            let intersection = current_shape.intersection_int_octagon(&object_shape);
                            if ignore.contains_tile_shape(&TileShape::IntOctagon(intersection)) {
                                // ignore also all objects, whose intersection is contained in the
                                // 2-dim overlap-door with the fromRoom.
                                if !ignore.contains_tile_shape(&TileShape::IntOctagon(current_shape)) {
                                    new_bounding_shape = new_bounding_shape.union_int_octagon(&current_shape);
                                    new_result.push((current_shape, current_contained));
                                }
                                continue;
                            }
                        }
                    }
                    let first_new = new_result.len();
                    restrain_shape_45(current_shape, current_contained, &object_shape, &mut new_result);
                    // Java unions all rooms of newResult here; the union (component-wise
                    // min/max) is idempotent and the earlier rooms are already contained, so
                    // adding the new rooms gives the same shape.
                    for tmp in &new_result[first_new..] {
                        new_bounding_shape = new_bounding_shape.union_int_octagon(&tmp.0);
                    }
                } else {
                    new_bounding_shape = new_bounding_shape.union_int_octagon(&current_shape);
                    new_result.push((current_shape, current_contained));
                }
            }
            std::mem::swap(&mut result, &mut new_result);
            bounding_shape = new_bounding_shape;
        }
        let result: Vec<IncompleteFreeSpaceExpansionRoom> = result
            .into_iter()
            .map(|(shape, contained)| IncompleteFreeSpaceExpansionRoom::new(Some(TileShape::IntOctagon(shape)), room_layer, Some(TileShape::IntOctagon(contained))))
            .collect();
        let mut result = divide_large_room(TreeKind::FortyfiveDegree, result, &self.bounding_box);
        // remove rooms with shapes equal to the contained shape to prevent endless loop.
        result.retain(|r| !r.contained_shape.as_ref().unwrap().contains_tile_shape(r.shape.as_ref().unwrap()));
        result
    }

    // ------------------------------------------------------------------------------------------
    // ShapeSearchTree90Degree

    fn complete_shape_90(
        &self,
        t: usize,
        room: &IncompleteFreeSpaceExpansionRoom,
        net_number: NetNo,
        ignore_object: Option<TreeObject>,
        ignore_shape: Option<&TileShape>,
    ) -> Vec<IncompleteFreeSpaceExpansionRoom> {
        let Some(TileShape::IntBox(shape_to_be_contained)) = room.contained_shape.as_ref() else {
            log::warn!("BoxShapeSearchTree.complete_shape: unexpected shapeToBeContained");
            return Vec::new();
        };
        let shape_to_be_contained = *shape_to_be_contained;
        let tree = self.search_tree(t);
        if tree.tree.is_empty() {
            return Vec::new();
        }
        let mut start_shape = self.bounding_box;
        if let Some(s) = &room.shape {
            let TileShape::IntBox(b) = s else {
                log::warn!("BoxShapeSearchTree.complete_shape: startShape of type IntBox expected");
                return Vec::new();
            };
            start_shape = b.intersection_int_box(&start_shape);
        }
        let mut bounding_shape = start_shape;
        let room_layer = room.layer;
        let mut result: Vec<IncompleteFreeSpaceExpansionRoom> = vec![IncompleteFreeSpaceExpansionRoom::new(
            Some(TileShape::IntBox(start_shape)),
            room_layer,
            Some(TileShape::IntBox(shape_to_be_contained)),
        )];
        let mut cursor = TreeCursor::new();
        cursor.start(&tree.tree);
        let layer_mask = layer_bit(room_layer);
        while let Some(leaf) = cursor.next_leaf_masked(&tree.tree, &fr_geom::RegularTileShape::IntBox(bounding_shape), layer_mask) {
            let l = tree.tree.leaf(leaf).unwrap();
            let object = l.object;
            let shape_index = l.shape_index_in_object;
            let is_obstacle = self.object_is_trace_obstacle(object, net_number);
            let same_layer = self.object_shape_layer(t, object, shape_index) == room_layer;
            if !(is_obstacle && same_layer && !Self::is_ignored(object, ignore_object)) {
                continue;
            }
            let object_shape = self.object_tree_shape(t, object, shape_index).expect("tree shape").bounding_box();
            let mut new_result: Vec<IncompleteFreeSpaceExpansionRoom> = Vec::new();
            let mut new_bounding_shape = IntBox::EMPTY;
            for current_room in result {
                let current_shape = *current_room.shape.as_ref().unwrap().as_int_box().expect("IntBox room shape");
                let overlaps = current_shape.overlaps(&object_shape);
                if overlaps {
                    if matches!(object, TreeObject::Room { .. }) {
                        if let Some(ignore) = ignore_shape {
                            let intersection = current_shape.intersection_int_box(&object_shape);
                            if ignore.contains_tile_shape(&TileShape::IntBox(intersection)) {
                                // ignore also all objects, whose intersection is contained in the
                                // 2-dim overlap-door with the fromRoom.
                                continue;
                            }
                        }
                    }
                    let restrained = restrain_shape_90(&current_room, &object_shape);
                    new_result.extend(restrained);
                    for tmp in &new_result {
                        new_bounding_shape = new_bounding_shape.union_int_box(&tmp.shape.as_ref().unwrap().bounding_box());
                    }
                } else {
                    new_bounding_shape = new_bounding_shape.union_int_box(&current_shape.bounding_box());
                    new_result.push(current_room);
                }
            }
            result = new_result;
            bounding_shape = new_bounding_shape;
        }
        result
    }
}

/// `LineSegment(simplex, i)`: the i-th border segment of a simplex.
fn simplex_segment(simplex: &TileShape, i: i32) -> LineSegment {
    let s: &Simplex = simplex.as_simplex().unwrap();
    let n = s.lines().len() as i32;
    let prev = s.lines()[((i - 1 + n) % n) as usize].clone();
    let cur = s.lines()[i as usize].clone();
    let next = s.lines()[((i + 1) % n) as usize].clone();
    LineSegment::new(prev, cur, next)
}

/// Java `ShapeSearchTree.divideLargeRoom` (+ the 45 degree override converting the shapes to
/// bounding octagons).
fn divide_large_room(kind: TreeKind, room_list: Vec<IncompleteFreeSpaceExpansionRoom>, board_box: &IntBox) -> Vec<IncompleteFreeSpaceExpansionRoom> {
    let mut result = divide_large_room_base(room_list, board_box);
    if kind == TreeKind::FortyfiveDegree {
        for room in result.iter_mut() {
            let shape = room.shape.as_ref().unwrap().bounding_octagon().expect("bounding octagon");
            room.shape = Some(TileShape::IntOctagon(shape));
            let contained = room.contained_shape.as_ref().unwrap().bounding_octagon().expect("bounding octagon");
            room.contained_shape = Some(TileShape::IntOctagon(contained));
        }
    }
    result
}

fn divide_large_room_base(room_list: Vec<IncompleteFreeSpaceExpansionRoom>, board_box: &IntBox) -> Vec<IncompleteFreeSpaceExpansionRoom> {
    if room_list.len() != 1 {
        return room_list;
    }
    let current_room = &room_list[0];
    let room_box = current_room.shape.as_ref().unwrap().bounding_box();
    if 2i32.wrapping_mul(room_box.height()) <= board_box.height() || 2i32.wrapping_mul(room_box.width()) <= board_box.width() {
        return room_list;
    }
    let max_section_width = 0.5 * board_box.height().max(board_box.width()) as f64;
    let sections = current_room.shape.as_ref().unwrap().divide_into_sections(max_section_width);
    let contained = current_room.contained_shape.as_ref().unwrap();
    sections
        .into_iter()
        .map(|section| {
            let section_contained = section.intersection(contained);
            IncompleteFreeSpaceExpansionRoom::new(Some(section), current_room.layer, Some(section_contained))
        })
        .collect()
}

/// Java `ShapeSearchTree45Degree.obstacleSegmentTouchesInside`.
fn obstacle_segment_touches_inside(obstacle_shape: &IntOctagon, obstacle_border_line_no: i32, room_shape: &IntOctagon) -> bool {
    let mut current_border_line_no = obstacle_border_line_no;
    let cx = obstacle_shape.corner_x(obstacle_border_line_no);
    let cy = obstacle_shape.corner_y(obstacle_border_line_no);
    for _ in 0..5 {
        if room_shape.side_of_border_line(cx, cy, current_border_line_no) != Side::OnTheLeft {
            return false;
        }
        current_border_line_no = (current_border_line_no + 1) % 8;
    }
    let next_no = (obstacle_border_line_no + 1) % 8;
    let nx = obstacle_shape.corner_x(next_no);
    let ny = obstacle_shape.corner_y(next_no);
    current_border_line_no = (obstacle_border_line_no + 5) % 8;
    for _ in 0..3 {
        if room_shape.side_of_border_line(nx, ny, current_border_line_no) != Side::OnTheLeft {
            return false;
        }
        current_border_line_no = (current_border_line_no + 1) % 8;
    }
    true
}

/// Java `ShapeSearchTree45Degree.signedLineDistance` (int subtraction wraps like Java).
fn signed_line_distance(o: &IntOctagon, line_no: i32, c: &IntOctagon) -> f64 {
    match line_no {
        0 => o.bottom_y.wrapping_sub(c.top_y) as f64,
        2 => c.left_x.wrapping_sub(o.right_x) as f64,
        4 => c.bottom_y.wrapping_sub(o.top_y) as f64,
        6 => o.left_x.wrapping_sub(c.right_x) as f64,
        // factor 0.5 used instead to 1 / sqrt(2) to prefer orthogonal lines slightly
        1 => 0.5 * c.upper_left_diagonal_x.wrapping_sub(o.lower_right_diagonal_x) as f64,
        3 => 0.5 * c.lower_left_diagonal_x.wrapping_sub(o.upper_right_diagonal_x) as f64,
        5 => 0.5 * o.upper_left_diagonal_x.wrapping_sub(c.lower_right_diagonal_x) as f64,
        7 => 0.5 * o.lower_left_diagonal_x.wrapping_sub(c.upper_right_diagonal_x) as f64,
        _ => {
            log::warn!("ShapeSearchTree45Degree.signed_line_distance: obstacleLineNo out of range");
            0.0
        }
    }
}

/// Java `ShapeSearchTree45Degree.calcOutsideRestrainedShape`.
fn calc_outside_restrained_shape(o: &IntOctagon, line_no: i32, r: &IntOctagon) -> IntOctagon {
    let (mut lx, mut ly, mut rx, mut uy) = (r.left_x, r.bottom_y, r.right_x, r.top_y);
    let (mut ulx, mut lrx, mut llx, mut urx) = (r.upper_left_diagonal_x, r.lower_right_diagonal_x, r.lower_left_diagonal_x, r.upper_right_diagonal_x);
    match line_no {
        0 => uy = o.bottom_y,
        2 => lx = o.right_x,
        4 => ly = o.top_y,
        6 => rx = o.left_x,
        1 => ulx = o.lower_right_diagonal_x,
        3 => llx = o.upper_right_diagonal_x,
        5 => lrx = o.upper_left_diagonal_x,
        7 => urx = o.lower_left_diagonal_x,
        _ => log::warn!("ShapeSearchTree45Degree.calc_outside_restrained_shape: obstacleLineNo out of range"),
    }
    IntOctagon::new(lx, ly, rx, uy, ulx, lrx, llx, urx).normalize()
}

/// Java `ShapeSearchTree45Degree.calcInsideRestrainedShape`.
fn calc_inside_restrained_shape(o: &IntOctagon, line_no: i32, r: &IntOctagon) -> IntOctagon {
    let (mut lx, mut ly, mut rx, mut uy) = (r.left_x, r.bottom_y, r.right_x, r.top_y);
    let (mut ulx, mut lrx, mut llx, mut urx) = (r.upper_left_diagonal_x, r.lower_right_diagonal_x, r.lower_left_diagonal_x, r.upper_right_diagonal_x);
    match line_no {
        0 => ly = o.bottom_y,
        2 => rx = o.right_x,
        4 => uy = o.top_y,
        6 => lx = o.left_x,
        1 => lrx = o.lower_right_diagonal_x,
        3 => urx = o.upper_right_diagonal_x,
        5 => ulx = o.upper_left_diagonal_x,
        7 => llx = o.lower_left_diagonal_x,
        _ => log::warn!("ShapeSearchTree45Degree.calc_inside_restrained_shape: obstacleLineNo out of range"),
    }
    IntOctagon::new(lx, ly, rx, uy, ulx, lrx, llx, urx).normalize()
}

/// Java `ShapeSearchTree90Degree.restrainShape`.
fn restrain_shape_90(incomplete_room: &IncompleteFreeSpaceExpansionRoom, obstacle: &IntBox) -> Vec<IncompleteFreeSpaceExpansionRoom> {
    let mut result = Vec::new();
    let Some(contained) = &incomplete_room.contained_shape else {
        return result;
    };
    if contained.is_empty() {
        log::trace!("BoxShapeSearchTree.restrain_shape: shapeToBeContained is empty");
        return result;
    }
    let room = incomplete_room.shape.as_ref().unwrap().bounding_box();
    let c = contained.bounding_box();
    let layer = incomplete_room.layer;
    let mut cut_line_distance = 0;
    let mut restrained: Option<IntBox> = None;
    if room.ll.x < obstacle.ur.x && room.ur.x > obstacle.ur.x && room.ur.y > obstacle.ll.y && room.ll.y < obstacle.ur.y {
        // The right line segment of the obstacle intersects the interior of shape
        let d = c.ll.x.wrapping_sub(obstacle.ur.x);
        if d > cut_line_distance {
            cut_line_distance = d;
            restrained = Some(IntBox::new(obstacle.ur.x, room.ll.y, room.ur.x, room.ur.y));
        }
    }
    if room.ll.x < obstacle.ll.x && room.ur.x > obstacle.ll.x && room.ur.y > obstacle.ll.y && room.ll.y < obstacle.ur.y {
        // The left line segment
        let d = obstacle.ll.x.wrapping_sub(c.ur.x);
        if d > cut_line_distance {
            cut_line_distance = d;
            restrained = Some(IntBox::new(room.ll.x, room.ll.y, obstacle.ll.x, room.ur.y));
        }
    }
    if room.ll.y < obstacle.ll.y && room.ur.y > obstacle.ll.y && room.ur.x > obstacle.ll.x && room.ll.x < obstacle.ur.x {
        // The lower line segment
        let d = obstacle.ll.y.wrapping_sub(c.ur.y);
        if d > cut_line_distance {
            cut_line_distance = d;
            restrained = Some(IntBox::new(room.ll.x, room.ll.y, room.ur.x, obstacle.ll.y));
        }
    }
    if room.ll.y < obstacle.ur.y && room.ur.y > obstacle.ur.y && room.ur.x > obstacle.ll.x && room.ll.x < obstacle.ur.x {
        // The upper line segment
        let d = c.ll.y.wrapping_sub(obstacle.ur.y);
        if d > cut_line_distance {
            restrained = Some(IntBox::new(room.ll.x, obstacle.ur.y, room.ur.x, room.ur.y));
        }
    }
    if let Some(r) = restrained {
        result.push(IncompleteFreeSpaceExpansionRoom::new(Some(TileShape::IntBox(r)), layer, Some(TileShape::IntBox(c))));
        return result;
    }
    // Now shapeToBeContained intersects with the obstacle; shapeToBeContained and shape evtl.
    // need to be divided in two.
    let is = c.intersection_int_box(obstacle);
    if is.is_empty() {
        log::warn!("BoxShapeSearchTree.restrain_shape: Intersection between obstacleShape and shapeToBeContained expected");
        return result;
    }
    let mut new_shapes: Option<(IntBox, IntBox)> = None;
    if is.ll.x > room.ll.x && is.ll.x == obstacle.ll.x && is.ll.x < room.ur.x {
        new_shapes = Some((IntBox::new(room.ll.x, room.ll.y, is.ll.x, room.ur.y), IntBox::new(is.ll.x, room.ll.y, room.ur.x, room.ur.y)));
    } else if is.ur.x > room.ll.x && is.ur.x == obstacle.ur.x && is.ur.x < room.ur.x {
        new_shapes = Some((IntBox::new(is.ur.x, room.ll.y, room.ur.x, room.ur.y), IntBox::new(room.ll.x, room.ll.y, is.ur.x, room.ur.y)));
    } else if is.ll.y > room.ll.y && is.ll.y == obstacle.ll.y && is.ll.y < room.ur.y {
        new_shapes = Some((IntBox::new(room.ll.x, room.ll.y, room.ur.x, is.ll.y), IntBox::new(room.ll.x, is.ll.y, room.ur.x, room.ur.y)));
    } else if is.ur.y > room.ll.y && is.ur.y == obstacle.ur.y && is.ur.y < room.ur.y {
        new_shapes = Some((IntBox::new(room.ll.x, is.ur.y, room.ur.x, room.ur.y), IntBox::new(room.ll.x, room.ll.y, room.ur.x, is.ur.y)));
    }
    if let Some((new_shape1, new_shape2)) = new_shapes {
        let new_contained = c.intersection_int_box(&new_shape1);
        if new_contained.dimension() > 0 {
            result.push(IncompleteFreeSpaceExpansionRoom::new(Some(TileShape::IntBox(new_shape1)), layer, Some(TileShape::IntBox(new_contained))));
            let new_room = IncompleteFreeSpaceExpansionRoom::new(
                Some(TileShape::IntBox(new_shape2)),
                layer,
                Some(TileShape::IntBox(c.intersection_int_box(&new_shape2))),
            );
            result.extend(restrain_shape_90(&new_room, obstacle));
        }
    }
    result
}

/// Java `ShapeSearchTree45Degree.restrainShape(room, obstacleShape)` for a room with octagon
/// shape `room_shape` and octagon shape to be contained: appends the restrained rooms
/// (shape, shape to be contained) to `out`, in the order of the Java result list.
fn restrain_shape_45(room_shape: IntOctagon, shape_to_be_contained: IntOctagon, obstacle_shape: &IntOctagon, out: &mut Vec<(IntOctagon, IntOctagon)>) {
    if shape_to_be_contained.is_empty() {
        log::debug!("ShapeSearchTree45Degree.restrain_shape: shapeToBeContained is empty");
        return;
    }
    let mut cut_line_distance = -1.0;
    let mut restraining_line_no = -1;
    for obstacle_line_no in 0..8 {
        let current_distance = signed_line_distance(obstacle_shape, obstacle_line_no, &shape_to_be_contained);
        if current_distance > cut_line_distance && obstacle_segment_touches_inside(obstacle_shape, obstacle_line_no, &room_shape) {
            cut_line_distance = current_distance;
            restraining_line_no = obstacle_line_no;
        }
    }
    if cut_line_distance >= 0.0 {
        let restrained = calc_outside_restrained_shape(obstacle_shape, restraining_line_no, &room_shape);
        out.push((restrained, shape_to_be_contained));
        return;
    }
    // There is no cut line, so that all shapeToBeContained is completely on the right side
    // of that line. Search a cut line, so that at least part of shapeToBeContained is on the
    // right side.
    if shape_to_be_contained.dimension() < 1 {
        // There is already a completed expansion room around shapeToBeContained.
        return;
    }
    restraining_line_no = -1;
    for obstacle_line_no in 0..8 {
        if obstacle_segment_touches_inside(obstacle_shape, obstacle_line_no, &room_shape) {
            let current_line = obstacle_shape.border_line(obstacle_line_no);
            if shape_to_be_contained.side_of(&current_line) == Side::Collinear {
                // current_line intersects with the interior of shapeToBeContained
                restraining_line_no = obstacle_line_no;
                break;
            }
        }
    }
    if restraining_line_no < 0 {
        // cut line not found, parts or the whole of shape may be already occupied from
        // somewhere else.
        return;
    }
    let restrained = calc_outside_restrained_shape(obstacle_shape, restraining_line_no, &room_shape);
    if restrained.dimension() == 2 {
        let new_contained = shape_to_be_contained.intersection_int_octagon(&restrained);
        if new_contained.dimension() > 0 {
            out.push((restrained, new_contained));
        }
    }
    let rest_piece = calc_inside_restrained_shape(obstacle_shape, restraining_line_no, &room_shape);
    if rest_piece.dimension() >= 2 {
        // Java: shapeToBeContained.intersection(restPiece) = restPiece.intersection(shapeToBeContained)
        let rest_contained = rest_piece.intersection_int_octagon(&shape_to_be_contained);
        if rest_contained.dimension() >= 0 {
            restrain_shape_45(rest_piece, rest_contained, obstacle_shape, out);
        }
    }
}
