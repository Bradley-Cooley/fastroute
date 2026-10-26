//! Port of `autoroute/maze/{MazeSearchEngine, MazeExpansionEngine, MazeRipupResolver,
//! MazeTraceShover, MazeListElement}.java`: the maze search between the start and destination
//! items of a connection.

use std::cmp::Ordering;

use fr_geom::{FloatLine, FloatPoint, IntPoint, Line, LineSegment, Point, Polyline, Side, TileShape};
use fr_jcompat::{JavaRandom, JavaTreeSet};

use crate::board::actions::forced_pad_router::CheckDrillResult;
use crate::board::actions::forced_via_inserter;
use crate::board::optimize::trace_shover::TraceShover;
use crate::board::{ItemKey, ItemSelectionFilter, ItemSet, RoutingBoard, SelectableChoices};
use crate::ids::{AngleRestriction, FixedState, LayerNo, NetNo};
use crate::structure::Unit;

use super::connection::Connection;
use super::control::{jmax, jmin, AutorouteControl};
use super::destination::DestinationDistance;
use super::engine::{Adjustment, AutorouteEngine, DoorId, DrillId, Expandable, IncompleteRef, PageId, RoomId, RoomKind};
use super::TRACE_WIDTH_TOLERANCE;

/// fastroute: maze search steps (expanded queue elements) of all searches in the process. The
/// count only depends on what is routed, not on the machine or its load, so it measures the
/// work of a routing run where wall-clock time would not (see `pipeline::run_pipeline`).
pub static SEARCH_STEPS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// Java `MazeSearchEngine.ALREADY_RIPPED_COSTS`.
pub const ALREADY_RIPPED_COSTS: i32 = 1;

const FANOUT_COST_CONSTANT: f64 = 20000.0;

/// Java `MazeListElement`: an element of the maze expansion queue.
#[derive(Clone, Debug)]
pub struct MazeListElement {
    /// The door or drill belonging to this element.
    pub door: Expandable,
    /// The section number of the door (or the layer index of the drill).
    pub section_no_of_door: i32,
    /// The door, from which this door was expanded.
    pub backtrack_door: Option<Expandable>,
    pub section_no_of_backtrack_door: i32,
    /// The weighted distance to the start of the expansion.
    pub expansion_value: f64,
    /// The expansion value plus the shortest distance to a destination (queue order).
    pub sorting_value: f64,
    /// The next room, which will be expanded from this element.
    pub next_room: Option<RoomId>,
    pub shape_entry: FloatLine,
    pub room_ripped: bool,
    pub adjustment: Adjustment,
    pub already_checked: bool,
    /// The ripup cost paid to enter `next_room` through this door.
    pub ripup_cost: i32,
}

impl MazeListElement {
    #[allow(clippy::too_many_arguments)]
    fn new(
        door: Expandable,
        section_no_of_door: i32,
        backtrack_door: Option<Expandable>,
        section_no_of_backtrack_door: i32,
        expansion_value: f64,
        sorting_value: f64,
        next_room: Option<RoomId>,
        shape_entry: FloatLine,
        room_ripped: bool,
        adjustment: Adjustment,
        already_checked: bool,
    ) -> Self {
        MazeListElement {
            door,
            section_no_of_door,
            backtrack_door,
            section_no_of_backtrack_door,
            expansion_value,
            sorting_value,
            next_room,
            shape_entry,
            room_ripped,
            adjustment,
            already_checked,
            ripup_cost: 0,
        }
    }
}

/// Java `MazeListElement.compareTo` (the door ids are evaluated live).
fn compare_elements(a: &MazeListElement, b: &MazeListElement, eng: &AutorouteEngine) -> Ordering {
    if a.sorting_value < b.sorting_value {
        return Ordering::Less;
    }
    if a.sorting_value > b.sorting_value {
        return Ordering::Greater;
    }
    if a.expansion_value < b.expansion_value {
        return Ordering::Less;
    }
    if a.expansion_value > b.expansion_value {
        return Ordering::Greater;
    }
    let id1 = eng.expandable_java_id(a.door);
    let id2 = eng.expandable_java_id(b.door);
    if id1 < id2 {
        return Ordering::Less;
    }
    if id1 > id2 {
        return Ordering::Greater;
    }
    a.section_no_of_door.cmp(&b.section_no_of_door)
}

/// Java `MazeSearchEngine.Result`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MazeSearchResult {
    pub destination_door: Expandable,
    pub section_no_of_door: i32,
}

/// Java `MazeTraceShover.DoorSection`.
struct DoorSection {
    door: DoorId,
    section_index: i32,
    section_line: FloatLine,
}

/// Java `MazeSearchEngine` (with `MazeExpansionEngine`, `MazeRipupResolver` and
/// `MazeTraceShover`).
pub struct MazeSearchEngine<'a> {
    pub eng: &'a mut AutorouteEngine,
    pub board: &'a mut RoutingBoard,
    pub ctrl: &'a AutorouteControl,
    /// The elements (the queue holds indices). fastroute: the slot of an element taken from the
    /// queue is reused (its data is copied to its door section when it is taken), so the list
    /// only grows to the largest queue size; a search over the whole board added millions.
    pub elements: Vec<MazeListElement>,
    free_elements: Vec<usize>,
    /// Java `mazeExpansionList` (a `TreeSet`).
    pub queue: JavaTreeSet<usize>,
    /// If set, the indices of the elements added to the queue are recorded (debugging aid).
    pub trace_added: Option<Vec<usize>>,
    pub destination_distance: DestinationDistance,
    random_generator: JavaRandom,
    destination_door: Option<Expandable>,
    section_no_of_destination_door: i32,
}

impl<'a> MazeSearchEngine<'a> {
    fn new(eng: &'a mut AutorouteEngine, board: &'a mut RoutingBoard, ctrl: &'a AutorouteControl) -> Self {
        let mut random_generator = JavaRandom::new(0);
        // Keep v1.9 deterministic randomization across passes.
        random_generator.set_seed(ctrl.ripup_costs as i64);
        let destination_distance = DestinationDistance::new(&ctrl.trace_costs, &ctrl.layer_active, ctrl.min_normal_via_cost, ctrl.min_cheap_via_cost);
        MazeSearchEngine {
            eng,
            board,
            ctrl,
            elements: Vec::new(),
            free_elements: Vec::new(),
            queue: JavaTreeSet::new(fr_jcompat::treemap::NaturalOrder),
            trace_added: None,
            destination_distance,
            random_generator,
            destination_door: None,
            section_no_of_destination_door: 0,
        }
    }

    /// Java `MazeSearchEngine.getInstance(startItems, destinationItems, engine, ctrl)`: `None` if
    /// the initialisation failed.
    pub fn get_instance(
        start_items: &ItemSet,
        destination_items: &ItemSet,
        eng: &'a mut AutorouteEngine,
        board: &'a mut RoutingBoard,
        ctrl: &'a AutorouteControl,
    ) -> Option<Self> {
        let mut new_instance = MazeSearchEngine::new(eng, board, ctrl);
        if new_instance.init(start_items, destination_items) {
            Some(new_instance)
        } else {
            None
        }
    }

    /// Java `mazeExpansionList.add(element)` (with the fanout length limits of the anonymous
    /// subclass).
    fn add_element(&mut self, element: MazeListElement) {
        let ctrl = self.ctrl;
        if ctrl.is_fanout {
            if let Some(center) = &ctrl.fanout_start_pin_center {
                let pin_center_float = center.to_float();
                let on_start_layer = element.next_room.map(|r| self.eng.room(r).layer == ctrl.fanout_start_pin_layer).unwrap_or(false);
                if on_start_layer {
                    let max_len = ctrl.fanout_max_escape_length_mm.map(|v| v * 1000.0).unwrap_or(3000.0);
                    let resolution = self.board.communication.get_resolution(Unit::Um);
                    let entry_point = element.shape_entry.a.middle_point(&element.shape_entry.b);
                    let dist = entry_point.distance(&pin_center_float);
                    if dist > max_len * resolution {
                        return;
                    }
                }
                if let Expandable::Drill(d) = element.door {
                    let min_len = ctrl.fanout_min_escape_length_mm.map(|v| v * 1000.0).unwrap_or(500.0);
                    let resolution = self.board.communication.get_resolution(Unit::Um);
                    let drill_dist = self.eng.drill(d).location.to_float().distance(&pin_center_float);
                    if drill_dist < min_len * resolution {
                        return;
                    }
                }
            }
        }
        let index = match self.free_elements.pop() {
            Some(i) => {
                self.elements[i] = element;
                i
            }
            None => {
                self.elements.push(element);
                self.elements.len() - 1
            }
        };
        let elements = &self.elements;
        let eng: &AutorouteEngine = self.eng;
        let added = self.queue.add_by(index, |a, b| compare_elements(&elements[*a], &elements[*b], eng));
        if added {
            if let Some(t) = &mut self.trace_added {
                t.push(index);
            }
        } else if self.trace_added.is_none() {
            // an equal element is queued already: the slot is free again
            self.free_elements.push(index);
        }
    }

    /// The number of elements in the queue.
    pub fn queue_len(&self) -> usize {
        self.queue.len()
    }

    /// The first element of the queue.
    pub fn queue_first(&self) -> Option<&MazeListElement> {
        self.queue.first().map(|i| &self.elements[*i])
    }

    /// Java `findConnection()`: the destination door and its section of the found connection.
    pub fn find_connection(&mut self) -> Option<MazeSearchResult> {
        let mut steps = 0u64;
        while self.occupy_next_element() {
            steps += 1;
        }
        SEARCH_STEPS.fetch_add(steps, std::sync::atomic::Ordering::Relaxed);
        self.destination_door.map(|d| MazeSearchResult { destination_door: d, section_no_of_door: self.section_no_of_destination_door })
    }

    /// Java `occupyNextElement()`: expands the next element of the queue. Returns false if the
    /// queue is exhausted or the destination is reached.
    pub fn occupy_next_element(&mut self) -> bool {
        if self.destination_door.is_some() {
            return false; // destination already reached
        }
        let mut list_element: Option<MazeListElement> = None;
        // Search the next element, which is not yet expanded.
        while !self.queue.is_empty() {
            if self.eng.is_stop_requested() {
                return false;
            }
            let index = self.queue.poll_first().unwrap();
            let e = self.elements[index].clone();
            // (kept while tracing: the trace reads the elements after the search)
            if self.trace_added.is_none() {
                self.free_elements.push(index);
            }
            if !self.eng.element(e.door, e.section_no_of_door).is_occupied {
                list_element = Some(e);
                break;
            }
        }
        let Some(list_element) = list_element else {
            return false;
        };
        {
            let section = self.eng.element_mut(list_element.door, list_element.section_no_of_door);
            section.backtrack_door = list_element.backtrack_door;
            section.section_no_of_backtrack_door = list_element.section_no_of_backtrack_door;
            section.room_ripped = list_element.room_ripped;
            section.ripup_cost = list_element.ripup_cost;
            section.adjustment = list_element.adjustment;
        }
        if let Expandable::Page(_) = list_element.door {
            self.expand_to_drills_of_page(&list_element);
            return true;
        }
        if let Expandable::Target(t) = list_element.door {
            if self.eng.is_destination_door(t) {
                // The destination is reached.
                self.destination_door = Some(list_element.door);
                self.section_no_of_destination_door = list_element.section_no_of_door;
                return false;
            }
        }
        let door_is_drill = matches!(list_element.door, Expandable::Drill(_));
        let backtrack_is_drill = matches!(list_element.backtrack_door, Some(Expandable::Drill(_)));
        if self.ctrl.is_fanout && door_is_drill && backtrack_is_drill {
            // algorithm completed after the first drill;
            self.destination_door = Some(list_element.door);
            self.section_no_of_destination_door = list_element.section_no_of_door;
            return false;
        }
        if self.ctrl.vias_allowed && door_is_drill && !backtrack_is_drill {
            self.expand_to_other_layers(&list_element);
        }
        if list_element.next_room.is_some() && !self.expand_to_room_doors(&list_element) {
            // occupation by ripup is delayed or nothing was expanded
            return true;
        }
        self.eng.element_mut(list_element.door, list_element.section_no_of_door).is_occupied = true;
        true
    }

    fn room_layer(&self, r: RoomId) -> LayerNo {
        self.eng.room(r).layer
    }

    /// Java `expandToRoomDoors(listElement)`: returns true if the from door section has to be
    /// occupied, false if the occupation is delayed.
    fn expand_to_room_doors(&mut self, list_element: &MazeListElement) -> bool {
        let ctrl = self.ctrl;
        let next_room = list_element.next_room.unwrap();
        let layer_index = self.room_layer(next_room);
        let li = layer_index as usize;
        let layer_active = ctrl.layer_active[li];
        if !layer_active && self.board.layer_structure.layers[li].is_signal {
            return true;
        }
        let mut half_width = ctrl.compensated_trace_half_width[li] as f64;
        let mut current_door_is_small = false;
        if let Expandable::Door(current_door) = list_element.door {
            let mut half_width_add = half_width + TRACE_WIDTH_TOLERANCE as f64;
            if ctrl.with_neckdown {
                // try evtl. neckdown at a destination pin
                let neck_down_half_width = self.check_neck_down_at_dest_pin(next_room);
                if neck_down_half_width > 0.0 {
                    half_width_add = jmin(half_width_add, neck_down_half_width);
                    half_width = half_width_add;
                }
            }
            current_door_is_small = self.door_is_small(current_door, 2.0 * half_width_add);
        }
        self.eng.complete_neighbour_rooms(self.board, next_room);
        let shape_entry_middle = list_element.shape_entry.a.middle_point(&list_element.shape_entry.b);
        if ctrl.with_neckdown {
            if let Expandable::Target(t) = list_element.door {
                // try evtl. neckdown at a start pin
                let start_item = self.eng.target_door(t).item;
                if self.board.item(start_item).is_pin() {
                    let neckdown_half_width = ctrl
                        .clamp_neckdown_half_width(self.board.pin_trace_neckdown_half_width(start_item, layer_index))
                        as f64;
                    if neckdown_half_width > 0.0 {
                        half_width = jmin(half_width, neckdown_half_width);
                    }
                }
            }
        }
        let next_room_is_thick = if self.eng.room(next_room).is_obstacle() {
            self.room_shape_is_thick(next_room)
        } else {
            let next_room_shape = self.eng.room(next_room).shape();
            if next_room_shape.min_width() < 2.0 * half_width {
                false // to prevent problems with the opposite side
            } else if !list_element.already_checked && self.eng.expandable_dimension(list_element.door) == 1 && !current_door_is_small {
                // The algorithm below works only, if location is on the border of roomShape.
                let nearest_points = next_room_shape.nearest_border_points_approx(&shape_entry_middle, 2);
                if nearest_points.len() < 2 {
                    log::warn!("MazeSearchEngine.expand_to_room_doors: nearestPoints.length == 2 expected");
                    false
                } else {
                    let current_distance = nearest_points[1].expect("nearest border point").distance(&shape_entry_middle);
                    current_distance > half_width + 1.0
                }
            } else {
                true
            }
        };
        if !layer_active {
            if let Expandable::Drill(d) = list_element.door {
                // check for drill to a foreign conduction area on split plane.
                let drill_location = self.eng.drill(d).location.clone();
                let filter = ItemSelectionFilter::single(SelectableChoices::Conduction);
                let picked_items = self.board.pick_items(&drill_location, layer_index, Some(&filter));
                for key in picked_items.iter() {
                    if !self.board.item(key).contains_net(ctrl.net_number) {
                        return true;
                    }
                }
            }
        }
        let mut something_expanded = self.expand_to_target_doors(list_element, next_room_is_thick, current_door_is_small, &shape_entry_middle);
        if !layer_active {
            return true;
        }
        let mut ripup_costs = 0;
        match self.eng.room(next_room).kind.clone() {
            RoomKind::Incomplete { .. } | RoomKind::Complete { .. } => {
                if !list_element.already_checked && current_door_is_small {
                    let mut enter_through_small_door = false;
                    if next_room_is_thick {
                        // check to enter the thick room from a ripped item through a small door
                        enter_through_small_door = self.check_leaving_ripped_item(list_element);
                    }
                    if !enter_through_small_door {
                        return something_expanded;
                    }
                }
            }
            RoomKind::Obstacle { item: obstacle_item, .. } => {
                if !list_element.already_checked {
                    let mut room_rippable = false;
                    if ctrl.ripup_allowed {
                        ripup_costs = self.check_ripup(list_element, obstacle_item, current_door_is_small);
                        room_rippable = ripup_costs >= 0;
                    }
                    if ripup_costs != ALREADY_RIPPED_COSTS
                        && next_room_is_thick
                        && !current_door_is_small
                        && ctrl.max_shove_trace_recursion_depth > 0
                        && self.board.item(obstacle_item).is_trace()
                    {
                        let shoved = self.shove_trace_room(list_element, next_room);
                        if !shoved {
                            if ripup_costs > 0 {
                                // delay the occupation by ripup to allow shoving the room by
                                // another door sections.
                                let mut new_element = MazeListElement::new(
                                    list_element.door,
                                    list_element.section_no_of_door,
                                    list_element.backtrack_door,
                                    list_element.section_no_of_backtrack_door,
                                    list_element.expansion_value + ripup_costs as f64,
                                    list_element.sorting_value + ripup_costs as f64,
                                    list_element.next_room,
                                    list_element.shape_entry,
                                    true,
                                    list_element.adjustment,
                                    true,
                                );
                                new_element.ripup_cost = ripup_costs;
                                self.add_element(new_element);
                            }
                            return something_expanded;
                        }
                    }
                    if !room_rippable {
                        return true;
                    }
                }
            }
        }
        let room_doors_snapshot: Vec<DoorId> = self.eng.room(next_room).doors.clone();
        for to_door in room_doors_snapshot {
            if Expandable::Door(to_door) == list_element.door {
                continue;
            }
            if self.expand_to_door(to_door, list_element, ripup_costs, next_room_is_thick, Adjustment::None) {
                something_expanded = true;
            }
        }
        // Expand also the drill pages intersecting the room.
        if ctrl.vias_allowed && !matches!(list_element.door, Expandable::Drill(_)) {
            let next_is_free = self.eng.room(next_room).is_complete_free();
            if (something_expanded || next_room_is_thick) && next_is_free {
                // avoid setting somethingExpanded to true when nextRoom is thin to allow
                // occupying by different sections of the door
                let shape = self.eng.room(next_room).shape().clone();
                let overlapping_drill_pages = self.eng.overlapping_drill_pages(&shape);
                for to_drill_page in overlapping_drill_pages {
                    self.expand_to_drill_page(to_drill_page, list_element);
                    something_expanded = true;
                }
            } else if let Some(item) = self.eng.room(next_room).obstacle_item() {
                if self.board.item(item).is_via() {
                    let via_drill_info = self.eng.via_drill_info(self.board, item);
                    self.expand_to_drill(via_drill_info, list_element, ripup_costs);
                }
            }
        }
        something_expanded
    }

    /// Java `expandToTargetDoors(...)`: returns true if at least one target door was expanded.
    fn expand_to_target_doors(
        &mut self,
        list_element: &MazeListElement,
        next_room_is_thick: bool,
        current_door_is_small: bool,
        shape_entry_middle: &FloatPoint,
    ) -> bool {
        let ctrl = self.ctrl;
        let next_room = list_element.next_room.unwrap();
        if current_door_is_small {
            let mut enter_through_small_door = false;
            if let Expandable::Door(_) = list_element.door {
                if let Some(from_room) = self.eng.expandable_other_room(list_element.door, Some(next_room)) {
                    if self.eng.room(from_room).is_obstacle() {
                        // otherwise entering through the small door may fail, because it was not
                        // checked.
                        enter_through_small_door = true;
                    }
                }
            }
            if !enter_through_small_door {
                return false;
            }
        }
        let mut result = false;
        let target_doors: Vec<_> = self.eng.room(next_room).target_doors().to_vec();
        for to_door in target_doors {
            if Expandable::Target(to_door) == list_element.door {
                continue;
            }
            let (item, tree_entry_no) = {
                let td = self.eng.target_door(to_door);
                (td.item, td.tree_entry_no)
            };
            let tree_shape_count = self.board.tree_shape_count(self.eng.tree, item);
            if tree_entry_no < 0 || tree_entry_no >= tree_shape_count {
                continue;
            }
            let Some(target_shape) = self.board.trace_connection_shape(self.eng.tree, item, tree_entry_no) else {
                continue;
            };
            let connection_point = target_shape.nearest_point_approx(shape_entry_middle).expect("nearest point");
            if !next_room_is_thick {
                // check the line from shapeEntryMiddle to the nearest point.
                let current_net_numbers = [ctrl.net_number];
                let current_layer = self.room_layer(next_room);
                let p0 = shape_entry_middle.round();
                let p1 = connection_point.round();
                if p0 != p1 {
                    let check_polyline = Polyline::from_int_points(&[p0, p1]);
                    let check_ok = self.board.check_forced_trace_polyline(
                        &check_polyline,
                        ctrl.trace_half_width[current_layer as usize],
                        current_layer,
                        &current_net_numbers,
                        ctrl.trace_clearance_class,
                        ctrl.max_shove_trace_recursion_depth,
                        ctrl.max_shove_via_recursion_depth,
                        ctrl.max_spring_over_recursion_depth,
                    );
                    if !check_ok {
                        continue;
                    }
                }
            }
            let new_shape_entry = FloatLine::new(connection_point, connection_point);
            if self.expand_to_door_section(Expandable::Target(to_door), 0, Some(new_shape_entry), list_element, 0, Adjustment::None) {
                result = true;
            }
        }
        result
    }

    /// Java `expandToDoor(toDoor, listElement, addCosts, nextRoomIsThick, adjustment)`.
    fn expand_to_door(&mut self, to_door: DoorId, list_element: &MazeListElement, add_costs: i32, next_room_is_thick: bool, adjustment: Adjustment) -> bool {
        let next_room = list_element.next_room.unwrap();
        let half_width = self.ctrl.compensated_trace_half_width[self.room_layer(next_room) as usize] as f64;
        let mut something_expanded = false;
        let line_sections = self.eng.door_section_segments(to_door, half_width);
        for i in 0..line_sections.len() {
            if self.eng.element(Expandable::Door(to_door), i as i32).is_occupied {
                continue;
            }
            let new_shape_entry;
            if next_room_is_thick {
                new_shape_entry = line_sections[i];
                let door = self.eng.door(to_door);
                if door.dimension == 1
                    && line_sections.len() == 1
                    && self.eng.room(door.first_room).is_complete_free()
                    && self.eng.room(door.second_room).is_complete_free()
                {
                    // check entering the toDoor at an acute corner of the shape of
                    // listElement.nextRoom
                    let shape_entry_middle = new_shape_entry.a.middle_point(&new_shape_entry.b);
                    let room_shape = self.eng.room(next_room).shape();
                    if room_shape.min_width() < 2.0 * half_width {
                        return false;
                    }
                    let nearest_points = room_shape.nearest_border_points_approx(&shape_entry_middle, 2);
                    if nearest_points.len() < 2 || nearest_points[1].expect("nearest border point").distance(&shape_entry_middle) <= half_width + 1.0 {
                        return false;
                    }
                }
            } else {
                // expand only doors on the opposite side of the room from the shapeEntry.
                if self.eng.door(to_door).dimension == 1 && i == 0 && line_sections[0].b.distance_square(&line_sections[0].a) < 1.0 {
                    // toDoor is small belonging to a via or thin room
                    continue;
                }
                match segment_projection(&list_element.shape_entry, &line_sections[i]) {
                    Some(e) => new_shape_entry = e,
                    None => continue,
                }
            }
            if self.expand_to_door_section(Expandable::Door(to_door), i as i32, Some(new_shape_entry), list_element, add_costs, adjustment) {
                something_expanded = true;
            }
        }
        something_expanded
    }

    /// Java `doorIsSmall(door, traceWidth)`.
    fn door_is_small(&self, door: DoorId, trace_width: f64) -> bool {
        let d = self.eng.door(door);
        if d.dimension == 1 || self.eng.room(d.first_room).is_complete_free() && self.eng.room(d.second_room).is_complete_free() {
            let door_shape = self.eng.door_shape(door);
            if door_shape.is_empty() {
                log::trace!("MazeSearchEngine:check_door_width doorShape is empty");
                return true;
            }
            let door_length = match self.board.rules.get_trace_angle_restriction() {
                AngleRestriction::NinetyDegree => door_shape.bounding_box().max_width(),
                AngleRestriction::FortyfiveDegree => door_shape.bounding_octagon().expect("bounding octagon").max_width(),
                AngleRestriction::None => {
                    let s = door_shape.diagonal_corner_segment().expect("diagonal corner segment");
                    s.b.distance(&s.a)
                }
            };
            return door_length < trace_width;
        }
        false
    }

    /// Java `expandToDoorSection(door, sectionIndex, shapeEntry, fromElement, addCosts,
    /// adjustment)`: returns true if the door section was expanded.
    fn expand_to_door_section(
        &mut self,
        door: Expandable,
        section_index: i32,
        shape_entry: Option<FloatLine>,
        from_element: &MazeListElement,
        add_costs: i32,
        adjustment: Adjustment,
    ) -> bool {
        let ctrl = self.ctrl;
        let door_section_occupied = self.eng.element(door, section_index).is_occupied;
        let Some(shape_entry) = shape_entry else {
            return false;
        };
        if door_section_occupied {
            return false;
        }
        let next_room = self.eng.expandable_other_room(door, from_element.next_room);
        let layer = self.room_layer(from_element.next_room.unwrap());
        let lu = layer as usize;
        let shape_entry_middle = shape_entry.a.middle_point(&shape_entry.b);
        let mut bend_cost_penalty = 0.0;
        if ctrl.bend_costs[lu] > 0.0 {
            if let Some(backtrack_door) = from_element.backtrack_door {
                let from_mid = from_element.shape_entry.a.middle_point(&from_element.shape_entry.b);
                // Build vectors prev->current and current->next to detect a direction change.
                let backtrack_cog = self.eng.expandable_shape(backtrack_door).centre_of_gravity();
                let prev_dx = from_mid.x - backtrack_cog.x;
                let prev_dy = from_mid.y - backtrack_cog.y;
                let next_dx = shape_entry_middle.x - from_mid.x;
                let next_dy = shape_entry_middle.y - from_mid.y;
                let cross_product = prev_dx * next_dy - prev_dy * next_dx;
                let sq_len_prev = prev_dx * prev_dx + prev_dy * prev_dy;
                let sq_len_next = next_dx * next_dx + next_dy * next_dy;
                if sq_len_prev > 0.0 && sq_len_next > 0.0 && (cross_product * cross_product) > 0.01 * sq_len_prev * sq_len_next {
                    bend_cost_penalty = ctrl.bend_costs[lu];
                }
            }
        }
        let from_middle = from_element.shape_entry.a.middle_point(&from_element.shape_entry.b);
        let mut step = shape_entry_middle.weighted_distance(&from_middle, ctrl.trace_costs[lu].horizontal, ctrl.trace_costs[lu].vertical);
        if let Some(h) = &ctrl.history {
            // fastroute failure-only history: other nets' failing corridors cost more
            step *= h.route_factor(layer, &from_middle.middle_point(&shape_entry_middle), ctrl.net_number);
        }
        let expansion_value = from_element.expansion_value + add_costs as f64 + bend_cost_penalty + step;
        let sorting_value = expansion_value + self.destination_distance.calculate_point(&shape_entry_middle, layer);
        let room_ripped = add_costs > 0 && adjustment == Adjustment::None || from_element.already_checked && from_element.room_ripped;
        let mut new_element = MazeListElement::new(
            door,
            section_index,
            Some(from_element.door),
            from_element.section_no_of_door,
            expansion_value,
            sorting_value,
            next_room,
            shape_entry,
            room_ripped,
            adjustment,
            false,
        );
        if add_costs > 0 && adjustment == Adjustment::None {
            new_element.ripup_cost = add_costs;
        }
        self.add_element(new_element);
        true
    }

    /// Java `MazeSearchEngine.reduceTraceShapesAtTiePins(itemList, ownNetNo, autorouteTree)`.
    fn reduce_trace_shapes_at_tie_pins(&mut self, items: &ItemSet, own_net_no: NetNo) {
        for key in items.iter() {
            let item = self.board.item(key);
            if item.is_pin() && item.net_count() > 1 {
                let pin_contacts = self.board.normal_contacts(key);
                for contact in pin_contacts.iter() {
                    let c = self.board.item(contact);
                    if !c.is_trace() || c.contains_net(own_net_no) {
                        continue;
                    }
                    self.board.reduce_trace_shape_at_tie_pin(self.eng.tree, key, contact);
                }
            }
        }
    }

    /// Java `init(startItems, destinationItems)`.
    fn init(&mut self, start_items: &ItemSet, destination_items: &ItemSet) -> bool {
        let ctrl = self.ctrl;
        let tree = self.eng.tree;
        self.reduce_trace_shapes_at_tie_pins(start_items, ctrl.net_number);
        self.reduce_trace_shapes_at_tie_pins(destination_items, ctrl.net_number);
        // process the destination items
        let mut destination_ok = false;
        for key in destination_items.iter() {
            if self.eng.is_stop_requested() {
                return false;
            }
            self.eng.item_info_mut(key).start_info = false;
            for i in 0..self.board.tree_shape_count(tree, key) {
                if let Some(s) = self.board.tree_shape(tree, key, i) {
                    let layer = self.board.shape_layer(key, i);
                    self.destination_distance.join(&s.bounding_box(), layer);
                }
            }
            destination_ok = true;
        }
        if !destination_ok && ctrl.is_fanout {
            // destination set is not needed for fanout
            let bb = self.board.bounding_box;
            self.destination_distance.join(&bb, 0);
            self.destination_distance.join(&bb, ctrl.layer_count - 1);
            destination_ok = true;
        }
        if !destination_ok {
            log::debug!(target: "fr_engine::pipeline::diag", "MazeSearchEngine.init: Failed - no valid destination items found");
            return false;
        }
        // process the start items
        let mut start_rooms: Vec<RoomId> = Vec::new();
        for key in start_items.iter() {
            if self.eng.is_stop_requested() {
                return false;
            }
            self.eng.item_info_mut(key).start_info = true;
            if self.board.item(key).is_connectable_class() {
                for i in 0..self.board.tree_shape_count(tree, key) {
                    let contained_shape = self.board.trace_connection_shape(tree, key, i);
                    let layer = self.board.shape_layer(key, i);
                    let new_start_room = self.eng.add_incomplete_expansion_room(None, layer, contained_shape);
                    start_rooms.push(new_start_room);
                }
            }
        }
        // complete the start rooms
        let mut completed_start_rooms: Vec<RoomId> = Vec::new();
        if self.eng.maintain_database {
            // add the completed start rooms carried over from the last autoroute.
            completed_start_rooms.extend(self.eng.rooms_with_target_items(start_items));
        }
        for room in start_rooms {
            if self.eng.is_stop_requested() {
                return false;
            }
            let rooms = self.eng.complete_expansion_room(self.board, IncompleteRef::Arena(room));
            completed_start_rooms.extend(rooms);
        }
        // Put the ItemExpansionDoors of the completed start rooms into the maze expansion list.
        let mut start_ok = false;
        for current_room in completed_start_rooms {
            let target_doors = self.eng.room(current_room).target_doors().to_vec();
            for current_door in target_doors {
                if self.eng.is_stop_requested() {
                    return false;
                }
                if self.eng.is_destination_door(current_door) {
                    continue;
                }
                let (item, tree_entry_no, door_room) = {
                    let td = self.eng.target_door(current_door);
                    (td.item, td.tree_entry_no, td.room)
                };
                let connection_shape = self.board.trace_connection_shape(tree, item, tree_entry_no).expect("trace connection shape");
                let connection_shape = connection_shape.intersection(self.eng.room(door_room).shape());
                let current_center = connection_shape.centre_of_gravity();
                let shape_entry = FloatLine::new(current_center, current_center);
                let layer = self.room_layer(current_room);
                let sorting_value = self.destination_distance.calculate_point(&current_center, layer);
                let new_list_element = MazeListElement::new(
                    Expandable::Target(current_door),
                    0,
                    None,
                    0,
                    0.0,
                    sorting_value,
                    Some(current_room),
                    shape_entry,
                    false,
                    Adjustment::None,
                    false,
                );
                self.add_element(new_list_element);
                start_ok = true;
            }
        }
        if !start_ok {
            log::debug!(target: "fr_engine::pipeline::diag", "MazeSearchEngine.init: Failed - no accessible expansion doors found");
        }
        start_ok
    }

    /// Java `roomShapeIsThick(obstacleRoom)`.
    fn room_shape_is_thick(&self, obstacle_room: RoomId) -> bool {
        let room = self.eng.room(obstacle_room);
        let item_key = room.obstacle_item().unwrap();
        let layer = room.layer;
        let item = self.board.item(item_key);
        let obstacle_half_width = if let Some(t) = item.as_trace() {
            t.half_width().wrapping_add(self.board.clearance_compensation_value(self.eng.tree, item.clearance_class(), layer)) as f64
        } else if item.is_via() {
            let via_shape = self.board.tree_shape_on_layer(self.eng.tree, item_key, layer).expect("via tree shape");
            0.5 * via_shape.max_width()
        } else {
            log::warn!("MazeSearchEngine. room_shape_is_thick: unexpected obstacle item");
            0.0
        };
        obstacle_half_width >= self.ctrl.compensated_trace_half_width[layer as usize] as f64
    }

    /// Java `shoveTraceRoom(listElement, obstacleRoom)`: shoves a trace room and expands the
    /// corresponding doors. Returns false if no door was expanded.
    fn shove_trace_room(&mut self, list_element: &MazeListElement, obstacle_room: RoomId) -> bool {
        if list_element.section_no_of_door != 0 && list_element.section_no_of_door != self.eng.expandable_element_count(list_element.door) - 1 {
            // No delay of occupation necessary because inner sections of a door are currently
            // not shoved.
            return true;
        }
        let mut result = false;
        if list_element.adjustment != Adjustment::Right {
            let mut left_to_door_section_list = Vec::new();
            if self.check_shove_trace_line(list_element, obstacle_room, false, &mut left_to_door_section_list) {
                result = true;
            }
            for s in left_to_door_section_list {
                let adjustment = if self.eng.door(s.door).dimension == 2 { Adjustment::Left } else { Adjustment::None };
                self.expand_to_door_section(Expandable::Door(s.door), s.section_index, Some(s.section_line), list_element, 0, adjustment);
            }
        }
        if list_element.adjustment != Adjustment::Left {
            let mut right_to_door_section_list = Vec::new();
            if self.check_shove_trace_line(list_element, obstacle_room, true, &mut right_to_door_section_list) {
                result = true;
            }
            for s in right_to_door_section_list {
                let adjustment = if self.eng.door(s.door).dimension == 2 { Adjustment::Right } else { Adjustment::None };
                self.expand_to_door_section(Expandable::Door(s.door), s.section_index, Some(s.section_line), list_element, 0, adjustment);
            }
        }
        result
    }

    /// Java `checkNeckDownAtDestPin(room)`.
    fn check_neck_down_at_dest_pin(&self, room: RoomId) -> f64 {
        let layer = self.room_layer(room);
        for &t in self.eng.room(room).target_doors() {
            let item = self.eng.target_door(t).item;
            if self.board.item(item).is_pin() {
                let half = self.board.pin_trace_neckdown_half_width(item, layer);
                return self.ctrl.clamp_neckdown_half_width(half) as f64;
            }
        }
        0.0
    }

    // ------------------------------------------------------------------------------------------
    // MazeExpansionEngine

    /// Java `MazeExpansionEngine.expandToDrill(drill, fromElement, addCosts)`.
    fn expand_to_drill(&mut self, drill: DrillId, from_element: &MazeListElement, add_costs: i32) {
        let ctrl = self.ctrl;
        let from_room = from_element.next_room.unwrap();
        let layer = self.room_layer(from_room);
        let lu = layer as usize;
        let trace_half_width = ctrl.compensated_trace_half_width[lu];
        let room_shape_is_thin = self.eng.room(from_room).shape().min_width() < 2.0 * trace_half_width as f64;
        let drill_shape = self.eng.drill(drill).shape.clone();
        if room_shape_is_thin {
            let intersects_backtrack = match from_element.backtrack_door {
                None => false,
                Some(b) => drill_shape.intersects_tile(&self.eng.expandable_shape(b)),
            };
            if !intersects_backtrack {
                return;
            }
        }
        let via_radius = ctrl.via_radii[lu];
        let shrinked_drill_shape = drill_shape.shrink(via_radius);
        let mut compare_corner = from_element.shape_entry.a.middle_point(&from_element.shape_entry.b);
        if let (Expandable::Page(_), Some(Expandable::Target(t))) = (from_element.door, from_element.backtrack_door) {
            let item = self.eng.target_door(t).item;
            if self.board.item(item).is_pin() {
                let location = self.eng.drill(drill).location.to_float();
                if let Some(nearest_exit_corner) = self.board.pin_nearest_trace_exit_corner(item, &location, trace_half_width, layer) {
                    compare_corner = nearest_exit_corner;
                }
            }
        }
        let nearest_point = shrinked_drill_shape.nearest_point_approx(&compare_corner).expect("nearest point");
        let shape_entry = FloatLine::new(nearest_point, nearest_point);
        let section_index = layer - self.eng.drill(drill).first_layer;
        let mut expansion_value = from_element.expansion_value
            + add_costs as f64
            + nearest_point.weighted_distance(&compare_corner, ctrl.trace_costs[lu].horizontal, ctrl.trace_costs[lu].vertical);
        let (new_backtrack_door, new_section_no_of_backtrack_door) = if let Expandable::Page(_) = from_element.door {
            (from_element.backtrack_door, from_element.section_no_of_backtrack_door)
        } else {
            expansion_value += ctrl.min_normal_via_cost;
            (Some(from_element.door), from_element.section_no_of_door)
        };
        let sorting_value = expansion_value + self.destination_distance.calculate_point(&nearest_point, layer);
        let new_element = MazeListElement::new(
            Expandable::Drill(drill),
            section_index,
            new_backtrack_door,
            new_section_no_of_backtrack_door,
            expansion_value,
            sorting_value,
            None,
            shape_entry,
            from_element.room_ripped,
            Adjustment::None,
            false,
        );
        self.add_element(new_element);
    }

    /// Java `MazeExpansionEngine.expandToDrillPage(drillPage, fromElement)`.
    fn expand_to_drill_page(&mut self, drill_page: PageId, from_element: &MazeListElement) {
        let ctrl = self.ctrl;
        let layer = self.room_layer(from_element.next_room.unwrap());
        let lu = layer as usize;
        let from_middle = from_element.shape_entry.a.middle_point(&from_element.shape_entry.b);
        let nearest_point = self.eng.page(drill_page).shape.nearest_point_float(&from_middle);
        let expansion_value = from_element.expansion_value + ctrl.min_normal_via_cost;
        let sorting_value = expansion_value
            + nearest_point.weighted_distance(&from_middle, ctrl.trace_costs[lu].horizontal, ctrl.trace_costs[lu].vertical)
            + self.destination_distance.calculate_point(&nearest_point, layer);
        let new_element = MazeListElement::new(
            Expandable::Page(drill_page),
            layer,
            Some(from_element.door),
            from_element.section_no_of_door,
            expansion_value,
            sorting_value,
            from_element.next_room,
            from_element.shape_entry,
            from_element.room_ripped,
            Adjustment::None,
            false,
        );
        self.add_element(new_element);
    }

    /// Java `MazeExpansionEngine.expandToDrillsOfPage(fromElement)`.
    fn expand_to_drills_of_page(&mut self, from_element: &MazeListElement) {
        let from_room_layer = from_element.section_no_of_door;
        let Expandable::Page(drill_page) = from_element.door else {
            return;
        };
        let drill_list = self.eng.page_drills(self.board, drill_page, self.ctrl.attach_smd_allowed);
        for current_drill in drill_list {
            let (section_index, room_ok, occupied) = {
                let d = self.eng.drill(current_drill);
                let section_index = from_room_layer - d.first_layer;
                if section_index < 0 || section_index >= d.room_arr.len() as i32 {
                    continue;
                }
                (section_index, d.room_arr[section_index as usize] == from_element.next_room, d.elements[section_index as usize].is_occupied)
            };
            let _ = section_index;
            if !room_ok || occupied {
                continue;
            }
            self.expand_to_drill(current_drill, from_element, 0);
        }
    }

    /// Java `MazeExpansionEngine.expandToOtherLayers(listElement)`.
    fn expand_to_other_layers(&mut self, list_element: &MazeListElement) {
        let ctrl = self.ctrl;
        let Expandable::Drill(current_drill) = list_element.door else {
            return;
        };
        let (first_layer, last_layer, room_arr) = {
            let d = self.eng.drill(current_drill);
            (d.first_layer, d.last_layer, d.room_arr.clone())
        };
        let from_layer = first_layer + list_element.section_no_of_door;
        let via_lower_bound;
        let via_upper_bound;
        let mut smd_attached_on_component_side = false;
        let mut smd_attached_on_solder_side = false;
        let room_ripped;
        let section_room = room_arr[list_element.section_no_of_door as usize].expect("drill room");
        if let Some(obstacle_item) = self.eng.room(section_room).obstacle_item() {
            if !ctrl.ripup_allowed {
                return;
            }
            let item = self.board.item(obstacle_item);
            if !item.is_via() {
                return;
            }
            let padstack = item.padstack(self.board).expect("via padstack");
            if !ctrl.via_rule.contains_padstack(padstack.id, &self.board.rules.via_infos) || item.clearance_class() != ctrl.via_clearance_class {
                return;
            }
            via_lower_bound = padstack.from_layer();
            via_upper_bound = padstack.to_layer();
            room_ripped = true;
        } else {
            let net_numbers = [ctrl.net_number];
            room_ripped = false;
            let via_lower_limit = first_layer.max(ctrl.via_lower_bound);
            let via_upper_limit = last_layer.min(ctrl.via_upper_bound);
            let mut current_layer = from_layer;
            loop {
                let current_room_shape = self.eng.room(room_arr[(current_layer - first_layer) as usize].expect("drill room")).shape().clone();
                let drill_result = self.check_layer_with_any_matching_via(current_drill, current_layer, &current_room_shape, &net_numbers);
                if drill_result == CheckDrillResult::NotDrillable {
                    via_lower_bound = current_layer + 1;
                    break;
                } else if drill_result == CheckDrillResult::DrillableWithAttachSmd {
                    if current_layer == 0 {
                        smd_attached_on_component_side = true;
                    } else if current_layer == ctrl.layer_count - 1 {
                        smd_attached_on_solder_side = true;
                    }
                }
                if current_layer <= via_lower_limit {
                    via_lower_bound = via_lower_limit;
                    break;
                }
                current_layer -= 1;
            }
            if via_lower_bound > first_layer {
                return;
            }
            current_layer = from_layer + 1;
            loop {
                if current_layer > via_upper_limit {
                    via_upper_bound = via_upper_limit;
                    break;
                }
                let current_room_shape = self.eng.room(room_arr[(current_layer - first_layer) as usize].expect("drill room")).shape().clone();
                let drill_result = self.check_layer_with_any_matching_via(current_drill, current_layer, &current_room_shape, &net_numbers);
                if drill_result == CheckDrillResult::NotDrillable {
                    via_upper_bound = current_layer - 1;
                    break;
                } else if drill_result == CheckDrillResult::DrillableWithAttachSmd && current_layer == ctrl.layer_count - 1 {
                    smd_attached_on_solder_side = true;
                }
                current_layer += 1;
            }
            if via_upper_bound < last_layer {
                return;
            }
        }
        for to_layer in via_lower_bound..=via_upper_bound {
            if to_layer == from_layer {
                continue;
            }
            let (current_first_layer, current_last_layer) = if to_layer < from_layer { (to_layer, from_layer) } else { (from_layer, to_layer) };
            let mut mask_found = false;
            for m in &ctrl.via_infos {
                if current_first_layer >= m.from_layer && current_last_layer <= m.to_layer && m.from_layer >= via_lower_bound && m.to_layer <= via_upper_bound {
                    let mask_ok = !(m.from_layer == 0 && smd_attached_on_component_side || m.to_layer == ctrl.layer_count - 1 && smd_attached_on_solder_side)
                        || m.attach_smd_allowed;
                    if mask_ok {
                        mask_found = true;
                        break;
                    }
                }
            }
            if !mask_found {
                continue;
            }
            let current_room_index = to_layer - first_layer;
            if self.eng.element(Expandable::Drill(current_drill), current_room_index).is_occupied {
                continue;
            }
            let expansion_value = list_element.expansion_value + ctrl.add_via_costs[from_layer as usize][to_layer as usize] as f64;
            let shape_entry_middle = list_element.shape_entry.a.middle_point(&list_element.shape_entry.b);
            let sorting_value = expansion_value + self.destination_distance.calculate_point(&shape_entry_middle, to_layer);
            let new_element = MazeListElement::new(
                Expandable::Drill(current_drill),
                current_room_index,
                Some(Expandable::Drill(current_drill)),
                list_element.section_no_of_door,
                expansion_value,
                sorting_value,
                room_arr[current_room_index as usize],
                list_element.shape_entry,
                room_ripped,
                Adjustment::None,
                false,
            );
            self.add_element(new_element);
        }
    }

    /// Java `MazeExpansionEngine.checkLayerWithAnyMatchingVia(drill, layer, roomShape,
    /// netNumbers)`.
    fn check_layer_with_any_matching_via(&mut self, drill: DrillId, layer: LayerNo, room_shape: &TileShape, net_numbers: &[NetNo]) -> CheckDrillResult {
        let ctrl = self.ctrl;
        let location = self.eng.drill(drill).location.clone();
        let mut drillable_with_attach_smd = false;
        for i in 0..ctrl.via_rule.via_count() {
            let via_info = self.board.rules.via_infos[ctrl.via_rule.get_via(i)].clone();
            let (from, to, via_radius) = {
                let via_padstack = self.board.library.padstacks.get(via_info.get_padstack()).expect("via padstack");
                let via_radius = match via_padstack.get_shape(layer) {
                    None => 0.0,
                    Some(s) => 0.5 * s.max_width(),
                };
                (via_padstack.from_layer(), via_padstack.to_layer(), via_radius)
            };
            if layer < from || layer > to {
                continue;
            }
            let required_radius = jmax(via_radius, ctrl.trace_half_width[layer as usize] as f64);
            let result = forced_via_inserter::check_layer(
                self.board,
                required_radius,
                via_info.get_clearance_class_index(),
                via_info.attach_smd_allowed(),
                room_shape,
                &location,
                layer,
                net_numbers,
                ctrl.max_shove_trace_recursion_depth,
                0,
                ctrl.trace_half_width[layer as usize],
                ctrl.trace_clearance_class,
            );
            if result == CheckDrillResult::Drillable {
                return result;
            }
            if result == CheckDrillResult::DrillableWithAttachSmd {
                drillable_with_attach_smd = true;
            }
        }
        if drillable_with_attach_smd {
            CheckDrillResult::DrillableWithAttachSmd
        } else {
            CheckDrillResult::NotDrillable
        }
    }

    // ------------------------------------------------------------------------------------------
    // MazeRipupResolver

    /// Java `MazeRipupResolver.checkRipup(listElement, obstacleItem, doorIsSmall)`: the ripup
    /// cost of the next room, or -1 if it cannot be ripped.
    fn check_ripup(&mut self, list_element: &MazeListElement, obstacle_item: ItemKey, door_is_small: bool) -> i32 {
        let ctrl = self.ctrl;
        if !self.board.item(obstacle_item).is_routable() {
            return -1;
        }
        if door_is_small && !self.enter_through_small_door(list_element, obstacle_item) {
            return -1;
        }
        let previous_room = self.eng.expandable_other_room(list_element.door, list_element.next_room);
        let room_was_shoved = list_element.adjustment != Adjustment::None;
        let previous_item = previous_room.and_then(|r| self.eng.room(r).obstacle_item());
        if room_was_shoved {
            if let Some(p) = previous_item {
                if p != obstacle_item && self.board.item(p).shares_net(self.board.item(obstacle_item)) {
                    return -1;
                }
            }
        } else if previous_item == Some(obstacle_item) {
            return ALREADY_RIPPED_COSTS;
        }
        let mut fanout_via_cost_factor = 1.0;
        let mut cost_factor = 1.0;
        let preserve_fanout_protection = !ctrl.remove_unconnected_vias && ctrl.ripup_costs <= ctrl.start_ripup_costs.wrapping_mul(2);
        let item = self.board.item(obstacle_item);
        if let Some(t) = item.as_trace() {
            cost_factor = t.half_width() as f64;
            if preserve_fanout_protection {
                fanout_via_cost_factor = self.calc_fanout_via_ripup_cost_factor(obstacle_item);
            }
        } else if item.is_via() {
            let mut look_if_fanout_via = preserve_fanout_protection;
            let contact_list = self.board.normal_contacts(obstacle_item);
            let mut contact_count = 0;
            for contact in contact_list.iter() {
                let c = self.board.item(contact);
                if !c.is_trace() || c.is_user_fixed() {
                    return -1;
                }
                contact_count += 1;
                cost_factor = jmax(cost_factor, c.as_trace().unwrap().half_width() as f64);
                if look_if_fanout_via && !ctrl.is_fanout {
                    let current = self.calc_fanout_via_ripup_cost_factor(contact);
                    if current > 1.0 {
                        fanout_via_cost_factor = current;
                        look_if_fanout_via = false;
                    }
                }
            }
            if fanout_via_cost_factor <= 1.0 {
                cost_factor *= 0.5 * (contact_count - 1).max(0) as f64;
            }
        }
        let mut ripup_cost = ctrl.ripup_costs as f64 * cost_factor;
        let mut detour = 1.0;
        if fanout_via_cost_factor <= 1.0 && !ctrl.is_fanout {
            if let Some(c) = Connection::get(self.eng, self.board, obstacle_item) {
                detour = self.eng.connections[c].get_detour(self.board);
            }
        }
        let randomize = ctrl.ripup_pass_no >= 4 && ctrl.ripup_pass_no % 3 != 0;
        if randomize {
            let random_number = self.random_generator.next_double();
            let random_factor = 0.5 + random_number * random_number;
            detour *= random_factor;
        }
        ripup_cost /= detour;
        ripup_cost *= fanout_via_cost_factor;
        if let Some(h) = &ctrl.history {
            // fastroute failure-only history: ripping in the net's own failing corridor is cheaper
            let at = list_element.shape_entry.a.middle_point(&list_element.shape_entry.b);
            ripup_cost /= h.rip_divisor(self.room_layer(list_element.next_room.unwrap()), &at, ctrl.net_number);
        }
        let mut result = (ripup_cost as i32).max(1);
        let max_ripup_costs = i32::MAX / 100;
        result = result.min(max_ripup_costs);
        result
    }

    /// Java `MazeRipupResolver.calcFanoutViaRipupCostFactor(trace)`.
    fn calc_fanout_via_ripup_cost_factor(&self, trace: ItemKey) -> f64 {
        for i in 0..2 {
            let current_end_contacts = if i == 0 { self.board.trace_start_contacts(trace) } else { self.board.trace_end_contacts(trace) };
            if current_end_contacts.len() != 1 {
                continue;
            }
            let contact = current_end_contacts.first().unwrap();
            let c = self.board.item(contact);
            let mut protect_fanout_via = false;
            if c.is_pin() && c.first_layer(self.board) == c.last_layer(self.board) {
                protect_fanout_via = true;
            } else if let Some(ct) = c.as_trace() {
                if c.fixed_state() == FixedState::ShoveFixed && ct.corner_count() == 2 {
                    protect_fanout_via = true;
                }
            }
            if protect_fanout_via {
                let t = self.board.item(trace).as_trace().unwrap();
                let mut f = t.half_width() as f64 / t.length();
                f *= f;
                f *= FANOUT_COST_CONSTANT;
                return jmax(f, 1.0);
            }
        }
        1.0
    }

    /// Java `MazeRipupResolver.checkLeavingRippedItem(listElement)`.
    fn check_leaving_ripped_item(&mut self, list_element: &MazeListElement) -> bool {
        let Expandable::Door(_) = list_element.door else {
            return false;
        };
        let Some(from_room) = self.eng.expandable_other_room(list_element.door, list_element.next_room) else {
            return false;
        };
        let Some(current_item) = self.eng.room(from_room).obstacle_item() else {
            return false;
        };
        if !self.board.item(current_item).is_routable() {
            return false;
        }
        self.enter_through_small_door(list_element, current_item)
    }

    /// Java `MazeRipupResolver.enterThroughSmallDoor(listElement, ignoreItem)`.
    fn enter_through_small_door(&mut self, list_element: &MazeListElement, ignore_item: ItemKey) -> bool {
        if self.eng.expandable_dimension(list_element.door) != 1 {
            return false;
        }
        let door_shape = self.eng.expandable_shape(list_element.door);
        let mut door_line: Option<Line> = None;
        let mut prev_corner = door_shape.corner_approx(0);
        let corner_count = door_shape.border_line_count();
        for i in 1..corner_count {
            let next_corner = door_shape.corner_approx(i);
            if next_corner.distance_square(&prev_corner) > 1.0 {
                door_line = Some(door_shape.border_line(i - 1));
                break;
            }
            prev_corner = next_corner;
        }
        let Some(door_line) = door_line else {
            return false;
        };
        let door_center: IntPoint = door_shape.centre_of_gravity().round();
        let current_layer = self.room_layer(list_element.next_room.unwrap());
        let check_radius = self.ctrl.compensated_trace_half_width[current_layer as usize] + TRACE_WIDTH_TOLERANCE;
        let lines = vec![
            door_line.translate(check_radius as f64),
            Line::from_point_direction(Point::Int(door_center), door_line.direction().turn_45_degree(2)),
            door_line.translate(-check_radius as f64),
        ];
        let check_polyline = Polyline::from_lines(lines);
        let Some(check_shape) = check_polyline.offset_shape(check_radius, 0) else {
            return false;
        };
        let ignore_net_nos = [self.ctrl.net_number];
        let overlapping_objects =
            self.board.overlapping_objects_in(self.eng.tree, &fr_geom::ConvexShape::Tile(check_shape), current_layer, &ignore_net_nos);
        let ignore = self.board.item(ignore_item);
        for object in overlapping_objects {
            let Some(current_item) = object.item() else { continue };
            if current_item == ignore_item {
                continue;
            }
            if !self.board.item(current_item).shares_net(ignore) {
                return false;
            }
            if !self.board.normal_contacts(current_item).contains(ignore.id()) {
                return false;
            }
        }
        true
    }

    // ------------------------------------------------------------------------------------------
    // MazeTraceShover

    /// Java `MazeTraceShover.checkShoveTraceLine(listElement, obstacleRoom, board, ctrl,
    /// shoveToTheLeft, toDoorList)`: returns false if the algorithm did not succeed and shoving
    /// from another door section may be more successful.
    fn check_shove_trace_line(&mut self, list_element: &MazeListElement, obstacle_room: RoomId, shove_to_the_left: bool, to_door_list: &mut Vec<DoorSection>) -> bool {
        let ctrl = self.ctrl;
        let Expandable::Door(from_door) = list_element.door else {
            return true;
        };
        let (obstacle_trace, trace_corner_no) = match &self.eng.room(obstacle_room).kind {
            RoomKind::Obstacle { item, index_in_item, .. } => (*item, *index_in_item),
            _ => return true,
        };
        let Some(trace) = self.board.item(obstacle_trace).as_trace() else {
            return true;
        };
        let trace_layer = self.room_layer(obstacle_room);
        let tl = trace_layer as usize;
        // only traces with the same halfwidth and the same clearance class can be shoved.
        if trace.half_width() != ctrl.trace_half_width[tl] || self.board.item(obstacle_trace).clearance_class() != ctrl.trace_clearance_class {
            return true;
        }
        let compensated_trace_half_width = ctrl.compensated_trace_half_width[tl] as f64;
        let from_door_shape = self.eng.door_shape(from_door);
        if from_door_shape.max_width() < 2.0 * compensated_trace_half_width {
            return true;
        }
        let trace_polyline = trace.polyline().clone();
        if trace_corner_no < 0 || trace_corner_no >= trace_polyline.lines.len() as i32 - 2 {
            return false;
        }
        let room_doors: Vec<DoorId> = self.eng.room(obstacle_room).doors.clone();
        let from_door_dimension = self.eng.door(from_door).dimension;
        let mut shove_line_segment: LineSegment;
        if from_door_dimension == 2 {
            // shove from a link door into the direction of the other link door.
            let Some(other_room) = self.eng.door_other_complete_room(from_door, obstacle_room) else {
                return false;
            };
            let Some(other_item) = self.eng.room(other_room).obstacle_item() else {
                return false;
            };
            if !self.end_points_matching(obstacle_trace, other_item) {
                return false;
            }
            let door_center = from_door_shape.centre_of_gravity();
            let corner1 = trace_polyline.corner_approx(trace_corner_no);
            let corner2 = trace_polyline.corner_approx(trace_corner_no + 1);
            if corner1.distance_square(&corner2) < 1.0 {
                // shoveLineSegment may be reduced to a point
                return false;
            }
            let shove_into_direction_of_trace_start = door_center.distance_square(&corner2) < door_center.distance_square(&corner1);
            shove_line_segment = LineSegment::from_polyline(&trace_polyline, trace_corner_no + 1).expect("line segment");
            if shove_into_direction_of_trace_start {
                // shove from the endpoint to the start point of the line segment
                shove_line_segment = shove_line_segment.opposite();
            }
        } else {
            let from_room = self.eng.door_other_complete_room(from_door, obstacle_room).expect("from room");
            let from_point = self.eng.room(from_room).shape().centre_of_gravity();
            let shove_trace_line = trace_polyline.lines[(trace_corner_no + 1) as usize].clone();
            let door_line_segment = from_door_shape.diagonal_corner_segment().expect("door line segment");
            let side_of_trace_line = shove_trace_line.side_of_float_tol(&door_line_segment.a, 0.0);
            let polar_line_segment = from_door_shape.polar_line_segment(&from_point).expect("polar line segment");
            let door_line_swapped =
                polar_line_segment.b.distance_square(&door_line_segment.a) < polar_line_segment.a.distance_square(&door_line_segment.a);
            // shove only from the right most section to the right or from the left most section
            // to the left.
            let shape_entry_check_distance = compensated_trace_half_width + 5.0;
            let check_dist_square = shape_entry_check_distance * shape_entry_check_distance;
            let se = &list_element.shape_entry;
            let section_ok = if shove_to_the_left && !door_line_swapped || !shove_to_the_left && door_line_swapped {
                list_element.section_no_of_door == self.eng.expandable_element_count(list_element.door) - 1
                    && (se.a.distance_square(&door_line_segment.b) <= check_dist_square || se.b.distance_square(&door_line_segment.b) <= check_dist_square)
            } else {
                list_element.section_no_of_door == 0
                    && (se.a.distance_square(&door_line_segment.a) <= check_dist_square || se.b.distance_square(&door_line_segment.a) <= check_dist_square)
            };
            if !section_ok {
                return false;
            }
            // create the line segment for shoving
            let shrinked_line_segment = polar_line_segment.shrink_segment(compensated_trace_half_width);
            let perpendicular_direction = shove_trace_line.direction().turn_45_degree(2);
            let lines = &trace_polyline.lines;
            let no = trace_corner_no as usize;
            let forward = |p: IntPoint| LineSegment::new(Line::from_point_direction(Point::Int(p), perpendicular_direction.clone()), lines[no + 1].clone(), lines[no + 2].clone());
            let backward =
                |p: IntPoint| LineSegment::new(Line::from_point_direction(Point::Int(p), perpendicular_direction.clone()), lines[no + 1].opposite(), lines[no].opposite());
            shove_line_segment = if side_of_trace_line == Side::OnTheLeft {
                if shove_to_the_left {
                    forward(shrinked_line_segment.b.round())
                } else {
                    backward(shrinked_line_segment.a.round())
                }
            } else if shove_to_the_left {
                backward(shrinked_line_segment.b.round())
            } else {
                forward(shrinked_line_segment.a.round())
            };
        }
        let trace_half_width = ctrl.trace_half_width[tl];
        let net_numbers = [ctrl.net_number];
        let mut shove_width = self.board.check_trace_segment_ls(&shove_line_segment, trace_layer, &net_numbers, trace_half_width, ctrl.trace_clearance_class, true);
        let mut segment_shortened = false;
        if shove_width < i32::MAX as f64 {
            // shorten shoveLineSegment
            shove_width -= 1.0;
            if shove_width <= 0.0 {
                return true;
            }
            shove_line_segment = shove_line_segment.change_length_approx(shove_width);
            segment_shortened = true;
        }
        let from_corner = shove_line_segment.start_point_approx();
        let to_corner = shove_line_segment.end_point_approx();
        let segment_is_point = from_corner.distance_square(&to_corner) < 0.1;
        if !segment_is_point {
            shove_width = TraceShover::check_line_segment(
                self.board,
                &shove_line_segment,
                shove_to_the_left,
                trace_layer,
                &net_numbers,
                trace_half_width,
                ctrl.trace_clearance_class,
                ctrl.max_shove_trace_recursion_depth,
                ctrl.max_shove_via_recursion_depth,
            );
            if shove_width <= 0.0 {
                return true;
            }
        }
        // Put the doors on this side of the room into toDoorList
        if segment_shortened {
            shove_width = jmin(shove_width, from_corner.distance(&to_corner));
        }
        let shove_line = shove_line_segment.get_line().clone();
        // fromDoorCompareDistance is used to check, that a door is between fromDoor and the end
        // point of the shove line.
        let from_door_compare_distance =
            if from_door_dimension == 2 || segment_is_point { f64::MAX } else { to_corner.distance_square(&from_door_shape.corner_approx(0)) };
        for current_door in room_doors {
            if current_door == from_door {
                continue;
            }
            {
                let d = self.eng.door(current_door);
                if let (Some(i1), Some(i2)) = (self.eng.room(d.first_room).obstacle_item(), self.eng.room(d.second_room).obstacle_item()) {
                    if i1 != i2 {
                        // there may be topological problems at a trace fork
                        continue;
                    }
                }
            }
            let current_door_shape = self.eng.door_shape(current_door);
            if self.eng.door(current_door).dimension == 2 && shove_width >= i32::MAX as f64 {
                let add_link_door = current_door_shape.contains_float(&to_corner);
                if add_link_door {
                    let line_sections = self.eng.door_section_segments(current_door, compensated_trace_half_width);
                    to_door_list.push(DoorSection { door: current_door, section_index: 0, section_line: line_sections[0] });
                }
            } else if !segment_is_point {
                // now currentDoor is 1-dimensional; check, that currentDoor is on the same
                // borderLine as fromDoor.
                let Some(current_door_segment) = current_door_shape.diagonal_corner_segment() else {
                    log::trace!("MazeTraceShover.check_shove_trace_line: door shape is empty");
                    continue;
                };
                let start_corner_side = shove_line.side_of_float_tol(&current_door_segment.a, 0.0);
                let end_corner_side = shove_line.side_of_float_tol(&current_door_segment.b, 0.0);
                if shove_to_the_left {
                    if start_corner_side != Side::OnTheLeft || end_corner_side != Side::OnTheLeft {
                        continue;
                    }
                } else if start_corner_side != Side::OnTheRight || end_corner_side != Side::OnTheRight {
                    continue;
                }
                let current_door_line = current_door_shape.polar_line_segment(&from_corner).expect("polar line segment");
                let current_door_nearest_corner = if current_door_line.a.distance_square(&from_corner) <= current_door_line.b.distance_square(&from_corner) {
                    current_door_line.a
                } else {
                    current_door_line.b
                };
                if to_corner.distance_square(&current_door_nearest_corner) >= from_door_compare_distance {
                    // currentDoor is not located into the direction of toCorner.
                    continue;
                }
                let current_door_projection = current_door_nearest_corner.projection_approx(&shove_line);
                if current_door_projection.distance(&from_corner) + compensated_trace_half_width <= shove_width {
                    let line_sections = self.eng.door_section_segments(current_door, compensated_trace_half_width);
                    for (i, current_line_section) in line_sections.iter().enumerate() {
                        let current_section_nearest_corner = if current_line_section.a.distance_square(&from_corner) <= current_line_section.b.distance_square(&from_corner) {
                            current_line_section.a
                        } else {
                            current_line_section.b
                        };
                        let current_section_projection = current_section_nearest_corner.projection_approx(&shove_line);
                        if current_section_projection.distance(&from_corner) <= shove_width {
                            to_door_list.push(DoorSection { door: current_door, section_index: i as i32, section_line: *current_line_section });
                        }
                    }
                }
            }
        }
        true
    }

    /// Java `MazeTraceShover.endPointsMatching(trace, fromItem)`.
    fn end_points_matching(&self, trace: ItemKey, from_item: ItemKey) -> bool {
        if from_item == trace {
            return true;
        }
        let t = self.board.item(trace);
        let f = self.board.item(from_item);
        if !t.shares_net(f) {
            return false;
        }
        if f.is_pin() || f.is_via() {
            let from_center = f.center(self.board);
            from_center == t.first_corner() || from_center == t.last_corner()
        } else if f.is_trace() {
            t.first_corner() == f.first_corner() || t.first_corner() == f.last_corner() || t.last_corner() == f.first_corner() || t.last_corner() == f.last_corner()
        } else {
            false
        }
    }
}

/// Java `MazeSearchEngine.segmentProjection(fromSegment, toSegment)`: the perpendicular
/// projection of `from_segment` onto `to_segment`, `None` if it is empty.
fn segment_projection(from_segment: &FloatLine, to_segment: &FloatLine) -> Option<FloatLine> {
    let check_segment = from_segment.adjust_direction(to_segment);
    let first_projection = to_segment.segment_projection(&check_segment);
    let second_projection = to_segment.segment_projection_2(&check_segment);
    match (first_projection, second_projection) {
        (Some(f), Some(s)) => {
            // (Java compares the object identity with toSegment.a / toSegment.b first; the
            // distance comparison below gives the same point in these cases.)
            let result_a = if f.a == to_segment.a || s.a == to_segment.a {
                to_segment.a
            } else if f.a.distance_square(&to_segment.a) <= s.a.distance_square(&to_segment.a) {
                f.a
            } else {
                s.a
            };
            let result_b = if f.b == to_segment.b || s.b == to_segment.b {
                to_segment.b
            } else if f.b.distance_square(&to_segment.b) <= s.b.distance_square(&to_segment.b) {
                f.b
            } else {
                s.b
            };
            Some(FloatLine::new(result_a, result_b))
        }
        (Some(f), None) => Some(f),
        (None, s) => s,
    }
}
