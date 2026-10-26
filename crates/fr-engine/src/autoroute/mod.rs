//! The autorouter core (porting unit U8): expansion rooms and doors, drills, the maze search and
//! the construction / insertion of the found connection.
//!
//! | Module | Java |
//! |---|---|
//! | [`rooms`] | `IncompleteFreeSpaceExpansionRoom` as a value (the `completeShape` input/output) |
//! | [`engine`] | `maze/AutorouteEngine`, `expansion/{ExpansionRoom, ExpansionDoor, TargetItemExpansionDoor, ObstacleExpansionRoom, CompleteFreeSpaceExpansionRoom, ...}`, `ItemAutorouteInfo`, `drill/*` |
//! | [`neighbours`] | `expansion/SortedRoomNeighbours` (any angle) |
//! | [`neighbours45`] | `expansion/Sorted45DegreeRoomNeighbours` |
//! | [`neighbours90`] | `expansion/SortedOrthogonalRoomNeighbours` |
//! | [`control`] | `maze/AutorouteControl`, `AutorouteAttemptResult` |
//! | [`destination`] | `maze/DestinationDistance` |
//! | [`maze`] | `maze/{MazeSearchEngine, MazeExpansionEngine, MazeRipupResolver, MazeTraceShover, MazeListElement, MazeSearchElement}` |
//! | [`connection`] | `path/Connection` |
//! | [`locator`] | `path/{FoundConnectionLocator, FoundConnectionLocator45Degree, FoundConnectionLocatorAnyAngle}` |
//! | [`inserter`] | `path/FoundConnectionInserter` |
//! | [`router`] | `pipeline/AutorouteConnectionRouter.route` (the per connection entry point of the batch autorouter) and `RoutingBoard.{initAutoroute, autoroute, fanout}` |
//!
//! Design (see also `docs/PORTING.md`, "Expansion rooms/doors/drills"):
//! * All expansion objects of one [`AutorouteEngine`](engine::AutorouteEngine) live in arenas
//!   ([`RoomId`](engine::RoomId), [`DoorId`](engine::DoorId), ...). Java object identity is
//!   arena index identity. The Java `getId()` values used for ordering (maze queue, neighbour
//!   sorting) are computed from the stored Java ids with `i32` wrapping arithmetic
//!   ([`AutorouteEngine::expandable_java_id`](engine::AutorouteEngine::expandable_java_id)).
//! * Complete free space rooms are inserted into the board's autoroute search tree with
//!   `RoomKey(room arena index)`, exactly when Java inserts them, and removed in Java order.
//! * The Java `Item.autorouteInfo` (and `Via.autorouteDrillInfo`) are owned by the engine
//!   (a map from [`ItemKey`](crate::board::ItemKey)); Java clears them in `AutorouteEngine.clear`
//!   (`clearAllItemTemporaryAutorouteData`), which the port does by clearing the map.
//! * The maze queue is a [`JavaTreeSet`](fr_jcompat::treemap::JavaTreeSet) with the
//!   `MazeListElement.compareTo` comparator evaluated live (the id of a drill page changes when
//!   its drills are calculated, like in Java), so equal keys are dropped (the old element is kept).
//! * The neighbour sets (`TreeSet<SortedRoomNeighbour>`, non-transitive tolerance comparator in the
//!   any angle case) are `JavaTreeSet`s too.
//! * Java exceptions that the Java code catches (e.g. in `completeExpansionRoom`,
//!   `autorouteConnection`) are modelled with `Result<_, JavaException>` where they can occur.

pub mod connection;
pub mod control;
pub mod destination;
pub mod engine;
pub mod history;
pub mod inserter;
pub mod locator;
pub mod maze;
pub mod neighbours;
pub mod neighbours45;
pub mod neighbours90;
pub mod rooms;
pub mod router;

pub use control::{AutorouteAttemptResult, AutorouteControl, ViaMask};
pub use engine::{AutorouteEngine, DoorId, DrillId, Expandable, PageId, RoomId, TargetDoorId};

/// Java `AutorouteEngine.TRACE_WIDTH_TOLERANCE`.
pub const TRACE_WIDTH_TOLERANCE: i32 = 2;

/// A Java exception that the Java code catches further up (the port returns it as an error).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct JavaException;

/// Result of an operation that may end in a caught Java exception.
pub type JResult<T> = Result<T, JavaException>;
