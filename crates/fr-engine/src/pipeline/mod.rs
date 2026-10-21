//! The routing pipeline (porting unit U9): Java `autoroute/pipeline/RoutingPipeline` and the
//! stages it runs.
//!
//! | Module | Java |
//! |---|---|
//! | [`autorouter`] | `BatchAutorouter`, `AutorouteBatchLoop`, `AutoroutePassRunner.runSingleThread`, strict DRC of `AutorouteConnectionRouter` |
//! | [`fanout`] | `BatchFanout` |
//! | [`optimizer`] | `BatchOptimizer` (`OptimizeCandidateTask`, `WorkerBoardState`, `ReadSortedRouteItems`), `ItemRouteResult` |
//! | [`history`] | `BoardHistory`, `BasicBoard.getHash` |
//! | [`stats`] | memoized `new BoardStatistics(board)` |
//!
//! Not ported: events, GUI hooks, analytics, profiling, the result manifest and the dead
//! multi-threaded autorouter (`runMultiThread`, `BatchAutorouterThread`).
//!
//! The Java `StoppableThread` of the job is a [`StopToken`]: `requestStopAutoRouter` (max
//! passes reached, stagnation) stops the autorouter only, and — like in Java — also the
//! autorouting passes inside the optimizer candidates (`autoroutePassesForOptimizingItem`
//! checks `isStopAutoRouterRequested`).
//!
//! The Java `TimeLimit`s of the board algorithms come from `RoutingBoard::time_limits`: for
//! parity runs the deterministic count mode of the Java parity jar
//! ([`TimeLimit::Count`](crate::datastructures::TimeLimit::Count), `limit_ms * 10` calls of
//! `limitExceeded()` per instance; the call sites are `AutorouteEngine.isStopRequested`,
//! `TraceTightener.isStopRequested`, `TraceShover.check`, `DrillItemMover.check`, called
//! exactly as often as in Java). The other wall clock limits (fanout / optimizer
//! `timeout_string`, job timeout) are only active with [`PipelineContext::wall_clock_limits`].
//!
//! Parity status (`scripts/parity-route.sh`, see the U9 report): SES and per-pass scores
//! identical to the Java parity build on all benchmark boards tried, autorouter only and full
//! pipeline (optimizer `java-compat`).

pub mod autorouter;
pub mod fanout;
pub mod history;
pub mod optimizer;
mod parallel_pass;
pub mod repair;
pub mod stats;

use fr_settings::{RouterSettings, ALGORITHM_CURRENT};

use crate::board::RoutingBoard;
use crate::datastructures::StopToken;

pub use autorouter::{BatchAutorouter, PassCounters};
pub use history::{board_hash, BoardHistory};
pub use optimizer::{BatchOptimizer, OptimizerMode};
pub use stats::StatsCache;

/// The job state shared by the stages.
#[derive(Clone)]
pub struct PipelineContext {
    /// Java `job.thread` stop requests.
    pub stop: StopToken,
    /// True to honour the wall clock stage limits (`fanout.timeout_string`,
    /// `optimizer.timeout_string`); false for deterministic runs.
    pub wall_clock_limits: bool,
    /// How the optimizer evaluates its candidates.
    pub optimizer_mode: OptimizerMode,
    /// fastroute improvements that change results compared with Freerouting
    /// (see docs/IMPROVEMENTS.md). Off for `--parity`.
    pub enhancements: bool,
    /// fastroute multi-start (with `enhancements`): if the autorouter leaves connections
    /// unrouted, it is rerun this many times in total with shuffled first-pass orders (in
    /// parallel) and the best board is kept. 1 = off.
    pub multi_start: usize,
    /// fastroute cost portfolio (`--multi-start-costs`): multi-start variants also scale the
    /// via and rip-up costs (see [`PORTFOLIO`]), not only the first-pass order.
    pub cost_portfolio: bool,
    /// fastroute region repair after the routing stage (`--repair`); `None` = off.
    pub repair: Option<repair::RepairOptions>,
    /// Called with the current board whenever a stage reached a new best state (see
    /// [`CheckpointKey`]); the CLI writes it as the session file, so a run that is stopped or
    /// killed still leaves its best result behind.
    pub checkpoint: Option<Checkpoint>,
    /// fastroute: told about the progress of the stages (the CLI's live viewer, `--live`).
    /// Must not change anything the routing depends on: it only reads the board.
    pub observer: Option<Observer>,
}

/// Callback for [`PipelineContext::checkpoint`].
pub type Checkpoint = std::sync::Arc<dyn Fn(&RoutingBoard, CheckpointKey) + Send + Sync>;

/// Callback for [`PipelineContext::observer`].
pub type Observer = std::sync::Arc<dyn Fn(&RoutingBoard, &LiveEvent) + Send + Sync>;

/// What [`PipelineContext::observer`] is told.
#[derive(Clone, Copy, Debug)]
pub enum LiveEvent<'a> {
    /// A stage started (`fanout`, `autorouter`, `multi-start`, `optimizer`, ...).
    Stage(&'a str),
    /// An autorouting pass committed a connection (`done` of `total` items of the pass).
    Connection { pass_no: i32, done: usize, total: usize, counters: PassCounters },
    /// A fanout pass checked a pin (`done` of `total` SMD pins of the pass).
    Fanout { pass_no: i32, done: i32, total: i32, routed: i32, not_routed: i32 },
    /// An autorouting pass ended.
    RouterPass { pass_no: i32, secs: f64, counters: PassCounters, incomplete: i32, violations: i32, score: f32 },
    /// An optimizer pass ended.
    OptimizerPass { pass_no: i32, secs: f64, incomplete: i32, violations: i32, score: f32 },
}

/// How good a checkpointed board is: fewer unrouted connections first, then fewer clearance
/// violations, then the later stage (the optimizer only accepts non-worse boards), then the
/// stage's score.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CheckpointKey {
    pub incomplete: i32,
    pub violations: i32,
    /// 0 = routing, 1 = optimizer.
    pub stage: u8,
    pub score: f32,
}

impl CheckpointKey {
    /// True if `self` is a better board than `other`.
    pub fn better_than(&self, other: &CheckpointKey) -> bool {
        (other.incomplete, other.violations, self.stage, ordered_f32(self.score))
            > (self.incomplete, self.violations, other.stage, ordered_f32(other.score))
    }
}

impl std::fmt::Debug for PipelineContext {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PipelineContext")
            .field("wall_clock_limits", &self.wall_clock_limits)
            .field("optimizer_mode", &self.optimizer_mode)
            .field("enhancements", &self.enhancements)
            .field("multi_start", &self.multi_start)
            .field("cost_portfolio", &self.cost_portfolio)
            .field("repair", &self.repair)
            .field("checkpoint", &self.checkpoint.is_some())
            .field("observer", &self.observer.is_some())
            .finish()
    }
}

impl PipelineContext {
    /// Reports a board to the checkpoint callback, if any.
    pub fn checkpoint(&self, board: &RoutingBoard, key: CheckpointKey) {
        if let Some(cb) = &self.checkpoint {
            cb(board, key);
        }
    }

    /// Reports progress to the observer, if any.
    pub fn observe(&self, board: &RoutingBoard, event: &LiveEvent) {
        if let Some(cb) = &self.observer {
            cb(board, event);
        }
    }
}

impl Default for PipelineContext {
    fn default() -> Self {
        PipelineContext {
            stop: StopToken::new(),
            wall_clock_limits: true,
            optimizer_mode: OptimizerMode::JavaCompat,
            enhancements: false,
            multi_start: 1,
            cost_portfolio: false,
            repair: None,
            checkpoint: None,
            observer: None,
        }
    }
}

/// Multi-start variants run in parallel but each as long as the first run; after a longer
/// first run they are skipped (a large board then spends an hour more for a few connections).
const MULTI_START_MAX_FIRST_RUN: std::time::Duration = std::time::Duration::from_secs(600);

/// fastroute cost portfolio: (via cost factor, rip-up cost factor) of multi-start variant
/// `v`, taken at `(v - 1) % len`. Cheap vias help boards that run out of room on one layer,
/// dear vias boards that run out of via sites; high rip-up costs keep routed nets in place,
/// low ones let new connections push through.
const PORTFOLIO: [(f64, f64); 6] = [(0.5, 1.0), (2.0, 1.0), (1.0, 3.0), (0.5, 3.0), (2.0, 0.5), (1.0, 0.5)];

/// The settings of multi-start variant `v` under the cost portfolio.
fn portfolio_settings(settings: &RouterSettings, v: usize) -> RouterSettings {
    let (via, ripup) = PORTFOLIO[(v - 1) % PORTFOLIO.len()];
    let scale = |x: i32, f: f64| ((x as f64 * f).round() as i32).max(1);
    let mut s = settings.clone();
    s.scoring.via_costs = Some(scale(settings.get_via_costs(), via));
    s.scoring.plane_via_costs = Some(scale(settings.get_plane_via_costs(), via));
    s.set_start_ripup_costs(scale(settings.get_start_ripup_costs(), ripup));
    s
}

/// The outcome of [`run_pipeline`].
#[derive(Clone, Copy, Debug, Default)]
pub struct PipelineOutcome {
    pub fanout_timed_out: bool,
    pub optimizer_timed_out: bool,
}

/// Java `RoutingPipeline.create(job).run()`: fanout + autorouter, `finishAutoroute`, then the
/// optimizer (if enabled). The board may be replaced (history restores, optimizer results).
pub fn run_pipeline(board: &mut RoutingBoard, settings: &mut RouterSettings, ctx: &PipelineContext) -> PipelineOutcome {
    // normalizeRouterAlgorithm
    if settings.autorouter.algorithm.as_deref() != Some(ALGORITHM_CURRENT) {
        log::warn!(
            "The algorithm '{}' is not supported. The default algorithm '{ALGORITHM_CURRENT}' will be used instead.",
            settings.autorouter.algorithm.as_deref().unwrap_or("null")
        );
        settings.autorouter.algorithm = Some(ALGORITHM_CURRENT.to_string());
    }
    let run_optimizer = settings.get_run_optimizer();
    if run_optimizer && settings.optimizer.algorithm.as_deref() != Some("freerouting-optimizer") {
        log::warn!("The optimizer algorithm is not supported; the default algorithm 'freerouting-optimizer' will be used instead.");
        settings.optimizer.algorithm = Some("freerouting-optimizer".to_string());
    }
    let mut autorouter = BatchAutorouter::for_job(board, settings);
    let mut outcome = PipelineOutcome::default();

    // runRoutingStage
    let routing_start = std::time::Instant::now();
    let router_enabled = settings.get_run_router() && settings.autorouter.max_passes.map(|m| m >= 0).unwrap_or(true);
    if router_enabled && !ctx.stop.is_stop_autorouter_requested() {
        let unrouted_board = (ctx.enhancements && ctx.multi_start > 1).then(|| board.clone());
        let routing_start = std::time::Instant::now();
        autorouter.run_batch_loop(board, settings, ctx);
        if let Some(start) = unrouted_board {
            // the variants route with sequential passes: after a parallel first run they take
            // several times as long
            let took = routing_start.elapsed();
            let variant_estimate = if autorouter.pass_threads > 1 { took * 3 } else { took };
            if variant_estimate > MULTI_START_MAX_FIRST_RUN {
                log::info!(
                    "Multi-start skipped: the first routing run took {:.0} s (variants run only if they are expected to take less than {} s).",
                    took.as_secs_f64(),
                    MULTI_START_MAX_FIRST_RUN.as_secs()
                );
            } else {
                ctx.observe(board, &LiveEvent::Stage("multi-start"));
                multi_start(board, &start, settings, ctx);
            }
        }
    } else if settings.is_fanout_enabled() && !ctx.stop.is_stop_autorouter_requested() {
        let original = settings.autorouter.max_passes;
        settings.autorouter.max_passes = Some(0);
        autorouter.run_batch_loop(board, settings, ctx);
        settings.autorouter.max_passes = original;
    }
    outcome.fanout_timed_out = autorouter.fanout_timed_out;
    board.finish_autoroute();
    if ctx.enhancements {
        // The autorouter's own stop rules (stagnation, max passes) request a stop of the
        // autorouter; in Java that request stays set, so every optimizer candidate fails to
        // route and the optimizer gives up after its failure limit.
        ctx.stop.clear_stop_autorouter();
    }
    if let Some(options) = ctx.repair {
        if router_enabled && !ctx.stop.is_stop_requested() {
            ctx.observe(board, &LiveEvent::Stage("repair"));
            repair::repair_unrouted(board, settings, ctx, options);
            ctx.stop.clear_stop_autorouter();
        }
    }

    // runOptimizationStage
    if run_optimizer && !ctx.stop.is_stop_requested() {
        ctx.observe(board, &LiveEvent::Stage("optimizer"));
        let mut optimizer = BatchOptimizer::new(ctx.optimizer_mode);
        if ctx.enhancements {
            // without an explicit optimizer.timeout the optimizer gets as long as routing took
            // (at least a minute): on large boards a pass can take a quarter of an hour
            optimizer.default_budget = Some(routing_start.elapsed().max(std::time::Duration::from_secs(60)));
        }
        let mut stats = StatsCache::new();
        stats.unclamped_optimizer_score = ctx.enhancements;
        optimizer.run_batch_loop(board, settings, ctx, &mut stats);
        outcome.optimizer_timed_out = optimizer.timed_out;
    }
    outcome
}

/// fastroute multi-start: reruns the routing stage from `start` with `ctx.multi_start - 1`
/// shuffled first-pass orders (in parallel) and keeps the best board (fewest unrouted
/// connections, then fewest clearance violations, then highest router score; ties keep the
/// earlier variant, so the result does not depend on thread timing).
fn multi_start(board: &mut RoutingBoard, start: &RoutingBoard, settings: &RouterSettings, ctx: &PipelineContext) {
    use rayon::prelude::*;
    let mut stats = StatsCache::new();
    let first = stats.score(board, settings);
    // connections of ignored net classes are never routed: no variant would do better
    if first.incomplete_count - stats::ignored_incomplete_count(board) <= 0 || ctx.stop.is_stop_requested() {
        return;
    }
    let variants: Vec<(usize, RoutingBoard, stats::Score)> = (1..ctx.multi_start)
        .into_par_iter()
        .map(|v| {
            let mut b = start.clone();
            let variant_ctx = PipelineContext {
                // own autorouter stop (stagnation), shared job stop (time limit, Ctrl+C)
                stop: ctx.stop.child(),
                wall_clock_limits: ctx.wall_clock_limits,
                optimizer_mode: ctx.optimizer_mode,
                enhancements: true,
                multi_start: 1,
                cost_portfolio: false,
                repair: None,
                checkpoint: ctx.checkpoint.clone(),
                // the variants run in parallel; the viewer shows the first run
                observer: None,
            };
            let variant_settings = if ctx.cost_portfolio { portfolio_settings(settings, v) } else { settings.clone() };
            let mut router = BatchAutorouter::for_job(&b, &variant_settings);
            router.order_seed = Some(0x5eed_0000 + v as i64);
            router.pass_threads = 1; // the variants already run in parallel
            router.run_batch_loop(&mut b, &variant_settings, &variant_ctx);
            b.finish_autoroute();
            let s = StatsCache::new().score(&b, settings);
            (v, b, s)
        })
        .collect();
    let key = |s: &stats::Score| (s.incomplete_count, s.clearance_violation_count, std::cmp::Reverse(ordered_f32(s.router_score)));
    let mut best: Option<(usize, RoutingBoard, stats::Score)> = None;
    for (v, b, s) in variants {
        log::info!(
            "Multi-start variant {v}{}: {} unrouted, {} violations, router score {:.2}.",
            if ctx.cost_portfolio {
                let (via, ripup) = PORTFOLIO[(v - 1) % PORTFOLIO.len()];
                format!(" (via costs x{via}, rip-up costs x{ripup})")
            } else {
                String::new()
            },
            s.incomplete_count,
            s.clearance_violation_count,
            s.router_score as f64
        );
        let better = match &best {
            None => key(&s) < key(&first),
            Some((_, _, bs)) => key(&s) < key(bs),
        };
        if better {
            best = Some((v, b, s));
        }
    }
    if let Some((v, b, s)) = best {
        log::info!(
            "Multi-start: variant {v} is better ({} unrouted, {} violations; first run {} unrouted, {} violations).",
            s.incomplete_count,
            s.clearance_violation_count,
            first.incomplete_count,
            first.clearance_violation_count
        );
        *board = b;
    }
}

/// Total order on finite scores (NaN sorts last).
fn ordered_f32(x: f32) -> i64 {
    if x.is_nan() {
        i64::MIN
    } else {
        (x as f64 * 1000.0).round() as i64
    }
}

/// Java `HeadlessBoardManager.scheduleDeferredPostLoadProcessing`: the board part (awaited by
/// `RoutingPipeline.run` through `awaitPostLoad`): the pre-existing and unfixable clearance
/// violation counts of the loaded board.
pub fn deferred_post_load_processing(board: &mut RoutingBoard) {
    let violations = crate::drc::all_clearance_violations(board);
    board.pre_existing_clearance_violations_count = violations.len() as i32;
    let unfixable = violations.iter().filter(|v| v.is_unfixable(board)).count() as i32;
    board.unfixable_clearance_violations_count = unfixable;
    if !violations.is_empty() {
        log::warn!(
            "Board has {} pre-existing clearance violation(s) in the loaded design ({} unfixable).",
            violations.len(),
            unfixable
        );
    }
}
