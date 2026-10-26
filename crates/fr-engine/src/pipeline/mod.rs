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
pub mod escape;
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
use crate::autoroute::history::FailureHistory;
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
    /// fastroute cost portfolio (`--multi-start-costs`): `multi_start - 1` more variants that
    /// also scale the via and rip-up costs (see [`PORTFOLIO`]), run next to the stock ones.
    pub cost_portfolio: bool,
    /// fastroute escape planner (`--escape-plan`): see [`EscapeMode`].
    pub escape: EscapeMode,
    /// fastroute failure-only history (`--failure-history`): see [`HistoryMode`].
    pub failure_history: HistoryMode,
    /// fastroute (`--prune-variants`): stop the extra multi-start variants (cost portfolio,
    /// escape plan) that fall clearly behind the first run (see [`autorouter::PruneRule`]).
    pub prune_variants: bool,
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
            .field("escape", &self.escape)
            .field("failure_history", &self.failure_history)
            .field("prune_variants", &self.prune_variants)
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
            escape: EscapeMode::Off,
            failure_history: HistoryMode::Off,
            prune_variants: false,
            repair: None,
            checkpoint: None,
            observer: None,
        }
    }
}

/// fastroute escape planner modes (see [`escape::plan_escapes`]).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum EscapeMode {
    #[default]
    Off,
    /// `multi_start` more multi-start variants (the first-run order and the stock orders)
    /// route with the escape plan, next to the stock ones: the best board can only get better.
    Variants,
    /// The first run and the stock variants route with the escape plan (to measure it).
    Only,
}

/// fastroute failure-only history modes (see [`crate::autoroute::history`]).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum HistoryMode {
    #[default]
    Off,
    /// `multi_start` more multi-start variants (the first-run order and the stock orders)
    /// route with the history, next to the stock ones: the best board can only get better.
    Variants,
    /// The first run and the stock variants route with the history (to measure it).
    Only,
}

/// Multi-start variants run in parallel but each about as long as the first run; after a
/// longer first run they are skipped (a large board then spends an hour more for a few
/// connections). The limit counts maze search steps (see
/// [`crate::autoroute::maze::SEARCH_STEPS`]) instead of seconds, so the decision does not
/// depend on the machine's load. Sequential routing runs at about 30 000-125 000 steps per
/// second on a cloud core (75 000-90 000 on the larger boards of the quick suite), so the
/// limit is roughly 10 minutes per variant; no board of the quick suite reaches it (SunLeaf,
/// the largest, takes about 13 million).
const MULTI_START_MAX_STEPS: u64 = 50_000_000;

/// fastroute cost portfolio: (via cost factor, rip-up cost factor) of cost variant `k`
/// (1-based, with the order seed of stock variant `k`), taken at `(k - 1) % len`. Cheap vias help boards that run out of room on one layer,
/// dear vias boards that run out of via sites; high rip-up costs keep routed nets in place,
/// low ones let new connections push through.
const PORTFOLIO: [(f64, f64); 6] = [(0.5, 1.0), (2.0, 1.0), (1.0, 3.0), (0.5, 3.0), (2.0, 0.5), (1.0, 0.5)];

/// The settings of cost variant `k` of the cost portfolio.
fn portfolio_settings(settings: &RouterSettings, k: usize) -> RouterSettings {
    let (via, ripup) = PORTFOLIO[(k - 1) % PORTFOLIO.len()];
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
    let escape_plan = (ctx.enhancements && ctx.escape != EscapeMode::Off).then(|| {
        let plan = escape::plan_escapes(board);
        log::info!(
            "Escape plan: {} dense part{}, {} pins with a target on another part, {} kept on their layer, {} given a via for crossing airlines, {} with a target on another layer only ({} on a dense part).",
            plan.dense_parts,
            if plan.dense_parts == 1 { "" } else { "s" },
            plan.planned,
            plan.keep_on_layer.len(),
            plan.via_for_crossings,
            plan.other_layer,
            plan.other_layer_dense
        );
        std::sync::Arc::new(plan.keep_on_layer)
    });
    if ctx.escape == EscapeMode::Only {
        autorouter.fanout_skip_pins = escape_plan.clone();
    }
    if ctx.enhancements && ctx.failure_history == HistoryMode::Only {
        autorouter.failure_history = Some(FailureHistory::new(board.communication.get_resolution(crate::structure::Unit::Um)));
        autorouter.history_in_costs = true;
    }

    // runRoutingStage
    let routing_start = std::time::Instant::now();
    let router_enabled = settings.get_run_router() && settings.autorouter.max_passes.map(|m| m >= 0).unwrap_or(true);
    if router_enabled && !ctx.stop.is_stop_autorouter_requested() {
        let unrouted_board = (ctx.enhancements && ctx.multi_start > 1).then(|| board.clone());
        let routing_start = std::time::Instant::now();
        let steps_before = crate::autoroute::maze::SEARCH_STEPS.load(std::sync::atomic::Ordering::Relaxed);
        autorouter.run_batch_loop(board, settings, ctx);
        let steps = crate::autoroute::maze::SEARCH_STEPS.load(std::sync::atomic::Ordering::Relaxed) - steps_before;
        log::debug!("First routing run: {} maze search steps in {:.1} s.", steps, routing_start.elapsed().as_secs_f64());
        if let Some(start) = unrouted_board {
            // decided by the maze search steps of the first run, not by its wall-clock time,
            // so the decision (and the result) does not depend on the machine's load
            if steps > MULTI_START_MAX_STEPS {
                log::info!(
                    "Multi-start skipped: the first routing run took {} maze search steps in {:.0} s (variants run only after at most {}).",
                    steps,
                    routing_start.elapsed().as_secs_f64(),
                    MULTI_START_MAX_STEPS
                );
            } else {
                ctx.observe(board, &LiveEvent::Stage("multi-start"));
                let history: Vec<i32> = autorouter.pass_history.iter().map(|h| h.0).collect();
                multi_start(board, &start, settings, ctx, escape_plan.as_ref(), history);
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
/// earlier variant, so the result does not depend on thread timing). The cost portfolio and
/// the escape planner add their variants after the stock ones.
fn multi_start(
    board: &mut RoutingBoard,
    start: &RoutingBoard,
    settings: &RouterSettings,
    ctx: &PipelineContext,
    escape_plan: Option<&std::sync::Arc<std::collections::BTreeSet<i32>>>,
    first_history: Vec<i32>,
) {
    use rayon::prelude::*;
    let mut stats = StatsCache::new();
    let first = stats.score(board, settings);
    // connections of ignored net classes are never routed: no variant would do better
    if first.incomplete_count - stats::ignored_incomplete_count(board) <= 0 || ctx.stop.is_stop_requested() {
        return;
    }
    struct Variant {
        label: String,
        /// first-pass order (`None`: the first run's)
        seed: Option<usize>,
        settings: RouterSettings,
        /// the pins the fanout leaves alone (an escape plan)
        skip: Option<std::sync::Arc<std::collections::BTreeSet<i32>>>,
        /// a stock variant (never pruned)
        stock: bool,
        /// routes with the failure history
        history: bool,
    }
    let n = ctx.multi_start;
    let only = ctx.escape == EscapeMode::Only && escape_plan.is_some_and(|p| !p.is_empty());
    let mut specs: Vec<Variant> = (1..n)
        .map(|v| Variant {
            label: if only { " (escape plan)".into() } else { String::new() },
            seed: Some(v),
            settings: settings.clone(),
            skip: if only { escape_plan.cloned() } else { None },
            stock: true,
            history: ctx.failure_history == HistoryMode::Only,
        })
        .collect();
    // The cost portfolio and the escape planner add their variants after the stock ones
    // instead of replacing them, so the stock result is always among the candidates and the
    // best one can only get better.
    if ctx.cost_portfolio {
        for k in 1..n {
            let (via, ripup) = PORTFOLIO[(k - 1) % PORTFOLIO.len()];
            specs.push(Variant {
                label: format!(" (order {k}, via costs x{via}, rip-up costs x{ripup})"),
                seed: Some(k),
                settings: portfolio_settings(settings, k),
                skip: None,
                stock: false,
                history: false,
            });
        }
    }
    if ctx.escape == EscapeMode::Variants && escape_plan.is_some() {
        // with an empty plan (no dense parts) the shuffled orders would repeat the stock
        // variants; only the first-run order (routed with sequential passes) differs
        let orders = if escape_plan.is_some_and(|p| p.is_empty()) { 1 } else { n };
        for k in 0..orders {
            let order = if k == 0 { "first-run order".to_string() } else { format!("order {k}") };
            specs.push(Variant {
                label: format!(" (escape plan, {order})"),
                seed: (k > 0).then_some(k),
                settings: settings.clone(),
                skip: escape_plan.cloned(),
                stock: false,
                history: false,
            });
        }
    }
    if ctx.failure_history == HistoryMode::Variants {
        for k in 0..n {
            let order = if k == 0 { "first-run order".to_string() } else { format!("order {k}") };
            specs.push(Variant {
                label: format!(" (failure history, {order})"),
                seed: (k > 0).then_some(k),
                settings: settings.clone(),
                skip: None,
                stock: false,
                history: true,
            });
        }
    }
    let prune = ctx.prune_variants.then(|| autorouter::PruneRule {
        reference: std::sync::Arc::new(first_history),
        min_pass: PRUNE_MIN_PASS,
        margin_pct: PRUNE_MARGIN_PCT,
        margin_min: PRUNE_MARGIN_MIN,
    });
    let variants: Vec<(usize, RoutingBoard, stats::Score, Vec<(i32, f64)>, Option<i32>)> = specs
        .par_iter()
        .enumerate()
        .map(|(i, spec)| {
            let mut b = start.clone();
            let variant_ctx = PipelineContext {
                // own autorouter stop (stagnation), shared job stop (time limit, Ctrl+C)
                stop: ctx.stop.child(),
                wall_clock_limits: ctx.wall_clock_limits,
                optimizer_mode: ctx.optimizer_mode,
                enhancements: true,
                multi_start: 1,
                cost_portfolio: false,
                escape: EscapeMode::Off,
                failure_history: HistoryMode::Off,
                prune_variants: false,
                repair: None,
                checkpoint: ctx.checkpoint.clone(),
                // the variants run in parallel; the viewer shows the first run
                observer: None,
            };
            let mut router = BatchAutorouter::for_job(&b, &spec.settings);
            router.order_seed = spec.seed.map(|s| 0x5eed_0000 + s as i64);
            router.fanout_skip_pins = spec.skip.clone();
            if spec.history {
                router.failure_history = Some(FailureHistory::new(start.communication.get_resolution(crate::structure::Unit::Um)));
                router.history_in_costs = true;
            }
            if !spec.stock {
                router.prune = prune.clone();
            }
            router.pass_threads = 1; // the variants already run in parallel
            router.run_batch_loop(&mut b, &spec.settings, &variant_ctx);
            b.finish_autoroute();
            let s = StatsCache::new().score(&b, settings);
            (i + 1, b, s, std::mem::take(&mut router.pass_history), router.pruned_at)
        })
        .collect();
    let key = |s: &stats::Score| (s.incomplete_count, s.clearance_violation_count, std::cmp::Reverse(ordered_f32(s.router_score)));
    let mut best: Option<(usize, RoutingBoard, stats::Score)> = None;
    for (v, b, s, h, pruned_at) in variants {
        log::info!(
            "Multi-start variant {v}{}: {} unrouted, {} violations, router score {:.2}{}; unrouted after each pass {:?} ({:.0} s).",
            specs[v - 1].label,
            s.incomplete_count,
            s.clearance_violation_count,
            s.router_score as f64,
            pruned_at.map(|p| format!(" (stopped after pass {p})")).unwrap_or_default(),
            h.iter().map(|x| x.0).collect::<Vec<_>>(),
            h.iter().map(|x| x.1).sum::<f64>()
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
            "Multi-start: variant {v}{} is better ({} unrouted, {} violations; first run {} unrouted, {} violations).",
            specs[v - 1].label,
            s.incomplete_count,
            s.clearance_violation_count,
            first.incomplete_count,
            first.clearance_violation_count
        );
        *board = b;
    }
}

/// [`autorouter::PruneRule`] of `--prune-variants`.
const PRUNE_MIN_PASS: usize = 3;
const PRUNE_MARGIN_PCT: i32 = 50;
const PRUNE_MARGIN_MIN: i32 = 3;

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
