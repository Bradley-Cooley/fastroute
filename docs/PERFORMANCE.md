# Performance work (result-neutral)

Goal: make `fastroute` faster **without changing any result**. Every step below keeps the SES
output byte-identical to the Java parity build (`fastroute --parity`) and keeps the parallel
optimizer mode deterministic and identical to its previous output.

## Methodology

* Machine: M-series Mac, 10 cores. Release build (`lto = "thin"`, `codegen-units = 1`).
* Profiling: `samply record --save-only --unstable-presymbolicate -o prof.json.gz -- <cmd>`,
  then `python3 scripts/prof-top.py prof.json.gz 40` (self + inclusive).
* Timings in the step log: single runs, wall clock (`/usr/bin/time`), machine otherwise idle
  unless noted; per-step numbers are indicative (±2 %). The final table is a serial run of all
  configurations with the pre-change binary and the final binary back to back.
* Parity gate after every step:
  * `cargo test --release --workspace` (includes `crates/fastroute/tests/pipeline_parity.rs`
    and the Java replay vector tests);
  * full pipeline `--parity` on bm02, bm06, bm07, bm08, bm09, bm11, pic_programmer, ecc83-pp,
    sonde xilinx, interf_u against `reference/parity-baseline/<board>.ses`;
  * autorouter only (`--router.optimizer.enabled=false`) on CM5_MINIMA_3 (`-mp 3`) and bm04
    against the pre-change binary, which was verified to be identical to the Java parity build
    for exactly these flags;
  * parallel optimizer (`--optimizer-mode=parallel --router.optimizer.max_threads=1` and `=8`)
    on bm11 and pic_programmer: 1 and 8 threads identical, and identical to the pre-change
    output;
  * the final binary additionally reproduces `reference/parity-baseline` on all 20 boards.
* `FASTROUTE_VERIFY_CACHES=1` (new): every memoized / indexed result is recomputed the old way
  and compared (contacts cache, per-net incomplete counts, indexed overlap queries against the
  Java traversal, leaf masks against the current shape layers). The whole gate passes in this
  mode.

## Optimizations (in the order applied)

| # | Change | Where | Measured effect |
|---|---|---|---|
| 1 | `Simplex` intersection tests without building the intersection: sort line *indices* with the ported TimSort and run the (unchanged) redundant-line algorithm on stack buffers; no `Line` clones (each clone went through `OnceLock` initialization) | `fr-geom/simplex.rs` (`redundant_lines_core`, `intersection_nonempty`), `java_sort::sort_indices_by` | CM5 AR `-mp 3`: 30.7 s → 25.1 s |
| 2 | Canonical sort of `overlaps` results by a precomputed `u64` key (kind, descending id, shape index) with `sort_unstable`; falls back to the comparator sort in traversal order if ids span ≥ 2^31, a shape index is negative or keys are not unique | `search_tree.rs` `sort_leaves_canonical` | CM5: 25.1 → 22.7 s |
| 3 | Memoized normal contacts: per-thread cache keyed by content epochs of the item repository and the default tree's item entries (epochs are process-wide unique numbers renewed by every `&mut` access, so clones share them only while their content is equal) | `board/epoch.rs`, `connectivity.rs`, `item_list.rs`, `search_tree.rs` | CM5: 22.7 → 13.7 s (fanout 18.9 s → 1.8 s) |
| 4 | Secondary spatial index for the sorted `overlaps` queries: hierarchical uniform grid over the leaf x/y ranges (flat arrays, cheap to clone), maintained on leaf insert/remove; the exact same leaf test is applied | `datastructures/leaf_grid.rs`, `MinAreaTree::overlapping_leaves_indexed` | CM5: 13.7 → 13.4 s, interf_u AR 8.9 → 8.2 s, bm04 AR 43.5 → 41.5 s |
| 5 | Layer masks in the search tree: each leaf carries the bit of its shape layer, inner nodes the union; queries that drop other layers (overlap queries with `layer >= 0`, the three `completeShape` variants) skip subtrees without leaves on that layer. The 45/90 degree `completeShape` walks see exactly the same sequence of relevant leaves (skipped leaves are ignored by the loop and never change the query) | `min_area_tree.rs` (`mask`, `next_leaf_masked`), `search_tree.rs`, `complete_shape.rs` | CM5: 13.4 → 10.5 s (only 7–17 % of the leaves visited by `completeShape` are on the room layer) |
| 6 | Shove checks compute only the needed tree shape of a substitute trace instead of all of them per index (was O(n²)); `IntOctagon::to_simplex` without the intermediate simplex, plus a small per-thread memo (Java memorizes it in the octagon) | `trace_shover.rs`, `int_octagon.rs` | bm04 AR: ~33 → 27.8 s |
| 7 | `Line` direction cache in atomics instead of `OnceLock` (cheap init and clone) | `fr-geom/line.rs` | ~1 %, removes `Once::call` (3–5 %) from profiles |
| 8 | One grid per layer (`LayeredGrid`) | `leaf_grid.rs` | bm04 `-mp 3`: 10.35 → 9.80 s user, CM5 `-mp 3`: 9.64 → 9.21 s |
| 9 | Compact structure-of-arrays copy of the node data for the traversals, specialized box/octagon tests (trees are homogeneous) | `min_area_tree.rs` (`HotNode`, `walk`) | ~1 % |
| 10 | mimalloc as global allocator of the CLI (**note:** the `#[global_allocator]` line was lost before the round-1 commit, so the round-1 results below were measured with the system allocator; restored in round 2, step R2.1) | `fastroute` | interf_u full: 17.3 → 15.7 s, bm04 full: 25.5 → 24.1 s |
| 11 | Per-net memo of the incomplete counts used by the pipeline (`calculateIncompleteCount`, called before and after every optimizer candidate), keyed by the keys and content versions of the net's items | `pipeline/stats.rs`, `item_list.rs` (`version`) | `NetIncompletes` 8.5 % → 4 % of bm11 full; ~1.5 % wall |
| 12 | `Simplex ∩ IntOctagon` uses the memoized octagon simplex; small per-thread memo for `Simplex::enlarge` (keyed by line-array identity, the memo holds the `Arc`) | `simplex.rs` | ~2 % |

Tried and rejected / not done:
* Single uniform grid with a "big leaves" list: slower than the tree (planes decompose into
  many large tiles) → replaced by the hierarchical grid.
* Replacing the maze `TreeSet` by a heap: the tie-break of the Java comparator uses door ids
  that change while elements are queued (incomplete rooms become complete), so the Java tree
  order cannot be reproduced by a heap.
* Memoizing shove / forced-via checks: they call count-mode `TimeLimit`s, whose call counts
  decide results in parity mode.

## Results (round 1)

Serial runs, pre-change binary vs final binary (wall clock seconds, peak RSS). `full` =
`fastroute --parity` (fanout, autorouter, java-compat optimizer); `ar` = autorouter only
(`--router.optimizer.enabled=false`, CM5 with `-mp 3`); `parN` = `--parity
--optimizer-mode=parallel --router.optimizer.max_threads=N`. Java parity = the Java parity build
(`reference/parity-baseline/summary.tsv`, same machine, JVM start-up included). All 20 full
runs reproduce `reference/parity-baseline/<board>.ses` byte for byte; the `ar`/`par` runs are
identical to the pre-change binary (and `par1` == `par8`).

| config | board | Java parity (s) | before (s) | after (s) | speed-up | RSS before/after (MB) | SES |
|---|---|---|---|---|---|---|---|
| full | DAC2020_bm01 | 959.59 | 687.49 | 458.88 | 1.50x | 366/378 | identical to Java |
| full | DAC2020_bm02 | 7.21 | 0.88 | 0.64 | 1.38x | 17/22 | identical to Java |
| full | DAC2020_bm04 | 137.82 | 49.22 | 25.59 | 1.92x | 552/342 | identical to Java |
| full | DAC2020_bm05 | 36.90 | 20.74 | 15.16 | 1.37x | 54/66 | identical to Java |
| full | DAC2020_bm06 | 28.31 | 9.62 | 6.35 | 1.51x | 26/32 | identical to Java |
| full | DAC2020_bm07 | 15.70 | 3.47 | 2.40 | 1.45x | 21/28 | identical to Java |
| full | DAC2020_bm08 | 3.58 | 0.14 | 0.09 | 1.56x | 7/8 | identical to Java |
| full | DAC2020_bm09 | 6.13 | 0.86 | 0.45 | 1.91x | 18/21 | identical to Java |
| full | DAC2020_bm10 | 112.81 | 34.60 | 17.06 | 2.03x | 240/316 | identical to Java |
| full | DAC2020_bm11 | 21.80 | 6.46 | 3.44 | 1.88x | 80/92 | identical to Java |
| full | CM5_MINIMA_3 | 409.82 | 65.68 | 26.70 | 2.46x | 344/539 | identical to Java |
| full | StickHub | 58.06 | 17.10 | 9.82 | 1.74x | 167/202 | identical to Java |
| full | complex_hierarchy | 10.49 | 1.08 | 0.88 | 1.23x | 42/60 | identical to Java |
| full | ecc83-pp | 3.26 | 0.01 | 0.00 | — | 6/6 | identical to Java |
| full | ecc83-pp_v2 | 3.17 | 0.01 | 0.01 | — | 6/7 | identical to Java |
| full | interf_u | 97.50 | 25.09 | 17.08 | 1.47x | 164/212 | identical to Java |
| full | multichannel_mixer-unrouted | 109.61 | 18.29 | 15.13 | 1.21x | 413/422 | identical to Java |
| full | multichannel_mixer | 69.71 | 10.29 | 8.30 | 1.24x | 521/540 | identical to Java |
| full | pic_programmer | 4.70 | 0.18 | 0.14 | 1.29x | 13/17 | identical to Java |
| full | sonde xilinx | 6.19 | 0.39 | 0.24 | 1.63x | 13/18 | identical to Java |
| ar | CM5_MINIMA_3 |  | 30.85 | 9.22 | 3.35x | 298/339 | identical to pre-change |
| ar | DAC2020_bm04 |  | 49.13 | 25.77 | 1.91x | 488/343 | identical to pre-change |
| ar | interf_u |  | 9.43 | 7.00 | 1.35x | 154/201 | identical to pre-change |
| par1 | DAC2020_bm11 |  | 6.24 | 3.17 | 1.97x | 70/83 | identical to pre-change |
| par1 | interf_u |  | 24.83 | 17.07 | 1.45x | 163/211 | identical to pre-change |
| par1 | DAC2020_bm06 |  | 9.50 | 6.28 | 1.51x | 27/34 | identical to pre-change |
| par8 | DAC2020_bm11 |  | 2.54 | 1.50 | 1.69x | 136/179 | identical to pre-change |
| par8 | interf_u |  | 13.56 | 9.74 | 1.39x | 251/345 | identical to pre-change |
| par8 | DAC2020_bm06 |  | 4.02 | 2.89 | 1.39x | 80/123 | identical to pre-change |
| full | **sum of 20 boards** | 2102.4 | 951.6 | 608.4 | 1.56x | | |

Peak RSS grows by 10–30 % on most boards and by 57 % on CM5_MINIMA_3 (mimalloc, the grid index,
the compact node copy, the per-thread caches); bm04 uses less.


## Round 2

Same rules and gate as round 1 (plus parallel mode checked with 1, 2, 3, 5 and 8 threads).
Per-step effect measured as user seconds of four runs: bm05 full / interf_u full / CM5
autorouter `-mp 3` / bm04 autorouter (all byte-identical to their references).

| # | Change | Where | Measured effect |
|---|---|---|---|
| R2.1 | Restored the mimalloc `#[global_allocator]` (lost in round 1) | `fastroute/src/main.rs` | bm05 full 15.22 → 13.78 s, interf_u full 17.15 → 15.91 s (wall) |
| R2.2 | Intersection of two simplices: the two line arrays are already sorted, so Java's TimSort of the concatenation equals the stable merge of the two runs whenever `Line.compareTo` is exact and consistent (non-zero directions with components < 2^26: the `f64` determinant is exact, the comparator is the angle order). Checked per call, otherwise TimSort; unit test against TimSort | `fr-geom/simplex.rs` (`merge_sorted_runs`) | 13.57/15.30/8.49/23.45 → 13.26/15.11/8.25/22.79 |
| R2.3 | Cloning a `MinAreaTree` no longer copies the grid index (rebuilt lazily; board snapshots and deep copies are rarely queried) | `min_area_tree.rs` | CM5 full peak RSS 361 → 325 MB (superseded: the grid is now shared, see "Memory in parallel mode") |
| R2.4 | Parallel optimizer: candidates are claimed in order by the workers (shared counter) and results consumed strictly in candidate order as soon as available, instead of fixed chunks with a barrier; nothing after the stop point is consumed, so the result is unchanged for any thread count | `pipeline/optimizer.rs` (`stream_parallel`, `PassConsumer`) | 8 threads: interf_u 9.74 → 7.93 s, bm06 2.89 → 1.65 s, bm11 1.50 → 1.46 s (wall) |
| R2.5 | 45-degree `completeShape`/`restrainShape` on plain `(IntOctagon, IntOctagon)` pairs with two reused buffers (no `TileShape`/`Vec` per obstacle and room); the bounding union only adds the new rooms (the component-wise union is idempotent) | `complete_shape.rs` | 13.26/15.18/8.26/22.76 → 12.99/14.68/8.11/22.57 |
| R2.6 | Redundant-line removal reads each line's int direction once into a stack array | `simplex.rs` | → 12.89/14.65/8.05/22.39 |
| R2.7 | `Simplex::get_instance` on index arrays (clones only the kept lines); octagon→simplex memo slot hash fixed (used only 256 of its slots) and enlarged to 1024 | `simplex.rs`, `int_octagon.rs` | → 12.70/14.46/7.99/22.02 |
| R2.8 | Grid: skip levels without leaves | `leaf_grid.rs` | → 12.63/14.40/7.99/21.95 |
| R2.9 | Profile-guided optimization: `scripts/build-pgo.sh` (instrumented build, training on bm06, bm11 java-compat + parallel, interf_u / CM5 / multichannel_mixer autorouter, `llvm-profdata merge`, optimized build in `target/pgo/use`); `--native` adds `-C target-cpu=native` (measured: no further gain on M-series) | `scripts/build-pgo.sh` | 12.80/14.54/8.03/22.03 → 10.46/12.46/7.06/18.39 (−13…−18 %) |

Tried without measurable gain (reverted / not applied): grid cell density and level span
tuning (±1 % across 5 settings), computing the maze key's door id once per `TreeSet` insertion,
mimalloc purge / eager-commit options (RSS unchanged). Optimizer-specific overheads outside
routing (incomplete counts, deep copies, statistics) are now ≈ 2–3 % of a candidate; bm01's
time is routing (pass 2 with increased ripup costs takes 4× pass 1).

The 45-degree `completeShape` walk must stay order-exact; its per-node test is already the
compact form. A result-neutral alternative exists in principle (the query only shrinks, so the
walk equals a scan of the leaves in fixed traversal order), but it needs the traversal rank of
leaves in a tree that changes between calls, which costs more than the walk saves.

### Results (round 2)

Serial runs (wall clock seconds). `round 1` = commit 6044d65 as built (system allocator, see
R2.1); `round 2` = this round, normal release build; `+ PGO` = `scripts/build-pgo.sh`. All
`full` runs reproduce `reference/parity-baseline/<board>.ses`; `ar`/`par` runs are identical to
the pre-round-1 binary, `par1` == `par8`. Java parity from `reference/parity-baseline/summary.tsv`.

| config | board | Java parity | pre-round-1 | round 1 (6044d65) | round 2 | round 2 + PGO | vs Java (PGO) | RSS MB r1 / r2 / PGO |
|---|---|---|---|---|---|---|---|---|
| full | DAC2020_bm01 | 959.59 | 687.49 | 458.88 | 380.34 | 310.27 | 3.1x | 378 / 300 / 296 |
| full | DAC2020_bm02 | 7.21 | 0.88 | 0.64 | 0.54 | 0.45 | 16.0x | 22 / 18 / 18 |
| full | DAC2020_bm04 | 137.82 | 49.22 | 25.59 | 22.05 | 18.40 | 7.5x | 342 / 201 / 202 |
| full | DAC2020_bm05 | 36.90 | 20.74 | 15.16 | 12.72 | 10.47 | 3.5x | 66 / 77 / 81 |
| full | DAC2020_bm06 | 28.31 | 9.62 | 6.35 | 5.40 | 4.41 | 6.4x | 32 / 51 / 51 |
| full | DAC2020_bm07 | 15.70 | 3.47 | 2.40 | 2.02 | 1.66 | 9.5x | 28 / 30 / 31 |
| full | DAC2020_bm08 | 3.58 | 0.14 | 0.09 | 0.07 | 0.06 | 59.7x | 8 / 6 / 7 |
| full | DAC2020_bm09 | 6.13 | 0.86 | 0.45 | 0.41 | 0.37 | 16.6x | 21 / 26 / 27 |
| full | DAC2020_bm10 | 112.81 | 34.60 | 17.06 | 14.78 | 12.35 | 9.1x | 316 / 169 / 145 |
| full | DAC2020_bm11 | 21.80 | 6.46 | 3.44 | 2.98 | 2.46 | 8.9x | 92 / 84 / 82 |
| full | CM5_MINIMA_3 | 409.82 | 65.68 | 26.70 | 23.05 | 20.14 | 20.3x | 539 / 309 / 314 |
| full | StickHub | 58.06 | 17.10 | 9.82 | 8.33 | 7.04 | 8.2x | 202 / 207 / 205 |
| full | complex_hierarchy | 10.49 | 1.08 | 0.88 | 0.74 | 0.64 | 16.4x | 60 / 62 / 61 |
| full | ecc83-pp | 3.26 | 0.01 | 0.00 | 0.00 | 0.00 |  | 6 / 6 / 6 |
| full | ecc83-pp_v2 | 3.17 | 0.01 | 0.01 | 0.00 | 0.00 |  | 7 / 6 / 7 |
| full | interf_u | 97.50 | 25.09 | 17.08 | 14.54 | 12.42 | 7.9x | 212 / 192 / 194 |
| full | multichannel_mixer-unrouted | 109.61 | 18.29 | 15.13 | 13.66 | 12.18 | 9.0x | 422 / 98 / 101 |
| full | multichannel_mixer | 69.71 | 10.29 | 8.30 | 7.51 | 6.78 | 10.3x | 540 / 120 / 114 |
| full | pic_programmer | 4.70 | 0.18 | 0.14 | 0.12 | 0.10 | 47.0x | 17 / 16 / 16 |
| full | sonde xilinx | 6.19 | 0.39 | 0.24 | 0.20 | 0.17 | 36.4x | 18 / 15 / 15 |
| ar | CM5_MINIMA_3 |  | 30.85 | 9.22 | 8.04 | 7.06 |  | 339 / 209 / 186 |
| ar | DAC2020_bm04 |  | 49.13 | 25.77 | 22.08 | 18.41 |  | 343 / 202 / 201 |
| ar | interf_u |  | 9.43 | 7.00 | 5.94 | 5.08 |  | 201 / 186 / 188 |
| par1 | DAC2020_bm11 |  | 6.24 | 3.17 | 2.63 | 2.16 |  | 83 / 74 / 73 |
| par1 | interf_u |  | 24.83 | 17.07 | 14.42 | 12.27 |  | 211 / 191 / 188 |
| par1 | DAC2020_bm06 |  | 9.50 | 6.28 | 5.31 | 4.36 |  | 34 / 71 / 74 |
| par8 | DAC2020_bm11 |  | 2.54 | 1.50 | 1.43 | 1.22 |  | 179 / 347 / 356 |
| par8 | interf_u |  | 13.56 | 9.74 | 7.70 | 6.62 |  | 345 / 535 / 527 |
| par8 | DAC2020_bm06 |  | 4.02 | 2.89 | 1.60 | 1.34 |  | 123 / 234 / 269 |
| full | **sum of 20 boards** | 2102.4 | 951.6 | 608.4 | 509.5 | 420.4 | 5.0x | |

Memory: the full and autorouter runs use less memory than in round 1 (CM5 539 → 309 MB,
bm04 342 → 201 MB, multichannel_mixer 540 → 120 MB). The 8-thread parallel runs use more
(interf_u 345 → 535 MB): mimalloc keeps per-thread heaps for the 8 concurrent board clones
(chunked evaluation with mimalloc: 516 MB, so it is the allocator, not the streaming).

## Memory in parallel mode

The default (parallel) mode used far more memory than the exact mode: 2.6 GB for the routing
stage and 4.1 GB in total on an 874-part 6-layer board, 3.7–4.8 GB on lora_node and some
PCBench boards. Measured with `--report` peak RSS, a sampling RSS trace per stage, dhat
(`cargo build --features dhat-heap`, writes `dhat-heap.json`) and a counting allocator:

* A board clone was a deep copy (37 MB on the 874-part board: item slots 9 MB, three search
  trees 7–13 MB each). The parallel pass took two per connection (the batch-start board and the
  worker board) and kept both until the result was committed in pass order: 18 connections in
  flight × 2 × 37 MB, the +1.3 GB jump within a second of the pass start.
* A maze search kept every element it ever queued; searches over the whole board (failing
  connections) reached ~88 MB each, on every worker thread.
* Every clone rebuilt the grid index of its trees on its first query.
* The parallel optimizer consumes candidate results in candidate order; the results evaluated
  ahead wait in a buffer, improved ones with their whole board. Behind one slow candidate,
  hundreds of boards piled up: 1.77 GB live (2.9 GB RSS) in one pass on the 265-part gs board.
* The rest is mimalloc's per-thread retention (peak live heap 332 MB vs. 811 MB RSS in the
  lora_node optimizer); its purge options did not change the peak.

Changes (results unchanged: byte-identical SES with and without `--parity`, all 20 parity
boards, full pipeline with the optimizer):

| Change | Where |
|---|---|
| A worker keeps only its changes and copies of the items involved; the connections dispatched before the board changes share one snapshot | `parallel_pass.rs` (`Condensed`), `autorouter.rs` |
| Item slots, tree nodes and the per-item tree info in `CowVec` (chunks of 64 behind `Arc`s, copied on write): a clone costs ~1–3 MB instead of 37 MB | `datastructures/cow_vec.rs`, `item_list.rs`, `min_area_tree.rs`, `search_tree.rs` |
| The traversal array (`hot`) and the grid index shared whole behind an `Arc` (copied by the first change of a clone): clones keep the grid instead of rebuilding it, and the hottest reads stay plain slice reads | `min_area_tree.rs` |
| Maze element slots reused once taken from the queue (the comparator does not depend on the index) | `maze.rs` |
| Optimizer: a buffered result keeps its board only while it can still become the pass winner (better than the current winner; `improved_over` is a strict total order and the winner only improves). A window limiting how far workers run ahead saved slightly more memory but made the pass up to 60 % slower | `optimizer.rs` (`stream_parallel`) |

Peak RSS before → after (routing stage `-mp 3`, optimizer off; full pipeline where noted):

| Board | Before | After |
|---|---|---|
| 874-part 6-layer board | 2593 MB (69.6 s) | 1103 MB (61.8 s) |
| CM5_MINIMA_3 | 851 MB | 445 MB |
| DAC2020_bm04 | 569 MB | 274 MB |
| interf_u | 445 MB | 189 MB |
| interf_u, full pipeline | 880 MB | 511 MB |
| DAC2020_bm11, full pipeline | 805 MB | 446 MB |
| gs (265 parts), optimizer from a routed session | 2947 MB | 1218 MB (pass time unchanged) |

Over the 92 boards of `scripts/bench.py --suite dac,kicad,dir:...,pcbench:60` (before the
optimizer change): sum of peak RSS 57.3 → 39.1 GB, total time 2888 → 2682 s, same unrouted
and violation totals.

Time: parallel mode unchanged or faster; exact mode (single thread) +3–4 % (CM5 23.8 → 24.5 s,
interf_u 14.8 → 15.4 s) from the chunked node and slot reads. Storing the traversal array and
the grid chunked as well cost +9–14 %, which is why they are shared whole.

## Round 3

Measured on a 4-core Linux machine. Gate: Aria, S1G_Mod_868, PSLab and
Hardware_Playground_minimal (PCBench fixtures) routed with `--router.optimizer.enabled=false
--router.autorouter.max_threads=1` by the pre-change and the new binary, byte-identical SES in
three configurations: default, `--parity`, and the multi-start stack (`--multi-start=4
--escape-plan --multi-start-costs --failure-history --prune-variants`); `cargo test --release
--workspace` unchanged.

Profiling: `cargo build --release --features cpu-profile -p fastroute`, then
`FASTROUTE_CPU_PROFILE=out.folded fastroute ...` writes folded stacks (`thread;outer;...;inner
count`, SIGPROF sampling at 1 kHz, so it works where perf events are not allowed, e.g. in
containers). Any flame graph tool reads the format. Aria, one routing run, one thread: maze
search 84 % (room completion 51 %, of which the 45-degree `completeShape` walk 30 %; drill
rooms 34 %), the `Arc::get_mut` atomics of `CowVec` writes ≈ 4.5 %.

| # | Change | Where | Measured effect |
|---|---|---|---|
| R3.1 | 45-degree `completeShape`: an obstacle the tree walk returns (it overlaps the bounding shape) that overlaps none of the rooms changes nothing, so it is skipped before the restrain step | `complete_shape.rs` | ≈ 1 % (single run) |
| R3.2 | `CowVec` writes use `Arc::make_mut` (one compare-exchange) instead of `Arc::get_mut` twice | `cow_vec.rs` | ≈ 2–3 % (single run) |
| R3.3 | Multi-start: the fanout depends only on the start board, the settings and the skipped pins, so the variants with the first run's settings reuse its fanout board (one shared fanout per escape plan); only the cost portfolio variants compute their own | `pipeline/mod.rs`, `pipeline/autorouter.rs` (`FanoutCache`) | Aria multi-start stack 230 → 205 s (fanout 15 → 5 runs) |

Single run R3.1 + R3.2: Aria 117.7 / 118.8 → 115.4 s, S1G 55.4 / 56.3 → 54.0 s; `--parity`
unchanged within noise.

## Next ideas

* 45-degree `completeShape` tree walk (~9 % self) and grid scans (~6 %) remain the largest
  single items; everything else is ≤ 5 %.
* Maze `TreeSet`: door ids are recomputed through room lookups on every comparison; caching
  them needs invalidation whenever a room is completed.
* Parallel mode: the remaining gap between live heap and RSS is mimalloc's per-thread
  retention (e.g. `mi_collect` after each candidate, if memory matters more than ~2 % speed);
  multi-start variants still hold one board and routing state each.
* Make PGO the default release pipeline (CI step running `scripts/build-pgo.sh`).
