#!/usr/bin/env python3
"""Routing quality benchmark: runs fastroute on a set of boards and tabulates the result.

Every run writes `--report` JSON (fastroute's own statistics), so the numbers are the
router's view of the board: unrouted connections, clearance violations, vias, trace length,
scores, time and peak memory. With `--baseline` the table is compared with an earlier run.

Usage:
  scripts/bench.py [--suite S[,S...]] [--out DIR] [-j N] [--baseline DIR/results.tsv]
                   [--diagnose] [--bin PATH] [--max-time SECONDS] [-- extra fastroute args]

Suites:
  dac          the 10 DAC2020 boards of Freerouting's benchmark
  kicad        the KiCad 10 demo boards of Freerouting's benchmark
  pcbench:N    N boards of the PCBench corpus (every k-th unrouted.dsn, deterministic)
  dir:PATH     directories below PATH holding a board exported with
               `route_cli.py --export-only --work-dir DIR` (board.dsn + args.txt): the board
               exactly as the KiCad plugin routes it
  FILE.dsn     a single board
  quick        16 boards that react to routing changes (unrouted connections or recent
               changes, each under ~100 s); run with -j 3, see QUICK_ARGS
  dense:N      the N PCBench boards (default 12) with the most fine-pitch parts (at least
               DENSE_MIN_PINS SMD pins at a pitch of at most DENSE_MAX_PITCH_UM, as
               fastroute's `pipeline::escape`), at most DENSE_MAX_COMPONENTS parts, not in
               quick; picked by scanning the DSN files; run with -j 3 (QUICK_ARGS)

The `escape` column counts the unrouted connections with an end at a fine-pitch part.

Default: dac,kicad (the 20 benchmark boards of docs/PERFORMANCE.md).
"""

import argparse
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
FIXTURES = ROOT / "reference/freerouting/scripts/benchmark/fixtures"

COLUMNS = [
    ("board", "{:<34}"),
    ("time_s", "{:>8}"),
    ("rss_mb", "{:>7}"),
    ("unrouted", "{:>8}"),
    ("escape", "{:>6}"),
    ("violations", "{:>10}"),
    ("vias", "{:>6}"),
    ("length_mm", "{:>10}"),
    ("router_score", "{:>12}"),
    ("optimizer_score", "{:>15}"),
    ("congestion", "{:>10}"),
    ("blocked", "{:>7}"),
    ("exit", "{:>4}"),
]


# boards of the quick suite: (suite directory or "pcbench", board name / PCBench directory prefix)
QUICK = [
    ("DAC2020_boards", "DAC2020_bm04"), ("DAC2020_boards", "DAC2020_bm05"),
    ("KiCad_10_demos", "CM5_MINIMA_3"), ("KiCad_10_demos", "StickHub"), ("KiCad_10_demos", "interf_u"),
    ("pcbench", "Aria_Aria"), ("pcbench", "HY-AI7688H-RevA"), ("pcbench", "Hardware_Playground_minimal"),
    ("pcbench", "Inkjet_PiezoDriver"), ("pcbench", "LED_port-status"), ("pcbench", "S1G-Mod_S1G_Mod_868"),
    ("pcbench", "SunLeaf_SunLeaf_V2"), ("pcbench", "USB-TypeC-breakout-board"), ("pcbench", "kitspace_PSLab"),
    ("pcbench", "mdbwerk_mdbwerk"), ("pcbench", "mightyduino_mightyduino"),
]
# the quick suite runs several boards at once: fewer threads per run (results depend on the
# thread count, so quick runs are only comparable with quick runs)
QUICK_ARGS = ["--router.autorouter.max_threads=3", "--router.optimizer.max_threads=3"]

# the dense suite: the same thresholds as fastroute's pipeline::escape
DENSE_MIN_PINS = 16
DENSE_MAX_PITCH_UM = 650.0
DENSE_MAX_COMPONENTS = 150

_TOKEN = r'("[^"]*"|[^\s()]+)'
_UNIT = re.compile(r"\(unit\s+(\w+)\)")
_IMAGE = re.compile(r"\(image\s+" + _TOKEN)
_PIN = re.compile(r"\(pin\s+" + _TOKEN + r"\s+(?:\(rotate[^)]*\)\s+)?" + _TOKEN + r"\s+(-?[\d.]+)\s+(-?[\d.]+)")
_PADSTACK = re.compile(r"\(padstack\s+" + _TOKEN)
_PLACE = re.compile(r"\(place\s+" + _TOKEN + r"\s+-?[\d.]")
_UM_PER = {"um": 1.0, "mm": 1000.0, "mil": 25.4, "inch": 25400.0, "cm": 10000.0}


def dense_part_count(dsn):
    """(number of placed fine-pitch parts, number of placed parts) of a DSN file.

    A part is fine-pitch with at least DENSE_MIN_PINS single-layer pins whose median
    nearest-neighbour distance is at most DENSE_MAX_PITCH_UM. Pins without a net count here
    (fastroute counts only pins with a net), so this is a slightly wider pick."""
    text = dsn.read_text(errors="replace")
    m = _UNIT.search(text)
    um = _UM_PER.get(m.group(1).lower(), 1.0) if m else 1.0
    # single-layer padstacks: one (shape ...) line
    smd = set()
    for m in _PADSTACK.finditer(text):
        end = text.find("(attach", m.end())
        block = text[m.end(): end if end > 0 else m.end() + 2000]
        if block.count("(shape") == 1:
            smd.add(m.group(1))
    dense_images = set()
    images = list(_IMAGE.finditer(text))
    for i, m in enumerate(images):
        block = text[m.end(): images[i + 1].start() if i + 1 < len(images) else len(text)]
        pts = [(float(p.group(3)), float(p.group(4))) for p in _PIN.finditer(block) if p.group(1) in smd]
        if len(pts) < DENSE_MIN_PINS:
            continue
        nearest = sorted(min(((x - a) ** 2 + (y - b) ** 2) ** 0.5 for j, (a, b) in enumerate(pts) if j != k)
                         for k, (x, y) in enumerate(pts))
        if nearest[len(nearest) // 2] * um <= DENSE_MAX_PITCH_UM:
            dense_images.add(m.group(1))
    # (component IMAGE (place REF ...) (place REF ...))
    dense = total = 0
    for m in re.finditer(r"\(component\s+" + _TOKEN, text):
        places = len(_PLACE.findall(text[m.end(): text.find("(component", m.end()) if text.find("(component", m.end()) > 0 else text.find("(library", m.end())]))
        total += places
        if m.group(1) in dense_images:
            dense += places
    return dense, total


class Board:
    def __init__(self, name, dsn, args=()):
        self.name, self.dsn, self.args = name, Path(dsn), list(args)


def suite_boards(spec):
    if spec == "quick":
        boards = []
        for suite, name in QUICK:
            if suite == "pcbench":
                d = sorted(p for p in (FIXTURES / "PCBench").glob(name + "*") if (p / "unrouted.dsn").is_file())
                if d:
                    boards.append(Board("pcb/" + d[0].name[:30], d[0] / "unrouted.dsn", QUICK_ARGS))
            else:
                boards.append(Board(name, FIXTURES / suite / f"{name}.dsn", QUICK_ARGS))
        return boards
    if spec == "dense" or spec.startswith("dense:"):
        n = int(spec.split(":", 1)[1]) if ":" in spec else 12
        quick = {name for suite, name in QUICK if suite == "pcbench"}
        picked = []
        for p in sorted((FIXTURES / "PCBench").glob("*/unrouted.dsn")):
            if any(p.parent.name.startswith(q) for q in quick):
                continue
            dense, total = dense_part_count(p)
            if dense and total <= DENSE_MAX_COMPONENTS:
                picked.append((-dense, p.parent.name, p))
        return [Board("pcb/" + p.parent.name[:30], p, QUICK_ARGS) for _, _, p in sorted(picked)[:n]]
    if spec == "dac":
        return [Board(p.stem, p) for p in sorted((FIXTURES / "DAC2020_boards").glob("*.dsn"))]
    if spec == "kicad":
        return [Board(p.stem, p) for p in sorted((FIXTURES / "KiCad_10_demos").glob("*.dsn"))]
    if spec.startswith("pcbench"):
        n = int(spec.split(":", 1)[1]) if ":" in spec else 40
        all_ = sorted(p for p in (FIXTURES / "PCBench").glob("*/unrouted.dsn"))
        step = max(1, len(all_) // n)
        return [Board("pcb/" + p.parent.name[:30], p) for p in all_[::step][:n]]
    if spec.startswith("dir:"):
        boards = []
        for args_txt in sorted(Path(spec[4:]).expanduser().glob("**/args.txt")):
            raw = args_txt.read_text().split("\n")
            raw = [a for a in raw if a]
            extra, i = [], 0
            while i < len(raw):
                if raw[i] in ("-de", "-do"):
                    i += 2
                    continue
                extra.append(raw[i])
                i += 1
            name = args_txt.parent.name
            if name in ("work", ".") and args_txt.parent.parent.name:
                name = args_txt.parent.parent.name
            boards.append(Board("kicad/" + name, args_txt.parent / "board.dsn", extra))
        return boards
    p = Path(spec)
    if p.suffix == ".dsn" and p.is_file():
        # PCBench boards are all called unrouted.dsn: name them after their directory
        name = "pcb/" + p.parent.name[:30] if p.stem == "unrouted" else p.stem
        return [Board(name, p)]
    sys.exit(f"unknown suite '{spec}'")


def run_board(board, binary, out, extra, diagnose, max_time):
    safe = board.name.replace("/", "_").replace(" ", "_")
    ses, report, log = out / f"{safe}.ses", out / f"{safe}.json", out / f"{safe}.log"
    report.unlink(missing_ok=True)
    cmd = [str(binary), "-de", str(board.dsn), "-do", str(ses), f"--report={report}"]
    cmd += board.args + list(extra)
    if diagnose:
        cmd.append("--diagnose")
    if max_time:
        cmd.append(f"--max-time={max_time}")
    start = time.monotonic()
    with open(log, "w") as f:
        proc = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT)
        _, status, usage = os.wait4(proc.pid, 0)
    wall = time.monotonic() - start
    proc.returncode = os.waitstatus_to_exitcode(status)
    # ru_maxrss: bytes on macOS, KiB on Linux
    rss = usage.ru_maxrss / (1 << 20) if sys.platform == "darwin" else usage.ru_maxrss / 1024
    row = {"board": board.name, "time_s": f"{wall:.1f}", "rss_mb": f"{rss:.0f}", "exit": str(proc.returncode)}
    try:
        r = json.loads(report.read_text())
        s = r["stats"]
        row.update(
            unrouted=s["unrouted"],
            escape=s.get("unrouted_escape", ""),
            violations=s["violations"],
            vias=s["vias"],
            length_mm=f"{s['trace_length_mm']:.0f}" if s["trace_length_mm"] is not None else "",
            router_score=f"{s['router_score']:.1f}",
            optimizer_score=f"{s['optimizer_score']:.1f}",
        )
        if "diagnosis" in r:
            row.update(congestion=r["diagnosis"]["congestion"], blocked=r["diagnosis"]["blocked"])
    except (OSError, ValueError, KeyError):
        pass
    return row


def fmt_row(row):
    return " ".join(f.format(str(row.get(c, ""))) for c, f in COLUMNS)


def read_tsv(path):
    lines = Path(path).read_text().splitlines()
    head = lines[0].split("\t")
    return {r[0]: dict(zip(head, r)) for r in (l.split("\t") for l in lines[1:]) if r and r[0]}


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def totals(rows):
    t = {"board": f"TOTAL ({len(rows)} boards)"}
    for c in ("time_s", "unrouted", "escape", "violations", "vias", "length_mm"):
        vals = [num(r.get(c)) for r in rows]
        vals = [v for v in vals if v is not None]
        t[c] = f"{sum(vals):.0f}" if c != "time_s" else f"{sum(vals):.1f}"
    t["exit"] = str(sum(1 for r in rows if r.get("exit") not in ("0", 0)))
    return t


def compare(rows, base):
    """Prints boards whose result changed, and the totals of the boards in both runs."""
    print("\nchanges against the baseline (baseline -> now):")
    worse = better = 0
    common = [r for r in rows if r["board"] in base]
    for r in common:
        b = base[r["board"]]
        diffs = []
        for c in ("unrouted", "violations", "vias", "length_mm", "optimizer_score"):
            x, y = num(b.get(c)), num(r.get(c))
            if x is None or y is None or x == y:
                continue
            if c in ("vias", "length_mm") and abs(y - x) <= 0.02 * max(abs(x), 1):
                continue
            diffs.append(f"{c} {b.get(c)} -> {r.get(c)}")
        tb, tr = num(b.get("time_s")), num(r.get("time_s"))
        if tb and tr and tb > 2 and abs(tr - tb) > 0.25 * tb:
            diffs.append(f"time {tb:.1f} -> {tr:.1f} s")
        if not diffs:
            continue
        ub, ur = num(b.get("unrouted")) or 0, num(r.get("unrouted")) or 0
        vb, vr = num(b.get("violations")) or 0, num(r.get("violations")) or 0
        tag = "WORSE " if (ur, vr) > (ub, vb) else "better" if (ur, vr) < (ub, vb) else "      "
        worse += tag == "WORSE "
        better += tag == "better"
        print(f"  {tag} {r['board']:<34} " + ", ".join(diffs))
    tb = totals([base[r["board"]] for r in common])
    tn = totals(common)
    print(f"  totals over {len(common)} boards: " + ", ".join(
        f"{c} {tb[c]} -> {tn[c]}" for c in ("unrouted", "violations", "vias", "length_mm", "time_s")))
    print(f"  {better} boards better, {worse} worse (unrouted, then violations)")
    missing = [b for b in base if b not in {r["board"] for r in rows}]
    if missing and len(missing) < len(base):
        print(f"  ({len(missing)} baseline boards not in this run)")


def main():
    argv = sys.argv[1:]
    extra = []
    if "--" in argv:
        i = argv.index("--")
        argv, extra = argv[:i], argv[i + 1:]
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--suite", default="dac,kicad")
    ap.add_argument("--out", type=Path, default=Path("/tmp/fastroute-bench"))
    ap.add_argument("-j", "--jobs", type=int, default=1, help="boards routed at the same time (times get noisy)")
    ap.add_argument("--baseline", type=Path, help="results.tsv of an earlier run to compare with")
    ap.add_argument("--diagnose", action="store_true", help="classify unrouted connections (congestion/blocked)")
    ap.add_argument("--max-time", type=int, help="per-board time limit in seconds")
    target = os.environ.get("CARGO_TARGET_DIR", str(ROOT / "target"))
    ap.add_argument("--bin", type=Path, default=Path(os.environ.get("FASTROUTE_BIN", Path(target) / "release/fastroute")))
    args = ap.parse_args(argv)

    boards = [b for spec in args.suite.split(",") if spec for b in suite_boards(spec)]
    missing = [b for b in boards if not b.dsn.is_file()]
    if missing:
        sys.exit("missing: " + ", ".join(str(b.dsn) for b in missing))
    args.out.mkdir(parents=True, exist_ok=True)
    version = subprocess.run([str(args.bin), "-V"], capture_output=True, text=True).stdout.strip()
    print(f"{version} ({args.bin}), {len(boards)} boards, extra args: {' '.join(extra) or '-'}")
    print(fmt_row({c: c for c, _ in COLUMNS}))

    rows = [None] * len(boards)
    if args.jobs <= 1:
        for i, b in enumerate(boards):
            rows[i] = run_board(b, args.bin, args.out, extra, args.diagnose, args.max_time)
            print(fmt_row(rows[i]), flush=True)
    else:
        from concurrent.futures import ThreadPoolExecutor

        with ThreadPoolExecutor(args.jobs) as ex:
            futs = {ex.submit(run_board, b, args.bin, args.out, extra, args.diagnose, args.max_time): i
                    for i, b in enumerate(boards)}
            for f in futs:
                rows[futs[f]] = f.result()
                print(fmt_row(rows[futs[f]]), flush=True)
    print(fmt_row(totals(rows)))
    print(f"fully routed: {sum(1 for r in rows if str(r.get('unrouted')) == '0')}/{len(rows)}")

    tsv = args.out / "results.tsv"
    with open(tsv, "w") as f:
        f.write("\t".join(c for c, _ in COLUMNS) + "\n")
        for r in rows:
            f.write("\t".join(str(r.get(c, "")) for c, _ in COLUMNS) + "\n")
    print(f"results: {tsv}")
    if args.baseline:
        compare(rows, read_tsv(args.baseline))


if __name__ == "__main__":
    main()
