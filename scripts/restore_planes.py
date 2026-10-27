#!/usr/bin/env python3
"""Puts the original design's planes back into a PCBench board's unrouted.dsn.

PCBench's processed boards (the conversion input) have no copper zones, but the designers' inner
layers are still of KiCad's `power` type, so the exported unrouted.dsn has power layers without
planes: the router may not route on them, and nothing connects to them. The board's
reference-routed.dsn (exported from the original) has the zone outlines as `plane` scopes. This
copies those on power layers into the unrouted board's structure, so the router gets the planes the
hand route had. Pours on signal layers stay out: KiCad fills them around the tracks afterwards,
while the router would take them as fixed copper.

Usage: scripts/restore_planes.py BOARD_DIR [OUT.dsn]   (default: BOARD_DIR/planes.dsn)
"""

import re
import sys
from pathlib import Path


def _block(text, start):
    """The balanced s-expression starting at `start` (an opening parenthesis)."""
    depth = 0
    for i in range(start, len(text)):
        if text[i] == "(":
            depth += 1
        elif text[i] == ")":
            depth -= 1
            if depth == 0:
                return text[start:i + 1]
    raise ValueError(f"unbalanced parentheses at offset {start}")


def _power_layers(dsn):
    layers = []
    for m in re.finditer(r"\(layer\s+(\"[^\"]*\"|[^\s()]+)", dsn):
        if re.search(r"\(type\s+power\)", _block(dsn, m.start())):
            layers.append(m.group(1).strip('"'))
    return layers


def restore(board_dir, out):
    """Writes the unrouted board with the reference's planes on power layers to `out`; returns
    the number of planes copied (0: nothing written)."""
    board_dir = Path(board_dir)
    ref = board_dir / "reference-routed.dsn"
    if not ref.is_file():
        return 0
    dsn = (board_dir / "unrouted.dsn").read_text(encoding="utf-8", errors="replace")
    power = _power_layers(dsn)
    if re.search(r"\(plane\s", dsn) or not power:
        return 0
    rtext = ref.read_text(encoding="utf-8", errors="replace")
    planes = []
    for m in re.finditer(r"\(plane\s", rtext):
        blk = _block(rtext, m.start())
        layer = re.search(r"\(polygon\s+(\"[^\"]*\"|\S+)", blk)
        if layer and layer.group(1).strip('"') in power:
            planes.append(blk)
    if not planes:
        return 0
    k = dsn.index("(boundary", dsn.index("(structure"))
    Path(out).write_text(dsn[:k] + "\n    ".join(planes) + "\n    " + dsn[k:], encoding="utf-8")
    return len(planes)


if __name__ == "__main__":
    if len(sys.argv) not in (2, 3):
        sys.exit(__doc__)
    d = Path(sys.argv[1])
    out = Path(sys.argv[2]) if len(sys.argv) == 3 else d / "planes.dsn"
    n = restore(d, out)
    print(f"{out}: {n} plane{'' if n == 1 else 's'} restored" if n else f"{d}: nothing to restore")
