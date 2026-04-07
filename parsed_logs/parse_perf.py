#!/usr/bin/env python3
"""
parse_perf.py — Parse .wpilog files and report per-subsystem timing statistics.

Usage:
    python parse_perf.py                        # latest log
    python parse_perf.py path/to/log.wpilog     # specific log
    python parse_perf.py --all                  # all logs in ../logs/

Requires: pip install pyntcore
"""

import argparse
import os
import sys
from pathlib import Path

import numpy as np

try:
    from wpiutil.log import DataLogReader
except ImportError:
    sys.exit("ERROR: Install pyntcore first:  pip install pyntcore")

LOGS_DIR = Path(__file__).resolve().parent.parent / "logs"

# Entry name prefixes we care about
PERF_PREFIX = "/Robot/Perf/"
LOOP_PERIOD_MS = 20.0


def parse_log(filepath: Path) -> dict[str, list[float]]:
    """Return {short_name: [values]} for all Perf/* double entries."""
    reader = DataLogReader(str(filepath))
    entries: dict[int, str] = {}  # entry_id -> short name
    entry_types: dict[int, str] = {}  # entry_id -> type string
    series: dict[str, list[float]] = {}

    for rec in reader:
        if rec.isStart():
            data = rec.getStartData()
            if data.name.startswith(PERF_PREFIX):
                short = data.name[len(PERF_PREFIX) :]
                entries[data.entry] = short
                entry_types[data.entry] = data.type
                series.setdefault(short, [])
        elif not rec.isControl():
            eid = rec.getEntry()
            if eid in entries:
                name = entries[eid]
                typ = entry_types[eid]
                try:
                    if typ == "double":
                        series[name].append(rec.getDouble())
                    elif typ == "int64":
                        series[name].append(float(rec.getInteger()))
                except Exception:
                    pass
    return series


def print_stats(series: dict[str, list[float]], filepath: Path) -> None:
    """Print a formatted statistics table."""
    print(f"\n{'=' * 78}")
    print(f"  Log: {filepath.name}")
    print(f"{'=' * 78}")

    if not series:
        print("  No Perf/* entries found.")
        return

    # ── Timing entries (ms) ──────────────────────────────────────────────────
    timing_keys = [
        k
        for k in series
        if k
        not in (
            "HeapUsedMB",
            "GCDeltaCount",
            "GCDeltaTimeMs",
            "BatteryVoltage",
            "BrownoutVoltage",
            "OverrunCount",
        )
    ]

    if timing_keys:
        print(f"\n  {'Subsystem':<22} {'Mean':>7} {'P50':>7} {'P95':>7} {'P99':>7} {'Max':>7}  {'N':>6}")
        print(f"  {'-' * 22} {'-' * 7} {'-' * 7} {'-' * 7} {'-' * 7} {'-' * 7}  {'-' * 6}")

        # Sort by mean descending
        ranked = sorted(timing_keys, key=lambda k: np.mean(series[k]), reverse=True)
        for key in ranked:
            vals = np.array(series[key])
            if len(vals) == 0:
                continue
            mean = np.mean(vals)
            p50 = np.percentile(vals, 50)
            p95 = np.percentile(vals, 95)
            p99 = np.percentile(vals, 99)
            mx = np.max(vals)
            n = len(vals)
            flag = " !" if p95 > LOOP_PERIOD_MS else ""
            print(
                f"  {key:<22} {mean:>6.2f}ms {p50:>6.2f}ms {p95:>6.2f}ms {p99:>6.2f}ms {mx:>6.2f}ms  {n:>6}{flag}"
            )

    # ── Overrun summary ──────────────────────────────────────────────────────
    if "Total" in series:
        total = np.array(series["Total"])
        overruns = np.sum(total > LOOP_PERIOD_MS)
        pct = (overruns / len(total)) * 100 if len(total) > 0 else 0
        print(f"\n  Loop overruns: {int(overruns)}/{len(total)} cycles ({pct:.1f}%)")
        if overruns > 0:
            over_vals = total[total > LOOP_PERIOD_MS]
            print(
                f"  Overrun magnitudes: mean={np.mean(over_vals):.2f}ms, "
                f"max={np.max(over_vals):.2f}ms"
            )

    # ── Unaccounted time ─────────────────────────────────────────────────────
    # Total = SignalRefresh + Scheduler, and Scheduler includes all subsystem periodics
    if "Total" in series and "SignalRefresh" in series and "Scheduler" in series:
        n = min(len(series["Total"]), len(series["SignalRefresh"]), len(series["Scheduler"]))
        total = np.array(series["Total"][:n])
        sig = np.array(series["SignalRefresh"][:n])
        sched = np.array(series["Scheduler"][:n])
        unaccounted = total - sig - sched
        print(f"\n  Unaccounted time (Total - SignalRefresh - Scheduler):")
        print(f"    mean={np.mean(unaccounted):.2f}ms, max={np.max(unaccounted):.2f}ms")

    # ── GC summary ───────────────────────────────────────────────────────────
    if "GCDeltaCount" in series and "GCDeltaTimeMs" in series:
        gc_counts = np.array(series["GCDeltaCount"])
        gc_times = np.array(series["GCDeltaTimeMs"])
        gc_cycles = np.sum(gc_counts > 0)
        print(f"\n  GC events: {int(np.sum(gc_counts))} across {int(gc_cycles)} cycles")
        if np.sum(gc_counts) > 0:
            gc_nonzero = gc_times[gc_counts > 0]
            print(f"  GC time per event cycle: mean={np.mean(gc_nonzero):.2f}ms, max={np.max(gc_nonzero):.2f}ms")

    # ── Heap ─────────────────────────────────────────────────────────────────
    if "HeapUsedMB" in series:
        heap = np.array(series["HeapUsedMB"])
        print(f"\n  Heap: mean={np.mean(heap):.1f}MB, max={np.max(heap):.1f}MB")

    print()


def main():
    parser = argparse.ArgumentParser(description="Parse FRC .wpilog performance data")
    parser.add_argument("logfile", nargs="?", help="Path to .wpilog file (default: latest)")
    parser.add_argument("--all", action="store_true", help="Parse all logs in ../logs/")
    args = parser.parse_args()

    if args.all:
        files = sorted(LOGS_DIR.glob("*.wpilog"))
        if not files:
            sys.exit(f"No .wpilog files in {LOGS_DIR}")
        for f in files:
            series = parse_log(f)
            print_stats(series, f)
    elif args.logfile:
        p = Path(args.logfile)
        if not p.exists():
            sys.exit(f"File not found: {p}")
        series = parse_log(p)
        print_stats(series, p)
    else:
        files = sorted(LOGS_DIR.glob("*.wpilog"))
        if not files:
            sys.exit(f"No .wpilog files in {LOGS_DIR}")
        latest = files[-1]
        series = parse_log(latest)
        print_stats(series, latest)


if __name__ == "__main__":
    main()
