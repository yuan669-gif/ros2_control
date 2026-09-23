#!/usr/bin/env python3
"""Measure the MARGINAL compile-time cost of static_topology, across topology shapes.

WHY THIS EXISTS
---------------
hierarchical_control is header-only, so every template it instantiates is paid by every downstream
translation unit. An earlier single measurement (one linear chain) gave ~9 ms/node and was
explicitly flagged as NOT a general constant because only one shape had been measured. This script
measures the shapes that could differ.

METHOD: MARGINAL COST, NOT ABSOLUTE COST
----------------------------------------
A "baseline" file cannot simply omit the header, because then the topology does not compile at all
(the types would be undefined), and comparing against an empty file would just measure compiler
start-up. What isolates the header's marginal cost from its fixed parse cost is to compile two
files that differ by exactly one topology item:

    T(N)     -- N topology items
    T(N-1)   -- N-1 topology items
    marginal = T(N) - T(N-1)

Both include the header, so the header's own parse cost cancels and what remains is the cost of
instantiating one more item. Absolute times are machine-specific; the shape-to-shape RATIOS of the
marginal cost are the transferable result.

Usage:
    python3 hierarchical_control/test/measure_compile_cost.py [--runs 3] [--samples 3]
"""
import argparse
import os
import shutil
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
INCLUDE_DIR = os.path.abspath(os.path.join(HERE, os.pardir, "include"))

HEADER = '#include "hierarchical_control/static_topology.hpp"\n'
NS = "namespace st = hierarchical_control::static_topology;\n"


def linear_chain(prefix, depth):
    """A chain of `depth` nodes; returns one expression per node so item counts are comparable."""
    lines = [f'struct {prefix}n0 {{ static constexpr auto value = st::NameOf("{prefix}0"); }};',
             f"using {prefix}0 = st::Root<{prefix}n0>;"]
    for i in range(1, depth):
        lines.append(
            f'struct {prefix}n{i} {{ static constexpr auto value = st::NameOf("{prefix}{i}"); }};')
        lines.append(f"using {prefix}{i} = st::Descendant<{prefix}n{i}, {prefix}{i - 1}>;")
    lines.append(f"static_assert(st::node_count<{prefix}{depth - 1}>() == {depth});")
    return lines


def fanout_children(prefix, width):
    """One root with `width` children."""
    lines = [f'struct {prefix}root_n {{ static constexpr auto value = st::NameOf("{prefix}root"); }};',
             f"using {prefix}root = st::Root<{prefix}root_n>;"]
    for i in range(width):
        lines.append(
            f'struct {prefix}c{i}_n {{ static constexpr auto value = st::NameOf("{prefix}c{i}"); }};')
        lines.append(f"using {prefix}c{i} = st::Descendant<{prefix}c{i}_n, {prefix}root>;")
    return lines


def build_source(shape, chain_depth, items):
    """`items` = number of topology items to instantiate.

    linear : one chain of `chain_depth`, repeated ceil(items/chain_depth) times
    fanout : one fanout root per child, `items` children in total
    """
    body = [HEADER.rstrip(), NS.rstrip()]
    if shape == "linear":
        remaining = items
        r = 0
        while remaining > 0:
            depth = min(chain_depth, remaining)
            if depth < 2:
                break
            body.extend(linear_chain(f"t{r}_", depth))
            remaining -= depth
            r += 1
    elif shape == "fanout":
        body.extend(fanout_children("t0_", items))
    else:
        raise ValueError(shape)
    body.append("int main() { return 0; }")
    return "\n".join(body) + "\n"


def find_compiler():
    for candidate in (os.environ.get("CXX"), "g++", "c++", "clang++"):
        if candidate:
            path = shutil.which(candidate)
            if path:
                return path
    return None


def time_compile(compiler, source_path):
    cmd = [compiler, "-std=c++17", f"-I{INCLUDE_DIR}", "-fsyntax-only", source_path]
    start = time.perf_counter()
    proc = subprocess.run(cmd, capture_output=True, text=True)
    elapsed = time.perf_counter() - start
    if proc.returncode != 0:
        raise RuntimeError(f"compile failed:\n{proc.stderr[:1500]}")
    return elapsed


def best_of(compiler, source_path, runs):
    return min(time_compile(compiler, source_path) for _ in range(runs))


def fit_slope(compiler, tmp, shape, chain_depth, counts, runs):
    """Compile a file for each item count and least-squares fit time against item count.

    Subtracting two near-identical times is dominated by noise; a slope over a range of counts
    averages that noise out and is the honest way to state a per-item cost.
    """
    points = []
    for n in counts:
        path = os.path.join(tmp, f"{shape}_{chain_depth}_{n}.cpp")
        with open(path, "w") as handle:
            handle.write(build_source(shape, chain_depth, n))
        points.append((n, best_of(compiler, path, runs)))

    # least squares over (n, seconds)
    k = len(points)
    mean_n = sum(n for n, _ in points) / k
    mean_t = sum(t for _, t in points) / k
    num = sum((n - mean_n) * (t - mean_t) for n, t in points)
    den = sum((n - mean_n) ** 2 for n, _ in points)
    slope = num / den if den else 0.0
    intercept = mean_t - slope * mean_n
    return slope, intercept, points


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs", type=int, default=3, help="repeats per file (best of)")
    args = parser.parse_args()

    compiler = find_compiler()
    if compiler is None:
        print("SKIP: no C++ compiler found")
        return 0
    print(f"compiler: {compiler}   best of {args.runs} runs, -fsyntax-only, marginal cost\n")

    tmp = tempfile.mkdtemp(prefix="hc_compile_cost_")
    try:
        # fixed header-parse cost, for context
        smallest = os.path.join(tmp, "smallest.cpp")
        open(smallest, "w").write(build_source("linear", 2, 2))
        fixed = best_of(compiler, smallest, args.runs)
        print(f"  fixed cost (header + 2 nodes)        {fixed:7.3f} s\n")

        scenarios = [
            ("linear, chain depth 8", "linear", 8, (16, 32, 64, 96, 128)),
            ("linear, chain depth 16", "linear", 16, (16, 32, 64, 96, 128)),
            ("linear, chain depth 32", "linear", 32, (32, 64, 96, 128, 160)),
            ("linear, chain depth 64", "linear", 64, (64, 128, 192, 256, 320)),
            ("fanout, width (one root)", "fanout", 0, (16, 32, 64, 96, 128)),
        ]
        print("  scenario                      slope/item     intercept    points")
        slopes = {}
        for label, shape, depth, counts in scenarios:
            slope, intercept, points = fit_slope(compiler, tmp, shape, depth, counts, args.runs)
            slopes[label] = slope
            rendered = " ".join(f"{n}:{t:.3f}" for n, t in points)
            print(
                f"  {label:28s} {slope * 1000:8.3f} ms  {intercept:9.3f} s   {rendered}"
            )

        print(
            "\nREAD THIS CAREFULLY:\n"
            "  * `marginal/item` is the cost of ONE more instantiated node, with the header's fixed\n"
            "    parse cost cancelled by construction.\n"
            "  * Absolute values are specific to this machine and compiler. The transferable result is\n"
            "    the RATIO between shapes -- in particular whether deep chains cost more per node than\n"
            "    wide fan-outs (the former recurse in `depth`, the latter do not).\n"
            "  * This measures static_topology alone. A real controller TU also includes rclcpp, whose\n"
            "    cost dominates; see doc/COMPILE_COST.md for the caveat."
        )
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
