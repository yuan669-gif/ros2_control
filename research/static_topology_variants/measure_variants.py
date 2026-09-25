#!/usr/bin/env python3
"""A/B the two `static_topology` ancestry implementations (review item: deep-chain compile cost).

WHY THIS EXISTS
---------------
`doc/COMPILE_COST.md` section 3 used to record an *optimisation direction*: `Node` materialises its
root-to-self ancestry as `std::array<std::string_view, depth>` by copying the parent's array, which
is O(d) work per level and therefore O(d^2) for a chain of depth d. The suggested fix was to store
only the parent pointer and walk the parent chain, "把 O(d^2) 降到 O(d)".

That change was implemented and measured. **It is about twice as SLOW**, and the asymptotic argument
is simply the wrong cost model here:

    variant                        template instantiation     GGC memory     total
    parent-chain walk (rejected)   0.56 / 0.58 / 0.63 s       20 MB          ~1.07 s
    array copy (shipped)           0.24 / 0.28 / 0.28 s       13 MB          ~0.60 s

Copying an array is ONE constexpr lambda evaluation per node. Walking the chain is a RECURSIVE
FUNCTION TEMPLATE: one distinct instantiation per ancestor level, each with its own symbol and
constexpr evaluation, so the compiler does far more work even though the abstract asymptotic count is
lower. Wall-clock measurement of the whole compile is noisy; `-ftime-report`'s "template
instantiation" line is stable to ~0.05 s and is what this script reads.

This script keeps the rejected variant next to it so the negative result stays reproducible instead
of becoming a claim in a document.

Usage:
    python3 research/static_topology_variants/measure_variants.py [--runs 3] [--depth 64]
"""
import argparse
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.abspath(os.path.join(HERE, os.pardir, os.pardir))
LIVE_INCLUDE = os.path.join(WS, "hierarchical_control", "include")
VARIANT_DIR = HERE
VARIANT_HEADER = os.path.join(VARIANT_DIR, "static_topology_walk.hpp")

SOURCE = """#include "hierarchical_control/static_topology.hpp"
namespace st = hierarchical_control::static_topology;
struct n0 {{ static constexpr auto value = st::NameOf("n0"); }};
using t0 = st::Root<n0>;
{body}
static_assert(st::node_count<t{depth_minus_1}>() == {depth});
"""


def chain_source(depth):
    body = []
    for i in range(1, depth):
        body.append(f'struct n{i} {{ static constexpr auto value = st::NameOf("n{i}"); }};')
        body.append(f"using t{i} = st::Descendant<n{i}, t{i - 1}>;")
    return SOURCE.format(body="\n".join(body), depth=depth, depth_minus_1=depth - 1)


TIME_RE = re.compile(r"template instantiation\s*:\s*([0-9.]+)\s*\([^)]*\)\s*([0-9.]+)\s*\([^)]*\)\s*([0-9.]+)\s*\([^)]*\)\s*([0-9.]+)([kMG]?)")


def measure(include_dir, source_path, runs):
    """Best-of-N template-instantiation wall time (seconds) and GGC memory (MB)."""
    best = None
    for _ in range(runs):
        proc = subprocess.run(
            ["g++", "-std=c++17", "-fsyntax-only", f"-I{include_dir}", source_path, "-ftime-report"],
            capture_output=True, text=True)
        if proc.returncode != 0:
            print(proc.stderr[-2000:], file=sys.stderr)
            raise SystemExit(f"compile failed against {include_dir}")
        match = TIME_RE.search(proc.stderr)
        if match is None:
            raise SystemExit("could not parse -ftime-report output")
        wall = float(match.group(1))
        ggc_value = float(match.group(4))
        ggc = ggc_value * {"": 1.0, "k": 1e-3, "M": 1.0, "G": 1e3}[match.group(5)]
        if best is None or wall < best[0]:
            best = (wall, ggc)
    return best


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs", type=int, default=3, help="compiles per variant, best kept")
    parser.add_argument("--depth", type=int, default=64, help="chain depth of the test TU")
    args = parser.parse_args()

    work = os.path.join(WS, "log", "scratch", "static_topology_variants")
    os.makedirs(work, exist_ok=True)
    source_path = os.path.join(work, "chain.cpp")
    with open(source_path, "w", encoding="utf-8") as handle:
        handle.write(chain_source(args.depth))

    # The variant directory must shadow the live header: put a copy of the variant under the
    # canonical include path so `#include "hierarchical_control/static_topology.hpp"` resolves to it.
    shadow_root = os.path.join(work, "include")
    shadow_dir = os.path.join(shadow_root, "hierarchical_control")
    os.makedirs(shadow_dir, exist_ok=True)
    with open(os.path.join(shadow_dir, "static_topology.hpp"), "w", encoding="utf-8") as handle:
        handle.write(open(VARIANT_HEADER, encoding="utf-8").read())

    print(f"chain depth {args.depth}, best of {args.runs} compiles, -fsyntax-only -ftime-report")
    print()
    print(f"{'variant':<34}{'template instantiation':>24}{'GGC memory':>14}")
    live = measure(LIVE_INCLUDE, source_path, args.runs)
    print(f"{'array copy (SHIPPED)':<34}{live[0]:>21.2f} s{live[1]:>11.0f} MB")
    variant = measure(shadow_root, source_path, args.runs)
    print(f"{'parent-chain walk (rejected)':<34}{variant[0]:>21.2f} s{variant[1]:>11.0f} MB")
    print()
    ratio = variant[0] / live[0] if live[0] else float("inf")
    print(f"walk / array = {ratio:.2f}x  -> the proposed O(d) walk is SLOWER; the array stays.")
    print("See doc/COMPILE_COST.md section 3 for why the asymptotic argument does not hold here.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
