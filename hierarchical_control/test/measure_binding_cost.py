#!/usr/bin/env python3
"""Measure the compile-time cost of the BINDING layer: a deep chain vs a branching tree.

WHY THIS EXISTS
---------------
`measure_compile_cost.py` measures `static_topology` alone and shows that depth is what costs: a
chain grows super-linearly per node, a wide fan-out does not. Review item B then turned `BoundNode`
from a single `Next` slot into a variadic `Children...` pack stored in a `std::tuple`, which raises
the obvious question: does describing a TREE now cost more to instantiate than describing a chain of
the same size?

This script answers it for the binding layer (`topology_contract` + `topology_binding`, i.e.
`compose` / `make_leaf` / `fill_spec_rows` / the compile-time edge checks), at EQUAL NODE COUNT:

    chain of N  : root -> c1 -> ... -> c(N-1)
    tree  of N  : root -> M modules -> K leaves each      (1 + M + M*K == N)

Both TUs declare one `MinimalController` per node and one `Contract<PortList<>, PortList<>>`
per node, so nothing but the SHAPE differs. The metric is the marginal cost between two sizes, so
the header's fixed parse cost and the rclcpp/controller_interface parse cost (which dominate the
absolute numbers, ~11 s here) cancel.

MEASURED (2026-09-24, this container: g++ 11, `-fsyntax-only`, best of 5, THREE runs)

    run  : chain ms/node   tree ms/node
    1    : 130.5           25.2
    2    : 153.2            2.2
    3    : 172.1            2.3

So branching is not the expensive direction -- depth is. The TREE number is the noisy one (2-25
ms/node); the CHAIN number is stable (130-172 ms/node) and the ordering never reversed, so the
transferable statement is "a deep chain costs far more per node than a branching tree of the same
size", not a precise ratio. This matches `measure_compile_cost.py`, which finds the same asymmetry
inside `static_topology` alone (deep chain ~9.7 ms/node at depth 64 vs wide fan-out ~1.1 ms/child).

Usage:
    python3 hierarchical_control/test/measure_binding_cost.py [--runs 5]
"""
import argparse
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.abspath(os.path.join(HERE, os.pardir, os.pardir))

HEAD = """#include "hierarchical_control/topology_contract.hpp"
#include "hierarchical_control/topology_binding.hpp"
#include "test_controller_stub.hpp"
namespace tc = hierarchical_control::topology_contract;
namespace st = hierarchical_control::static_topology;
namespace hct = hierarchical_control_test;
using Empty = tc::Contract<tc::PortList<>, tc::PortList<>>;
"""


def include_flags():
    """Reuse the include paths colcon resolved, because controller_interface is involved.

    Falls back to the two package include dirs, which is enough only on an installed tree; the
    caller is told when the compile fails.
    """
    flags_make = os.path.join(
        WS, "build", "hierarchical_control", "CMakeFiles", "test_contract_regression.dir",
        "flags.make")
    flags = []
    if os.path.isfile(flags_make):
        with open(flags_make) as handle:
            for line in handle:
                if line.startswith("CXX_INCLUDES"):
                    flags = line.split("=", 1)[1].split()
                    break
    flags.append("-I" + os.path.join(WS, "hierarchical_control", "test"))
    return flags


def node_declaration(index, parent):
    root = parent is None
    return (
        f'struct n{index} {{ static constexpr auto value = st::NameOf("node{index}"); }};\n'
        f'inline hct::MinimalController obj{index}{{"node{index}"}};\n' +
        (f"using node{index} = st::Root<n{index}>;\n" if root
         else f"using node{index} = st::Descendant<n{index}, node{parent}>;\n"))


def chain_source(count):
    """root -> c1 -> ... -> c(count-1), composed from the innermost node outwards."""
    out = [HEAD]
    for i in range(count):
        out.append(node_declaration(i, None if i == 0 else i - 1))
    out.append(f"inline const auto leaf = tc::make_leaf<node{count - 1}, Empty>(&obj{count - 1});\n")
    previous = "leaf"
    for i in range(count - 2, -1, -1):
        out.append(f"inline const auto b{i} = tc::compose<node{i}, Empty>(&obj{i}, {previous});\n")
        previous = f"b{i}"
    out.append(f"static_assert(tc::binding_depth<std::decay_t<decltype({previous})>>() == {count});\n")
    out.append("int main() { return 0; }\n")
    return "".join(out)


def tree_source(modules, per_module):
    """root -> `modules` children -> `per_module` leaves each. Returns (source, node count)."""
    out = [HEAD]
    out.append(
        'struct nroot { static constexpr auto value = st::NameOf("root"); };\n'
        'inline hct::MinimalController objroot{"root"};\n'
        "using noderoot = st::Root<nroot>;\n")
    total = 1
    module_bindings = []
    for m in range(modules):
        module = f"m{m}"
        out.append(
            f'struct n{module} {{ static constexpr auto value = st::NameOf("mod{m}"); }};\n'
            f'inline hct::MinimalController obj{module}{{"mod{m}"}};\n'
            f"using node{module} = st::Descendant<n{module}, noderoot>;\n")
        total += 1
        leaves = []
        for k in range(per_module):
            leaf = f"l{m}_{k}"
            out.append(
                f'struct n{leaf} {{ static constexpr auto value = st::NameOf("leaf{m}_{k}"); }};\n'
                f'inline hct::MinimalController obj{leaf}{{"leaf{m}_{k}"}};\n'
                f"using node{leaf} = st::Descendant<n{leaf}, node{module}>;\n"
                f"inline const auto {leaf} = tc::make_leaf<node{leaf}, Empty>(&obj{leaf});\n")
            leaves.append(leaf)
            total += 1
        out.append(
            f"inline const auto mod{m} = "
            f"tc::compose<node{module}, Empty>(&obj{module}, {', '.join(leaves)});\n")
        module_bindings.append(f"mod{m}")
    out.append(
        f"inline const auto root = "
        f"tc::compose<noderoot, Empty>(&objroot, {', '.join(module_bindings)});\n")
    out.append(f"static_assert(tc::binding_depth<std::decay_t<decltype(root)>>() == {total});\n")
    out.append("int main() { return 0; }\n")
    return "".join(out), total


def time_compile(source, tag, flags, runs):
    path = os.path.join(HERE, f"_binding_cost_{tag}.cpp")
    with open(path, "w") as handle:
        handle.write(source)
    best = None
    for _ in range(runs):
        start = time.monotonic()
        result = subprocess.run(
            ["g++", "-std=c++17", "-fsyntax-only"] + flags + [path],
            capture_output=True, text=True)
        elapsed = time.monotonic() - start
        if result.returncode != 0:
            print(f"compile failed for {tag}:\n{result.stderr[:2000]}", file=sys.stderr)
            sys.exit(1)
        best = elapsed if best is None else min(best, elapsed)
    os.remove(path)
    return best


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs", type=int, default=5)
    args = parser.parse_args()
    flags = include_flags()

    # Equal node counts: 1 + M + M*K == N.
    sizes = [(41, (5, 7)), (85, (6, 13))]
    print(f"g++ -fsyntax-only, best of {args.runs} runs, marginal cost per node")
    print(f"{'N':>5} {'chain s':>10} {'tree s':>10} {'tree - chain ms':>16}")
    chains, trees = {}, {}
    for count, (modules, per_module) in sizes:
        chain = time_compile(chain_source(count), f"chain_{count}", flags, args.runs)
        source, total = tree_source(modules, per_module)
        assert total == count, (total, count)
        tree = time_compile(source, f"tree_{modules}x{per_module}", flags, args.runs)
        chains[count], trees[count] = chain, tree
        print(f"{count:>5} {chain:>10.3f} {tree:>10.3f} {(tree - chain) * 1000:>16.1f}")

    for (n0, _), (n1, _) in zip(sizes, sizes[1:]):
        chain_slope = (chains[n1] - chains[n0]) / (n1 - n0) * 1000
        tree_slope = (trees[n1] - trees[n0]) / (n1 - n0) * 1000
        print(
            f"marginal {n0}->{n1}: chain {chain_slope:.1f} ms/node, "
            f"tree {tree_slope:.1f} ms/node, tree/chain {tree_slope / chain_slope:.2f}")
    print(
        "\nAbsolute times are machine-specific. What transfers is the ORDERING and the ratio: "
        "depth is the expensive shape, branching is not.")


if __name__ == "__main__":
    main()
