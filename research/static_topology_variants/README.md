# `static_topology` ancestry: a measured negative result

This directory exists to keep one **falsified optimisation** reproducible, instead of leaving it as a
suggestion in a document.

## The hypothesis

`doc/COMPILE_COST.md` §3 recorded, as an obvious optimisation direction:

> `ancestry` 目前按值复制父数组，可改为继承 + 计数（每层只记录父指针与自身名字，`ancestry` 查询
> 沿父链递归），把 `O(d²)` 降到 `O(d)`。

Each `Node<Name, Parent>` materialises `static constexpr std::array<std::string_view, depth> ancestry`
by copying the parent's array and appending its own name — O(d) work per level, O(d²) along a chain.

## The measurement

`static_topology_walk.hpp` in this directory is that change, implemented: the array is removed and
`ancestry_contains` / `ancestor_name` / `ancestry_of` walk the parent chain instead. Compiling a
depth-64 chain with `-ftime-report`, best of 3:

| variant | template instantiation | GGC memory |
|---|---|---|
| **array copy (shipped)** | **0.17 s** | **13 MB** |
| parent-chain walk (rejected) | 0.55 s | 20 MB |

**walk / array = 3.24×** — the "asymptotically better" version is more than three times slower.

## Why the asymptotic argument does not apply

The abstract operation count is not the cost model of template instantiation:

* copying an array is **one** constexpr lambda evaluation per node, with a small loop inside;
* walking the chain is a **recursive function template**: one distinct instantiation per ancestor
  level, each with its own mangled symbol and its own constexpr evaluation. The compiler does far
  more work even though the abstract step count is lower.

`-ftime-report`'s "template instantiation" line is used rather than total wall clock because the
total is dominated by parsing `<string_view>`/`<array>` and is noisy on this machine (±0.15 s),
while the instantiation line is stable to ~0.02 s.

## Reproduce

```bash
export TMPDIR="$PWD/log/scratch"      # /tmp is isolated per invocation in this environment
python3 research/static_topology_variants/measure_variants.py --runs 3 --depth 64
```

The script generates the chain TU, shadows the live header with the variant through `-I`, compiles
both, and prints the two rows above. The shipped header is unchanged; the variant is kept only here.

## What this means for the item

`IMPLEMENTATION_GUIDE.md` §12 item 15 ("deep-chain compile cost") is closed as **investigated and
rejected on measurement**, not as done. The remaining honest statement is the one already in
`COMPILE_COST.md`: deep chains (>32 levels) are expensive, real control hierarchies are shallow
(≤10), and at those depths the cost is irrelevant next to `rclcpp`.
