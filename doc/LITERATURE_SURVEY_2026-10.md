# Literature Survey and Prior-Art Positioning (October 2026)

Scope: position two claimed contributions — (1) tree-structured bidirectional two-pass scheduling in a
ROS 2 control framework, (2) compile-time (C++17 template metaprogramming) validation and description
of controller topology and interfaces. Method: `web_search` + `web_fetch` on primary sources
(publisher/DOI records via the Crossref API, arXiv, official docs, upstream source trees), plus
inspection of the local `ros2_control` Humble tree at `/home/mamingyuan/Desktop/ros2_control-humble`.

**Verification legend.** `[V]` = I fetched the primary record/PDF/page myself. `[V-src]` = verified in
the local Humble source tree. `[V-snip]` = verified only from a search-result snippet of the primary
document. `[R]` = recalled/standard knowledge, **not** independently verified here. `[UNVERIFIED]` =
could not confirm. Bibliographic details marked `[R]` must be checked before they enter a bibliography.

---

## 1. Executive verdict (blunt)

**Innovation 1 is not a new mechanism.** The exact scheduling rule it claims — *one linearization,
executed twice per cycle, once children-first for state and once parents-first for commands, giving
zero-cycle lag in both directions* — is already published, with a proof, a schedulability analysis, a
latency bound, and rate buckets, in the paper the authors themselves cite: FineMote, arXiv:2608.04600v1
`[V]`. FineMote's Eq. (2) is that rule verbatim. What remains for the authors is a **port-and-measure**
contribution inside `ros2_control`, not a scheduling contribution. That can still be publishable, but
only if the framing changes.

**Innovation 2 is a standard application of well-known techniques to a new domain object.** Type-level
topology encoding, illegal-state elimination, type-level dimension checking, and negative compilation
tests are all mature, citable lines (typestate 1986; dimension types 1994; session types 1998;
policy-based design 2001; Pigweed's `pw_compilation_testing`). I found **no** prior work that applies
compile-time type-level topology + port-dimension validation specifically to a ROS 2 control framework
`[V]`, but that is a *domain gap*, not a *methodological* one. Reviewers will call it "engineering the
known technique into a new API", and they will be right.

---

## 2. Prior art for Innovation 1

### 2.1 The anchor reference is real, and it already contains the mechanism

| Item | Value |
|---|---|
| Paper | "Static Timing Orchestration for Tree-Structured Robot Control Firmware" |
| Authors | Wang Xi, Feiran Wei, Mo Deng, Weiheng Lin, Pangkit Fong, Jianping He (Shanghai Jiao Tong Univ.) |
| Venue/date | arXiv:2608.04600v1 [cs.RO], submitted 5 Aug 2026 |
| DOI | [10.48550/arXiv.2608.04600](https://doi.org/10.48550/arXiv.2608.04600) `[V]` |

I extracted the full PDF locally. Three passages decide the positioning question:

- **§II-C** decomposes every device task into two stages: `Update` (`τ⁺`) which "updates the internal
  state of device `dₙ` using data provided by lower-level devices", and `Handle` (`τ⁻`) which
  "generates actions for the current cycle ... and propagates decision information to lower-level
  devices" `[V]`. This is `update_phase`/`handle_phase` exactly.
- **§III-B, Eq. (2)** fixes the execution order inside each period bucket `Q_w = (d_{n1}, …, d_{n|Qw|})`:

  `τ⁺_{n1}, τ⁺_{n2}, …, τ⁺_{n|Qw|}, τ⁻_{n|Qw|}, …, τ⁻_{n2}, τ⁻_{n1}`

  i.e. **all Update stages in child→parent order, then all Handle stages in parent→child order, over
  the same list, within one bucket cycle** `[V]`. That is the claimed two-pass rule.
- **§IV-C2, Corollary 2** states the zero-lag result: for same-period, aligned devices "the forward
  traversal follows the child-to-parent direction for ■ and the reverse traversal follows the
  parent-to-child direction for ●, intra-tree propagation introduces no release-level waiting", giving
  `L_m ≤ max R̂⁻_{n1}` `[V]`. This is the claimed "zero-cycle lag in both directions", already proved.

FineMote also already supplies what the claim lists as supporting features: **rate buckets** with
RMS priorities over distinct periods, and a Liu–Layland utilization-bound schedulability test
(§III-B/§III-C) `[V]`, using Liu & Layland 1973 `[V]` (DOI [10.1145/321738.321743](https://doi.org/10.1145/321738.321743)).

A second, non-arXiv FineMote manuscript ("Synthesizing Real-Time Embedded Middleware for Complex
Mechatronic Systems in Robotics") is present in the workspace `[V-src]` and states the same design
motivation, including "data flows involving physical entities are often bidirectional, making existing
directed acyclic graph based scheduling approaches inapplicable" `[V-src]`.

### 2.2 Upstream `ros2_control` behaviour (what the contribution actually has to beat)

| Source | Type | URL | Ver. | Establishes |
|---|---|---|---|---|
| Issue #853 "Make sure that controllers are properly sorted and executed in a chain" | GitHub issue | [ros2_control#853](https://github.com/ros-controls/ros2_control/issues/853) | `[V]` | Opened 2022-11-11, **closed 2023-08-11**, label `enhancement`. Body: "When loading controllers that are part of the chain, they have to be loaded in a certain order. Otherwise, there could be a lag in execution"; asks for automatic re-sorting in `controller_manager.cpp`. |
| Issue #2189 "docs: What are preceding and following controllers?" | GitHub issue | [ros2_control#2189](https://github.com/ros-controls/ros2_control/issues/2189) | `[V]` | Opened 2025-04-16, **closed 2025-04-17**. Reports the chaining docs are wrong on Jazzy about activation order; confirms the ordering semantics are still poorly specified. |
| Chaining docs, `controller_manager/doc/controller_chaining.rst` | upstream doc | [controller_chaining.rst](https://github.com/ros-controls/ros2_control/blob/de00c17e643b19e7980e522965f23cf22c0a9c21/controller_manager/doc/controller_chaining.rst?plain=1) | `[V]` | Upstream defines preceding/following controller semantics and the state-child→parent / reference-parent→child convention. |
| `ControllerManager` Humble API | upstream API doc | [control.ros.org/humble ControllerManager](https://control.ros.org/humble/doc/api/classcontroller__manager_1_1ControllerManager.html) | `[V]` | Upstream Humble exposes `read()`, `update()`, `write()` and `chained_controllers_configuration_`; `update()` is documented as "Call update of all controllers" — a single pass. `controller_sorting` is not listed because it is a private member (consistent with the local header, where it sits under `private:` at line 558) `[V-src]`. |
| Local Humble tree | source | `controller_manager/src/controller_manager.cpp` (`std::stable_sort` + `controller_sorting`), `include/controller_manager/controller_manager.hpp:649` | `[V-src]` | At controller-configuration time the manager **already reorders the controller list** so chained controllers are consistently ordered. So "keep one linearization derived from the tree" is **not** a contribution — it is upstream behaviour since the #853 fix. A local code comment records that upstream `controller_sorting()` "places a chainable controller that claims NO command interface BEFORE its parent", i.e. the single linearization structurally favours the state direction `[V-src]`. |

**Consequence.** The delta over upstream is exactly one thing: adding a second pass. The delta over
FineMote is exactly one thing: doing it at runtime over `dlopen`-ed plugins with admission checks
instead of statically over C++17 partially-ordered dynamic initialisation.

### 2.3 Adjacent scheduling/dataflow lines

| Prior-art line | Closest primary citation | Ver. | Proximity | What it leaves uncovered |
|---|---|---|---|---|
| Synchronous Dataflow (SDF) | Lee & Messerschmitt, *Static Scheduling of Synchronous Data Flow Programs for DSP*, IEEE Trans. Computers, 1987, [10.1109/TC.1987.5009446](https://doi.org/10.1109/TC.1987.5009446) | `[V]` | **Exact theory, opposite remedy.** A cycle with no delay register has no admissible static schedule. The claim *removes* the delay rather than inserting one. | SDF forbids the problematic cycle; the claim makes it schedulable by two-phase execution. That distinction is the honest novelty hook if any exists. |
| Dataflow process networks | Lee & Parks, Proc. IEEE, 1995, [10.1109/5.381846](https://doi.org/10.1109/5.381846) | `[V]` | Analogy | Same: feedback requires initial tokens. |
| Synchronous languages / causality | Lustre: Caspi et al., POPL'87, [10.1145/41625.41641](https://doi.org/10.1145/41625.41641); Esterel: Berry & Gonthier, SCP 1992, [10.1016/0167-6423(92)90005-V](https://doi.org/10.1016/0167-6423(92)90005-V) | `[V]` | **Exact theory.** An instantaneous cycle not broken by `pre()` is a *causality error*, rejected at compile time. | The claim accepts the cycle and resolves it by ordering two passes; Lustre would reject it. Must be cited to show awareness that the "stale by one cycle" fact is textbook. |
| Logical Execution Time | Giotto: Henzinger, Horowitz, Kirsch, EMSOFT 2001, [10.1007/3-540-45449-7_12](https://doi.org/10.1007/3-540-45449-7_12); Proc. IEEE 2003, [10.1109/JPROC.2002.805825](https://doi.org/10.1109/JPROC.2002.805825); TTA: Kopetz & Bauer, Proc. IEEE 2003, [10.1109/JPROC.2002.805821](https://doi.org/10.1109/JPROC.2002.805821) | `[V]` | **Near, and rhetorically dangerous.** LET exists precisely because read/write fusion plus arbitrary global order makes results schedule-order-dependent. | LET *adds* determinism at the cost of latency; the claim *removes* latency while accepting order-dependence within the cycle. Say this explicitly. |
| AUTOSAR RTE implicit vs explicit communication | AUTOSAR CP SWS RTE, "Implicit Communication buffer handling for coherent implicit data access"; "Explicit Schedule Points … placed between RunnableEntitys after the data written with implicit …" | `[V-snip]` ([R24-11 SWS RTE](https://www.autosar.org/fileadmin/standards/R24-11/CP/AUTOSAR_CP_SWS_RTE.pdf)) | **Partial.** AUTOSAR solves coherent snapshotting of implicit (double-buffered) data and inserts explicit schedule points *between* runnables. | AUTOSAR's coherence is per-buffer, not a tree-wide "both directions fresh" guarantee; there is no bidirectional tree-inversion rule. Reviewers from automotive will ask why this is not just "implicit communication + schedule points". |
| IEC 61499 function blocks | Vyatkin, *Semantics of IEC 61499*; Wiley review [10.1155/2013/638521](https://doi.org/10.1155/2013/638521) | `[V-snip]` ([vyatkin.org PDF](http://www.vyatkin.org/publ/Semantics%20Of%20IEC%2061499.pdf)) | Analogy | Known critique: event-driven activation makes deterministic execution order ambiguous. Not a two-pass rule. |
| OROCOS / RTT | `RTT::TaskContext` API (updateHook/update), [orocos.org RTT docs](https://orocos.org/stable/documentation/rtt/v2.x/api/html/classRTT_1_1TaskContext.html) | `[UNVERIFIED — URL located, not fetched]` | Analogy | RTT is a component model without tree-structured bidirectional dataflow ordering. |
| Compiler dataflow analysis (forward/backward sweeps) | Tarjan, SIAM J. Comput. 1972, [10.1137/0201010](https://doi.org/10.1137/0201010); Kildall, POPL'73, [10.1145/512927.512945](https://doi.org/10.1145/512927.512945); Muchnick, *Advanced Compiler Design and Implementation*, 1997 `[R]` | `[V]` (Tarjan, Kildall); `[R]` (Muchnick) | **Weak analogy — do not overclaim.** Classical dataflow analysis iterates *one* direction to a fixed point across many passes; the claim runs two *opposite* directions *once each* per cycle. Sharing the word "pass" is not sharing the technique. |
| Forward–backward / belief propagation | Baum & Petrie 1966 `[R]`; Rauch–Tung–Striebel 1965 `[R]`; Pearl, *Probabilistic Reasoning in Intelligent Systems*, 1988 `[R]` | `[R]` | Analogy only | Upward and downward sweeps on a tree. Structurally suggestive; no real-time scheduling content. Mark `[R]` — these were not verified here. |
| EtherCAT consistent process data image | ETG materials; Beckhoff EL3783 manual ("the next EtherCAT cycle fetches this data") | `[V-snip]` | Analogy, and a boundary condition | Fieldbus I/O images are already snapshotted consistently per cycle; the claim's issue is *controller-level* ordering, not I/O coherence. State this so reviewers do not think the I/O layer was overlooked. |
| Two-phase commit | Gray & Lamport, ACM TODS 2006, [10.1145/1132863.1132867](https://doi.org/10.1145/1132863.1132867); Gray & Reuter, *Transaction Processing*, 1993 `[R]` | `[V]` (Gray & Lamport) | **Not related — cite to disambiguate.** "Two-pass" ≠ "two-phase commit". The absence of atomic whole-group commit/rollback is a genuine limitation, not a novelty. |

---

## 3. Prior art for Innovation 2

| Prior-art line | Primary citation | Ver. | Proximity | What it leaves uncovered |
|---|---|---|---|---|
| Typestate | Strom & Yemini, IEEE TSE 1986, [10.1109/TSE.1986.6312929](https://doi.org/10.1109/TSE.1986.6312929) | `[V]` | **Conceptual ancestor.** "Illegal operation sequences are compile-time errors" is 40 years old. | Language-level, not a type list encoding a topology. |
| Session types | Honda, Vasconcelos, Kubo, ESOP 1998, [10.1007/BFb0053567](https://doi.org/10.1007/BFb0053567) | `[V]` | Analogy | Protocols, not component hierarchies. |
| Dimension types | Kennedy, *Dimension Types*, ESOP'94, [10.1007/3-540-57880-3_23](https://doi.org/10.1007/3-540-57880-3_23); *Relational parametricity and units of measure*, POPL'97, [10.1145/263699.263761](https://doi.org/10.1145/263699.263761); *Types for Units-of-Measure*, 2010, [10.1007/978-3-642-17685-2_8](https://doi.org/10.1007/978-3-642-17685-2_8) | `[V]` | **Directly covers the port-dimension claim.** A position-vs-velocity mismatch failing to compile is exactly what dimension types do. | Nothing at the units level. The contribution here is *applying* it to control ports. |
| Units libraries in C++ | `std::chrono`; Boost.Units (Brown 2001 `[R]`); mp-units ([mpusz.github.io/mp-units](https://mpusz.github.io/mp-units/)) | `[V]` (mp-units URL) | Direct | Ready-made; no research claim survives here. |
| Policy-based design / static interfaces / CRTP | Alexandrescu, *Modern C++ Design*, 2001 `[R]` | `[R]` | Near | Establishes the idiom, not the application. |
| C++ concepts & static assertion guidance | C++ Core Guidelines, [isocpp.github.io/CppCoreGuidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines) (T.10–T.13) `[R]`; Reflection for C++26, [P2996R9](https://isocpp.org/files/papers/P2996R9.html) `[V]` | mixed | Near | The Guidelines make `static_assert`-style intent standard practice. P2996 matters because C++17 (the stated target) **lacks** reflection, which is the honest justification for a type-chain encoding. |
| Negative compilation testing | Pigweed `pw_compilation_testing`, [pigweed.dev](http://pigweed.dev/pw_compilation_testing/) | `[V-snip]` | Direct | "Negative compile tests" are a documented, tool-supported practice *in embedded C++ specifically*. Do not present them as novel. |
| Code generation instead of metaprogramming | `generate_parameter_library` ([PickNikRobotics](https://github.com/PickNikRobotics/generate_parameter_library)) | `[V]` | Direct alternative | ROS 2's own answer to "validate configuration statically" is **codegen from YAML**, not template metaprogramming. Reviewers will ask why TMP beats codegen here. This is the single most likely methodological objection to Innovation 2. |
| Chaining API baseline | `ChainableControllerInterface` ([control.ros.org](https://control.ros.org/jazzy/doc/api/classcontroller__interface_1_1ChainableControllerInterface.html)) | `[V]` | Baseline | Upstream chaining is name-string-based and fully runtime; that is the gap the claim addresses. |

**Verdict.** I found no prior work encoding a *controller topology* as a C++ type chain with
`static_assert` diagnostics in a ROS 2 control framework `[V]`. Taken as a *systems-engineering
artifact* this is defensible. Taken as a *research contribution* it is not: every ingredient is
prior art, and the composition is a straightforward application. The defensible claim is an
**enumeration**: what in a real-time control framework is *statically determinable*, with a measurement
of how far compile time can be pushed before user configuration forces runtime checks.

---

## 4. Adjacent theory that must be cited

| Topic | Citation | DOI/URL | Ver. |
|---|---|---|---|
| Sampling-induced delay / phase margin | Franklin, Powell, Emami-Naeini, *Feedback Control of Dynamic Systems* `[R]`; Åström & Wittenmark, *Computer-Controlled Systems* (3rd ed., 1997) `[R]` | chapter numbers `[R]` | `[R]` — verify exact chapters before citing |
| Networked control stability | Zhang, Branicky, Phillips, *Stability of Networked Control Systems*, IEEE Control Systems Mag. 21(1), Feb 2001 | [10.1109/37.898794](https://doi.org/10.1109/37.898794) | `[V]` |
| SDF | Lee & Messerschmitt 1987 | [10.1109/TC.1987.5009446](https://doi.org/10.1109/TC.1987.5009446) | `[V]` |
| Dataflow process networks | Lee & Parks 1995 | [10.1109/5.381846](https://doi.org/10.1109/5.381846) | `[V]` |
| Task-chain / holistic latency | Tindell & Clark, *Holistic schedulability analysis for distributed hard real-time systems*, Microprocessing & Microprogramming 1994 | [10.1016/0165-6074(94)90080-9](https://doi.org/10.1016/0165-6074(94)90080-9) | `[V]` |
| Cause–effect chain data age | Becker, Dasari, Mubeen, Behnam, Nolte, *End-to-end timing analysis of cause-effect chains in automotive embedded systems*, JSA 2017 | [10.1016/j.sysarc.2017.09.004](https://doi.org/10.1016/j.sysarc.2017.09.004) | `[V]` |
| Sporadic cause–effect chains | Dürr, von der Brüggen, Chen, Chen, ACM TECS 2019 | [10.1145/3358181](https://doi.org/10.1145/3358181) | `[V]` |
| Tarjan topological order / DFS | Tarjan, SIAM J. Comput. 1(2), 1972 | [10.1137/0201010](https://doi.org/10.1137/0201010) | `[V]` |
| Forward/backward dataflow iteration | Kildall, POPL'73 | [10.1145/512927.512945](https://doi.org/10.1145/512927.512945) | `[V]` |
| Kahn networks | Kahn, *The semantics of a simple language for parallel programming*, IFIP 1974 | `[UNVERIFIED]` — see "Kahn networks, 50 years later", [GDR SoC²](https://www.gdr-soc.cnrs.fr/2024/10/28/50-years-of-kpn/) for a citable modern retrospective | `[V-snip]` for the retrospective |
| Two-phase commit | Gray & Lamport, ACM TODS 2006 | [10.1145/1132863.1132867](https://doi.org/10.1145/1132863.1132867) | `[V]` |
| RMS bound | Liu & Layland, JACM 1973 | [10.1145/321738.321743](https://doi.org/10.1145/321738.321743) | `[V]` |
| Fixed-priority response time | Joseph & Pandya 1986 `[R]` | — | `[R]` |
| Anchor | Xi et al., arXiv:2608.04600v1 | [10.48550/arXiv.2608.04600](https://doi.org/10.48550/arXiv.2608.04600) | `[V]` |

---

## 5. Gap analysis and honest verdict

### Innovation 1

- **Novel:** essentially nothing at the mechanism level. FineMote Eq. (2) + Corollary 2 already give
  the two-pass, child-first-state / parent-first-command rule and the zero-impact-on-release-waiting
  result `[V]`. Upstream `ros2_control` already derives and maintains one linearization from the tree
  `[V-src]`, `[V]`.
- **Standard:** "a bidirectional edge in a single-pass schedule implies one unit of staleness";
  "feedback without a delay is a causality error" (Lustre/Esterel/SDF); "rate buckets + RMS"
  (FineMote, Liu & Layland).
- **Genuinely defensible residues:** (a) an *empirical* characterisation of upstream `ros2_control`'s
  silent, order-dependent degradation of the reference direction — including the mechanism by which
  `controller_sorting()` biases the linearization `[V-src]`; (b) demonstrating that FineMote's static
  scheme is not directly transferable to a plugin-based, runtime-loaded controller set, and that the
  residue is a finite, enumerable set of admission checks; (c) quantifying the control cost of the
  removed lag (phase margin / achievable gain).
- **What a reviewer attacks:** "This is FineMote §III-B Eq. (2) re-implemented." "Upstream already
  sorts controllers; you added a loop." "Your two-pass is not forward/backward dataflow iteration —
  that iterates to a fixed point." "You dropped atomic commit — so a mid-cycle failure leaves
  partially written command interfaces; two-phase commit exists precisely for this."
- **Must-cite (6):** FineMote arXiv:2608.04600 `[V]`; `ros2_control` issue #853 `[V]`; Lee &
  Messerschmitt 1987 `[V]`; Lustre/Esterel (causality) `[V]`; Giotto/LET `[V]`; Zhang, Branicky &
  Phillips 2001 `[V]`.
- **Defensible contribution statement:** *"We port and empirically validate a two-pass
  child-first-state / parent-first-command execution discipline — previously established for
  compile-time-generated tree-structured firmware — onto `ros2_control`'s runtime-plugin controller
  set, quantify the one-cycle reference staleness that upstream's single linearisation silently
  incurs, and reduce the residual correctness conditions to a finite set of runtime admission
  checks."*

### Innovation 2

- **Novel:** the *application* to controller topology and control-port dimensions in a ROS 2 control
  framework appears unpublished `[V]`; the *techniques* are all prior art (typestate 1986, dimension
  types 1994, session types 1998, policy-based design 2001, negative compilation testing in Pigweed).
- **Standard:** type chains, `static_assert` diagnostics, unit-typed values, illegal-state
  elimination.
- **Orthogonal-but-publishable angle:** the research question "what is statically determinable in a
  real-time control framework, and where exactly does compile time stop?" is a genuine, answerable
  systems question — and the C++17 target is defensible (no reflection until P2996 `[V]`).
- **What a reviewer attacks:** "Codegen (`generate_parameter_library`) already does static validation
  from YAML; why TMP?" "Compile-time-only topology cannot express the runtime reality that
  `ros2_control` loads and unloads controllers dynamically and permits cycles via `switch_controller`
  — so your static guarantee covers only a subset." "Compile cost and diagnostic quality?" (the
  project's own `doc/COMPILE_COST.md` suggests the authors have started on this) "Units libraries
  already exist; you wrote another one."
- **Must-cite (6):** Strom & Yemini 1986 `[V]`; Kennedy ESOP'94 + POPL'97 `[V]`; Honda et al. 1998
  `[V]`; C++ Core Guidelines `[R]`/P2996 `[V]`; `generate_parameter_library` `[V]`; Pigweed
  `pw_compilation_testing` `[V-snip]`.
- **Defensible contribution statement:** *"We enumerate what a real-time control framework can
  determine statically and realise it in C++17 template metaprogramming — a type-chain controller
  hierarchy in which cycles, parent/child ownership violations, and port-dimension mismatches are
  compile errors — and report the measured boundary where user configuration forces the checks back
  to runtime."*

---

## 6. Venue notes

| Venue | Type | Typical contribution | Evaluation expectation | Fit | Source |
|---|---|---|---|---|---|
| ECRTS | Conf., real-time systems (top-3 with RTSS/RTAS) | Timing requirements, broadly construed; two tracks: Foundations/Theory and Tools/Implementations/Practical Experience | 20 pages LIPIcs excl. bibliography; **double-blind**; optional artifact evaluation; must address a timing requirement | Innovation 1 (Tools track) — but must survive a real-time audience asking "what is new over FineMote?" | [ECRTS 2026 CFP](https://archives.ecrts.org/fileadmin/WebsitesArchiv/ecrts2026/call-for-papers/) `[V]` |
| JOSS | Journal, software | Research software with demonstrated impact; **explicitly accepts** software that "implements solutions already solved in other software packages" *provided it cites prior similar work* | Installable, OSI license, tests/CI, ≥6 months public development, statement of need, **state of the field** comparing to alternatives and a build-vs-contribute justification, research-impact statement | Best realistic home for the software artifact — and its rules force exactly the honest framing recommended above | [JOSS review criteria](https://joss.readthedocs.io/en/latest/review_criteria.html) `[V]` |
| ICRA / IROS | Conf., robotics | Systems-and-algorithms; a scheduling change needs either theory or strong hardware evidence | Page limits and the exact 2026 CFP were **not verified here** `[UNVERIFIED]` | Possible for the combined paper | — |
| RA-L | Journal, robotics | Shorter, rolling; systems papers accepted | See [RA-L information for authors](https://www.ieee-ras.org/publications/ra-l/ra-l-information-for-authors/) `[UNVERIFIED — not fetched]` | Plausible for Innovation 1+measurements | — |
| RTSS / RTAS | Conf., real-time | Hard theory or systems RT | Theory reviewers will demand a schedulability/latency result the claim does not have (FineMote has it) | Weak unless the two-pass rule is extended beyond FineMote | — |
| T-RO / TII | Journals | Substantial systems contributions with validation | TII in particular favours industrial control software with quantitative evaluation | Innovation 1+2 combined, if the evaluation is strong | [TII scope](https://ieeexplore.ieee.org/) `[UNVERIFIED]` |
| ROSCon | Community conference | Practitioner talks on ROS ecosystem software | Not archival peer review; proposal-based selection (see [ROSCon BE 2026](https://roscon.ros.org/be/2026/) and [ROSCon UK 2026 CFP](https://ogin.easychair.org/cfp/rosconuk2026)) `[V-snip]` | Excellent for the engineering narrative and community feedback; **not** a substitute for a peer-reviewed claim | `[V-snip]` |
| CASE / IFAC WC | Conf. | Applied control/automation with software | Moderate novelty bar | Fallback | — |

---

## 7. Limitations of this survey

- `raw.githubusercontent.com` and direct `curl` are blocked in this sandbox, so upstream files were
  verified through the official docs site and the local Humble tree rather than pristine tarballs.
  I could not diff the local `controller_manager.cpp` against unmodified upstream Humble line by line.
- Kahn 1974, Muchnick 1997, Gray & Reuter 1993, Alexandrescu 2001, Franklin/Powell/Emami-Naeini, and
  Åström & Wittenmark are marked `[R]`; their exact editions/chapters/ISBNs were **not** verified here.
- OROCOS/RTT, AUTOSAR SWS RTE, IEC 61499, and EtherCAT are `[V-snip]` at best: primary PDFs were
  located but not fully read. Cite them from the primary documents, not from this report.
- ICRA/IROS page limits and RA-L/TII author guidelines are `[UNVERIFIED]`.
- No systematic search of IEEE Xplore or ACM DL full text was possible (paywalls); the Crossref API
  was used to confirm DOIs and titles. A reviewer-grade survey should additionally run Xplore/Scopus
  full-text queries on: `"two-phase" AND "controller chain"`, `"update" AND "handle" AND "scheduling"
  AND firmware`, `compile-time AND controller AND topology`, and `type-level AND real-time AND
  scheduler`.
- I found **no** prior publication of the two-pass rule specifically inside `ros2_control` `[V]`, but
  absence of evidence from this tool set is weak evidence of absence.
