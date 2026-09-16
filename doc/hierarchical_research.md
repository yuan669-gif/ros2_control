# Bidirectional hierarchy: research contract, not a completed feature

Scope: ROS 2 Humble, upstream baseline 469f3055da3b0f097d0616c8b214072434529053.

## Corrections to the initial proposal

- A vector is a storage choice, not evidence of absent hierarchy. A dependency graph can be
  compiled into a linear execution plan without losing its ordering guarantees.
- Humble already handles branched reference chains (`controller_sorting`, controller_manager.cpp).
  Adding parent fields or a tree traversal is not by itself a research contribution.
- In a command chain, the child exports a reference interface and the parent claims/writes it.
- The checked Humble ChainableControllerInterface exports reference interfaces, not the newer
  controller state-export API. Documentation mentions of state chaining are not proof that the API
  is implemented in this version.
- A parent can read hardware state directly after hardware.read(). A bottom-up traversal is not
  necessary just to obtain fresh raw joint feedback.
- Splitting estimation and actuation into separate components, or implementing a composite plugin
  with an internal schedule, can reproduce a two-phase schedule. Neither alternative may be omitted
  from the comparison.

## Narrow research question

Can explicit per-cycle data validity and output-commit contracts simplify composition of reusable
bidirectional controllers and prevent stale/partial commands, compared with ordinary chaining and
an internally scheduled composite plugin, at acceptable measured overhead?

This is a hypothesis. Novelty and performance benefits are not established by the current prototype.
No claim of first hierarchical control, automatic controller synthesis, or proved WCET is made.

## Phase graph

For node i define S_i (state production) and C_i (command production). For every parent p and child c:

    S_c -> S_p
    S_i -> C_i
    C_p -> C_c

With a single synchronous tree, postorder S followed by preorder C is a valid topological order.
At the object level, treating each controller as one indivisible update would require both child
before parent and parent before child when the parent needs a child-computed estimate. Separating
phases resolves that scheduling conflict. It does not invent a new class of control laws.

This only guarantees freshness of derived values when all producers succeed, run this cycle, and
publish valid data. Sensor sample age is distinct from callback execution age.

## Data contracts

Each state slot has fixed schema, units, value storage, source sample timestamp, produced cycle,
validity and fault provenance. The child is its sole writer; the parent has a read-only view.
Each reference slot has a sole parent writer and child reader, with produced cycle and validity.
Bindings are resolved and checked outside the real-time loop. No runtime string lookup or topology
construction is needed. Physical command ownership remains with ResourceManager.

First prototype: one synchronous frequency, one root, explicit YAML parents, no cross-tree edges,
no asynchronous callbacks accessing phase storage, no dynamic topology changes while active.
URDF verifies joint existence and anchors; it does not infer algorithms or mandatory control order.

## Output commit and failure

Normal cycle: read -> state phases -> command phases into scratch outputs -> validate -> commit
the complete command vector -> hardware.write(). A failed phase invalidates its dependent outputs.
Scratch output must not alias live hardware command handles before commit.

For the POV chassis, default fault domain is the whole chassis. One failed steering/drive node must
not silently leave the other wheels receiving a mixture of old and new commands. The application
must provide a validated fault action (for example controlled stop), command watchdog and hardware
limits. Zero effort, zero position, holding old commands, and skipping write are not universally safe.

Atomicity means a software command-buffer commit, not simultaneous CAN delivery or physical actuator
motion. Hardware faults during write still require the hardware driver's own fault response.

Lifecycle transitions stay outside phase callbacks. Validate all required children before activation;
activate leaves to root, deactivate root to leaves. If activation partially fails, roll back the
activated subset and report rollback failures. Do not promise transactional hardware behavior.

Isolation and fallback are deferred: a sibling can depend on the failed node through an ancestor.
Do not equate structural siblinghood with physical independence.

## POV validation case

Root chassis -> four logical swerve modules -> eight motor leaves (four steering, four drive).
The module layer is a proposed abstraction; current FineMote Swerve_t is a configuration record,
not independently scheduled code. FineMote POV forward kinematics is an estimator and must not be
silently replaced by a different estimator in only one comparison group.

State path: joint snapshot -> leaf unit conversion -> module state -> chassis estimate.
Command path: one external-command snapshot -> chassis kinematics -> module targets -> leaf output.
Convert ROS radians/radians-per-second to FineMote degrees/degrees-per-second at explicit boundaries.
If leaves run effort PID, simulation must expose effort commands. Velocity-command simulation hides
the low-level loop and is not an equivalent test of the same PID implementation.

## Fair comparisons and stop gates

1. Native Humble chaining, parent directly reading joint state where sufficient.
2. Explicit estimator/command decomposition with supported plumbing; document extra plumbing needed
   in Humble rather than assuming newer state-export APIs exist there.
3. One ordinary composite controller plugin with the same internal two-phase algorithm and buffers.
4. Proposed manager-level phase/data/commit contract.

Use identical control laws, sample snapshots, update rates, fault actions and hardware interfaces.
Do not compare against deliberately misordered flat controllers or deliberately inefficient dynamic
tree traversal. Faster execution is not assumed; two callbacks can increase overhead.

Gate A: reproduce a concrete child-derived-state dependency and fault-induced partial-output case.
Gate B: show an advantage over the composite-plugin baseline in reusable binding validation,
diagnostics, lifecycle handling or reduced handwritten integration, not merely the same outputs.
Gate C: measure deadline misses, execution-time distribution, oldest source-sample age, phase-cycle
mismatches, partial-command commits, allocations and integration changes when adding a module.
Measured maxima are not formal WCET bounds. Gazebo is for functionality, not hardware RT proof.

If the composite-plugin baseline meets requirements with similar effort and guarantees, stop changing
controller_manager and deliver a reusable composite-controller library instead.

## Current state

v1 remains an experimental tree planner and dispatch skeleton. It is not a production execution
contract: its executor continues command phases after state errors, accepts independently supplied
plans/controller arrays, and runs legacy update in the state pass. It does not implement the output
commit, validity, binding, lifecycle or manager integration described here. Do not use it on hardware.

## Local evidence

- controller_manager/src/controller_manager.cpp: configure_controller sorting near 906;
  update/error handling near 2178; branched comparator near 2633.
- controller_interface/src/chainable_controller_interface.cpp: update near 34.
- controller_interface/include/controller_interface/chainable_controller_interface.hpp: exports near 59.
- controller_manager/src/ros2_control_node.cpp: read/update/write loop.

These establish the Humble baseline only. A broader literature review and newer upstream comparison
are still required before claiming an academic contribution.
