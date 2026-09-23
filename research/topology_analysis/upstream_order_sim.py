#!/usr/bin/env python3
"""Faithful port of the upstream ros2_control controller ordering heuristic.

Ported from ros-controls/ros2_control master:
  controller_manager/src/controller_manager.cpp
    - build_controllers_topology_info()      (chain graph construction)
    - update_list_with_controller_chain()    (order construction, recursive insertion)

Purpose: determine what happens when one controller pair has BOTH a reference edge
(parent claims child/ref) and a state edge (parent claims child/state).

This is a port, not the original code. Line references are in doc/BIDIRECTIONAL_EDGE_ANALYSIS.md.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class Controller:
    name: str
    cmd_itfs: list[str] = field(default_factory=list)    # command_interface_configuration().names
    state_itfs: list[str] = field(default_factory=list)  # state_interface_configuration().names


class Manager:
    def __init__(self, controllers: list[Controller]):
        self.controllers = controllers
        self.names = [c.name for c in controllers]
        self.following: dict[str, list[str]] = {c.name: [] for c in controllers}
        self.preceding: dict[str, list[str]] = {c.name: [] for c in controllers}
        self.ordered: list[str] = []

    @staticmethod
    def _owner(interface: str, names: list[str]) -> str | None:
        prefix = interface.split("/", 1)[0]
        return prefix if prefix in names else None

    def build_topology(self) -> None:
        """Port of build_controllers_topology_info()'s chain construction."""
        for ctrl in self.controllers:
            for itf in ctrl.cmd_itfs:
                owner = self._owner(itf, self.names)
                if owner is None:
                    continue
                # reference edge: this controller produces, owner consumes
                self.following[ctrl.name].append(owner)      # owner follows this controller
                self.preceding[owner].append(ctrl.name)      # this controller precedes owner
            for itf in ctrl.state_itfs:
                owner = self._owner(itf, self.names)
                if owner is None:
                    continue
                # state edge: owner produces state, this controller consumes
                self.preceding[ctrl.name].append(owner)      # owner precedes this controller
                self.following[owner].append(ctrl.name)      # this controller follows owner

    def update_chain(self, ctrl_name: str, iterator: int, append_to_controller: bool) -> None:
        """Port of update_list_with_controller_chain(). `iterator` is an index into self.ordered."""
        if ctrl_name in self.ordered:
            return  # first insertion wins; later constraints are ignored
        for ctrl in self.following[ctrl_name]:
            if ctrl in self.ordered:
                pos = self.ordered.index(ctrl)
                if pos < iterator:
                    iterator = pos
        for ctrl in self.preceding[ctrl_name]:
            if ctrl in self.ordered:
                pos = self.ordered.index(ctrl)
                if pos > iterator:
                    iterator = pos
        insert_at = iterator + 1 if append_to_controller else iterator
        self.ordered.insert(insert_at, ctrl_name)
        for flwg in self.following[ctrl_name]:
            self.update_chain(flwg, self.ordered.index(ctrl_name), True)
        for preced in self.preceding[ctrl_name]:
            self.update_chain(preced, self.ordered.index(ctrl_name), False)

    def order(self) -> list[str]:
        self.ordered = []
        for ctrl in self.controllers:
            if ctrl.name not in self.ordered:
                self.update_chain(ctrl.name, len(self.ordered), False)
        return list(self.ordered)


def analyse(title: str, controllers: list[Controller], edges: list[tuple[str, str, str]]) -> None:
    """edges: (parent, child, kind) with kind in {'ref', 'state'}."""
    mgr = Manager(controllers)
    mgr.build_topology()
    order = mgr.order()
    pos = {name: i for i, name in enumerate(order)}
    print(f"### {title}")
    print(f"  load order            : {[c.name for c in controllers]}")
    print(f"  following_controllers : {mgr.following}")
    print(f"  preceding_controllers : {mgr.preceding}")
    print(f"  resulting run order   : {order}")
    for parent, child, kind in edges:
        if kind == "ref":
            fresh = pos[parent] < pos[child]        # parent first => child sees fresh reference
            print(
                f"  edge {parent}->{child} [reference] : "
                f"{'same-cycle (fresh)' if fresh else 'ONE-CYCLE DELAY'}")
        else:
            fresh = pos[child] < pos[parent]        # child first => parent sees fresh state
            print(
                f"  edge {child}->{parent} [state]     : "
                f"{'same-cycle (fresh)' if fresh else 'ONE-CYCLE DELAY'}")
    print()


def two_node(load_order: list[str]) -> None:
    controllers = [Controller("A", cmd_itfs=["B/ref"], state_itfs=["B/state"])
                   for _ in range(0)]
    by_name = {
        "A": Controller("A", cmd_itfs=["B/ref"], state_itfs=["B/state"]),
        "B": Controller("B"),
    }
    analyse(
        f"Two controllers, bidirectional edge, load order {load_order}",
        [by_name[n] for n in load_order],
        [("A", "B", "ref"), ("A", "B", "state")],
    )


def three_node(load_order: list[str]) -> None:
    by_name = {
        "r": Controller("r", cmd_itfs=["m/ref"], state_itfs=["m/state"]),
        "m": Controller("m", cmd_itfs=["l/ref"], state_itfs=["l/state"]),
        "l": Controller("l"),
    }
    analyse(
        f"Three-level chain, bidirectional at each level, load order {load_order}",
        [by_name[n] for n in load_order],
        [("r", "m", "ref"), ("r", "m", "state"),
         ("m", "l", "ref"), ("m", "l", "state")],
    )


if __name__ == "__main__":
    print("Port of upstream ros2_control ordering heuristic (see module docstring)\n")
    two_node(["A", "B"])
    two_node(["B", "A"])
    three_node(["r", "m", "l"])
    three_node(["l", "m", "r"])
    three_node(["m", "r", "l"])
