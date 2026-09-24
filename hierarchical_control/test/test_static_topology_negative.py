#!/usr/bin/env python3
"""Verify the compile-time interface guarantees, and that the check itself is falsifiable.

Two static guarantees are claimed, and both can only be tested by COMPILING files:

  A. topology acyclicity (static_topology.hpp) -- a hierarchy containing a cycle is rejected;
  B. dimensional consistency (dimensional_interfaces.hpp) -- a dimension mismatch between a
     command and a reference, or between an exported and a consumed state, is rejected.
  C. port ownership (topology_contract.hpp) -- a port whose "<owner>/" prefix is not a controller
     in the topology is rejected, which is the defect class upstream accepts silently.

This script compiles a small corpus and asserts each file's outcome:

    must_compile_control.cpp              -> must COMPILE (control: keeps the test falsifiable)
    compile_fail_self_loop.cpp            -> must FAIL (topology)
    compile_fail_two_cycle.cpp            -> must FAIL (topology)
    compile_fail_three_cycle.cpp          -> must FAIL (topology)
    compile_fail_dimension_reference.cpp  -> must FAIL (dimension)
    compile_fail_dimension_state.cpp      -> must FAIL (dimension)
    compile_fail_owner_unknown.cpp        -> must FAIL (ownership)
    compile_fail_port_unqualified.cpp     -> must FAIL (ownership)

If a "compile_fail" file ever starts compiling, a guarantee has regressed and this test fails.
If the control file stops compiling, the test infrastructure is broken and this test also fails --
without the control, a checker that rejected everything would look like a pass.

Usage:
    python3 hierarchical_control/test/test_static_topology_negative.py
"""
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
NEG_DIR = os.path.join(HERE, "static_topology_negative")
INCLUDE_DIR = os.path.abspath(os.path.join(HERE, os.pardir, "include"))


def build_flags(compiler):
    """Reuse the include paths colcon already resolved for this package.

    The corpus includes controller_interface, so a hand-written -I list is not enough; taking the
    flags from the build tree keeps the corpus compiling exactly as the tests do. Falls back to the
    package include dir when the build tree is absent.
    """
    flags_make = os.path.abspath(
        os.path.join(HERE, os.pardir, os.pardir, "build", "hierarchical_control",
                     "CMakeFiles", "test_contract_regression.dir", "flags.make")
    )
    if not os.path.isfile(flags_make):
        return [f"-I{INCLUDE_DIR}"]
    with open(flags_make) as handle:
        for line in handle:
            if line.startswith("CXX_INCLUDES"):
                return line.split("=", 1)[1].split()
    return [f"-I{INCLUDE_DIR}"]

# filename -> (expected to compile?, substring that the diagnostic must contain)
CORPUS = {
    "must_compile_control.cpp": (True, None),
    "compile_fail_self_loop.cpp": (False, "static_topology: CYCLE"),
    "compile_fail_two_cycle.cpp": (False, "static_topology: CYCLE"),
    "compile_fail_three_cycle.cpp": (False, "static_topology: CYCLE"),
    "compile_fail_dimension_reference.cpp": (False, "dimensional_interfaces: DIMENSION MISMATCH"),
    "compile_fail_dimension_state.cpp": (False, "dimensional_interfaces: DIMENSION MISMATCH"),
    "compile_fail_owner_unknown.cpp": (False, "OWNERSHIP VIOLATION"),
    "compile_fail_port_unqualified.cpp": (False, "OWNERSHIP VIOLATION"),
}


def find_compiler():
    env = os.environ.get("CXX")
    if env:
        return env
    for candidate in ("g++", "c++", "clang++"):
        path = shutil.which(candidate)
        if path:
            return path
    return None


def normalize_whitespace(text):
    """Collapse runs of whitespace so a diagnostic that wraps across lines still matches.

    Compilers wrap long static_assert messages; the checked phrase may straddle a newline.
    """
    return " ".join(text.split())


def try_compile(compiler, source, extra_flags):
    cmd = [compiler, "-std=c++17", f"-I{NEG_DIR}", "-fsyntax-only", source] + extra_flags
    proc = subprocess.run(cmd, capture_output=True, text=True)
    return proc.returncode == 0, proc.stderr


def main():
    compiler = find_compiler()
    if compiler is None:
        print("SKIP: no C++ compiler found (set CXX to override)")
        return 0

    flags = build_flags(compiler)
    print(f"compiler: {compiler}  ({len(flags)} include paths from the build tree)")
    failures = []
    for name, (expect_compile, expected_diagnostic) in sorted(CORPUS.items()):
        source = os.path.join(NEG_DIR, name)
        if not os.path.isfile(source):
            failures.append(f"{name}: file missing")
            print(f"  {name:38s} MISSING")
            continue
        compiled, stderr = try_compile(compiler, source, flags)
        ok = compiled == expect_compile
        status = "compiled" if compiled else "rejected"
        expected = "compile" if expect_compile else "reject"
        print(f"  {name:38s} {status:9s} (expected {expected:7s}) {'OK' if ok else 'FAIL'}")
        if not ok:
            failures.append(f"{name}: got {status}, expected {expected}")
        # For expected failures, confirm the rejection came from the intended static_assert and not
        # from an unrelated compile error.
        if not expect_compile and not compiled and expected_diagnostic is not None:
            if expected_diagnostic not in normalize_whitespace(stderr):
                failures.append(
                    f"{name}: rejected, but not by the expected diagnostic "
                    f"({expected_diagnostic!r})"
                )
                print(f"      !! rejection did not mention {expected_diagnostic!r}")

    if failures:
        print("\nFAILED:")
        for f in failures:
            print(f"  - {f}")
        return 1

    print("\nAll corpus entries behaved as specified: the static guarantee holds, and the control")
    print("file proves the check is not vacuous.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
