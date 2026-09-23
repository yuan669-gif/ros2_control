// Copyright 2026
// Licensed under the Apache License, Version 2.0.
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "controller_manager/cycle_tree.hpp"

namespace ct = controller_manager::cycle_tree;

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

struct Component : ct::Node
{
  bool fail_state = false, fail_command = false, stale = false, omit = false;
  int state_calls = 0, command_calls = 0;
  bool state(const ct::Context & c, const std::vector<ct::Value> & inputs,
    ct::Value & output) noexcept override
  {
    ++state_calls;
    if (fail_state) {return false;}
    output = {0.0, stale ? c.cycle - 1 : c.cycle, inputs[0].sample_ns, true};
    for (const auto & value : inputs)
    {
      if (!value.valid || value.cycle != c.cycle) {return false;}
      output.value += value.value;
      output.sample_ns = std::min(output.sample_ns, value.sample_ns);
    }
    // Simple derived estimate, deliberately identical in every comparison implementation.
    output.value *= 2.0;
    return true;
  }
  bool command(const ct::Context & c, const ct::Value & state,
    const ct::Value & reference, ct::OutputView children,
    double & actuator) noexcept override
  {
    ++command_calls;
    if (fail_command || state.cycle != c.cycle || reference.cycle != c.cycle) {return false;}
    if (omit) {return true;}
    const auto value = reference.value - state.value;
    for (std::size_t i = 0; i < children.size(); ++i) {children[i] = value;}
    actuator = value;
    return true;
  }
};

template<typename F>
void rejects(F f)
{
  bool rejected = false;
  try {f();} catch (const std::invalid_argument &) {rejected = true;}
  require(rejected, "invalid binding accepted");
}

int main()
{
  try
  {
    Component root, module, a, b;
    ct::Executor executor({{"root", &root}, {"module", &module}, {"a", &a}, {"b", &b}},
      {{"root", "module"}, {"module", "a"}, {"module", "b"}});
    std::vector<ct::Value> snapshot{{1.0, 0, 99, true}, {3.0, 0, 98, true}};
    for (int cycle = 1; cycle <= 1000; ++cycle)
    {
      snapshot[0].value = static_cast<double>(cycle);
      const auto result = executor.run(100, 1, 2, snapshot, 100.0);
      require(result.status == ct::Status::committed, "valid cycle rejected");
      // Handwritten composite baseline: identical estimator and command equations.
      const double sa = 2.0 * snapshot[0].value, sb = 2.0 * snapshot[1].value;
      const double sm = 2.0 * (sa + sb), sr = 2.0 * sm;
      const double module_reference = 100.0 - sr;
      const double leaf_reference = module_reference - sm;
      require(executor.committed_commands()[0] == leaf_reference - sa, "leaf a delayed");
      require(executor.committed_commands()[1] == leaf_reference - sb, "leaf b delayed");
      require(executor.committed_cycle() == result.cycle, "wrong commit cycle");
    }
    const auto previous = executor.committed_commands();
    const auto previous_cycle = executor.committed_cycle();
    Component * components[] = {&root, &module, &a, &b};
    for (auto * component : components)
    {
      component->fail_state = true;
      const auto commands_before = root.command_calls + module.command_calls + a.command_calls + b.command_calls;
      require(executor.run(100, 1, 2, snapshot, 7).status == ct::Status::state_failed,
        "state failure not reported");
      require(commands_before == root.command_calls + module.command_calls + a.command_calls + b.command_calls,
        "command executed after state failure");
      component->fail_state = false;
      component->fail_command = true;
      require(executor.run(100, 1, 2, snapshot, 7).status == ct::Status::command_failed,
        "command failure not reported");
      component->fail_command = false;
      require(executor.committed_commands() == previous, "partial command commit");
      require(executor.committed_cycle() == previous_cycle, "failed cycle marked committed");
    }
    module.stale = true;
    require(executor.run(100, 1, 2, snapshot, 7).status == ct::Status::state_failed,
      "stale derived state accepted");
    module.stale = false;
    b.omit = true;
    require(executor.run(100, 1, 2, snapshot, 7).status == ct::Status::command_failed,
      "missing output accepted");
    b.omit = false;
    snapshot[0].sample_ns = 97;
    require(executor.run(100, 1, 2, snapshot, 7).status == ct::Status::invalid_input,
      "old sensor sample accepted");
    snapshot[0].sample_ns = 101;
    require(executor.run(100, 1, 2, snapshot, 7).status == ct::Status::invalid_input,
      "future sensor sample accepted");
    snapshot[0].sample_ns = 99;
    snapshot[0].valid = false;
    require(executor.run(100, 1, 2, snapshot, 7).status == ct::Status::invalid_input,
      "invalid sensor sample accepted");
    snapshot[0].valid = true;
    require(executor.run(100, 1, 2, {}, 7).status == ct::Status::invalid_input,
      "missing snapshot accepted");
    require(executor.run(100, 0, 2, snapshot, 7).status == ct::Status::invalid_input,
      "zero period accepted");
    require(executor.run(100, 1, 2, snapshot, std::numeric_limits<double>::quiet_NaN()).status ==
      ct::Status::invalid_input, "NaN reference accepted");
    require(executor.run(100, 1, 2, snapshot, 7).status == ct::Status::committed,
      "recovery failed");

    // Declaration order differs from execution order; result must not depend on registration order.
    Component r2, m2, a2, b2;
    ct::Executor reordered({{"b", &b2}, {"module", &m2}, {"root", &r2}, {"a", &a2}},
      {{"module", "b"}, {"root", "module"}, {"module", "a"}});
    std::vector<ct::Value> reversed{snapshot[1], snapshot[0]};
    require(reordered.run(100, 1, 2, reversed, 7).status == ct::Status::committed,
      "reordered graph rejected");
    require(reordered.committed_commands()[0] == executor.committed_commands()[1] &&
      reordered.committed_commands()[1] == executor.committed_commands()[0],
      "binding order changed result");

    ct::Executor shallow({{"root", &r2}, {"a", &a2}}, {{"root", "a"}});
    require(shallow.run(100, 1, 2, {{3, 0, 99, true}}, 20).status == ct::Status::committed,
      "two-layer case failed");
    require(shallow.committed_commands()[0] == 2, "two-layer output delayed");
    rejects([&] {ct::Executor x({{"r", &root}, {"a", &a}}, {});});
    rejects([&] {ct::Executor x({{"r", &root}}, {{"r", "missing"}});});
    rejects([&] {ct::Executor x({{"r", &root}, {"a", &a}}, {{"r", "a"}, {"a", "r"}});});
    rejects([&] {ct::Executor x({{"r", &root}, {"a", &a}}, {{"r", "a"}, {"r", "a"}});});
    rejects([&] {ct::Executor x({{"r", &root}, {"a", &root}}, {{"r", "a"}});});
    rejects([&] {ct::Executor x({{"r", nullptr}}, {});});
    std::cout << "PASS: 1000 same-cycle composite comparisons; two/three layers; "
      "state/command faults at every node; stale/missing data; recovery; binding validation\n";
    return EXIT_SUCCESS;
  }
  catch (const std::exception & e)
  {
    std::cerr << "FAIL: " << e.what() << '\n';
    return EXIT_FAILURE;
  }
}
