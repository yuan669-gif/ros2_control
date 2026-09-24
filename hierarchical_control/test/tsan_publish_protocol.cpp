// Copyright 2026
// Licensed under the Apache License, Version 2.0.

// ThreadSanitizer harness for the membership publish protocol (review R8).
//
// The reviewed revision kept the two-phase membership set in a plain
// `std::vector<TwoPhaseEntry>` that the idle thread rebuilt while `update()` read it on the control
// thread. A `shared_ptr`'s refcount being thread-safe does not make assignment to the same
// `shared_ptr` object safe, and a plain vector has no synchronization at all.
//
// This harness runs BOTH patterns from the same shape of two threads:
//
//   --mode=racy     plain vector written by the publisher, read by the control thread
//   --mode=atomic   immutable snapshot published with std::atomic_store, read with std::atomic_load
//
// `racy` MUST be reported and `atomic` MUST be clean, and the runner script asserts both. That is
// what keeps the "clean" result from being vacuous: the same harness demonstrably detects the
// pattern the fix removed.
//
// Build and run with test/run_tsan_publish_protocol.sh (TSan needs `setarch -R` on this machine
// because the default ASLR entropy is incompatible with its shadow layout).

#include <atomic>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <memory>
#include <thread>
#include <vector>

namespace
{

/// Same shape as `ControllerManager::TwoPhaseEntry`: pointers the reader dereferences.
struct Entry
{
  const void * base;
  void * instance;
};

/// The reviewed pattern: one plain vector, written by the publisher and read by the control thread.
std::vector<Entry> g_plain;

/// The current pattern: an immutable snapshot swapped in with a single atomic store.
std::shared_ptr<const std::vector<Entry>> g_published;

std::atomic<bool> g_stop{false};

std::vector<Entry> make_entries(std::size_t round)
{
  std::vector<Entry> entries;
  entries.reserve(4);
  for (std::size_t i = 0; i < 4; ++i)
  {
    entries.push_back(Entry{reinterpret_cast<const void *>(round + i + 1), nullptr});
  }
  return entries;
}

/// What the control thread does with a membership set: walk it and dereference the entries.
std::size_t read_snapshot(const std::vector<Entry> & entries)
{
  std::size_t seen = 0;
  for (const auto & entry : entries)
  {
    if (entry.base != nullptr) {++seen;}
  }
  return seen;
}

int run_racy(int iterations)
{
  std::thread publisher(
    [iterations]
    {
      for (int round = 0; round < iterations; ++round)
      {
        g_plain = make_entries(static_cast<std::size_t>(round));
      }
      g_stop.store(true, std::memory_order_release);
    });

  std::size_t total = 0;
  while (!g_stop.load(std::memory_order_acquire)) {total += read_snapshot(g_plain);}
  publisher.join();
  std::printf("[tsan] racy mode walked %zu entries\n", total);
  return 0;
}

int run_atomic(int iterations)
{
  std::thread publisher(
    [iterations]
    {
      for (int round = 0; round < iterations; ++round)
      {
        auto next =
          std::make_shared<const std::vector<Entry>>(make_entries(static_cast<std::size_t>(round)));
        std::atomic_store(&g_published, next);
      }
      g_stop.store(true, std::memory_order_release);
    });

  std::size_t total = 0;
  while (!g_stop.load(std::memory_order_acquire))
  {
    // One atomic load per cycle; the local shared_ptr keeps the snapshot alive for the whole walk,
    // exactly as `ControllerManager::update()` does.
    auto snapshot = std::atomic_load(&g_published);
    if (snapshot) {total += read_snapshot(*snapshot);}
  }
  publisher.join();
  std::printf("[tsan] atomic mode walked %zu entries\n", total);
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  constexpr int kIterations = 20000;
  if (argc > 1 && std::strcmp(argv[1], "--mode=racy") == 0) {return run_racy(kIterations);}
  if (argc > 1 && std::strcmp(argv[1], "--mode=atomic") == 0) {return run_atomic(kIterations);}
  std::fprintf(stderr, "usage: %s --mode=racy|--mode=atomic\n", argv[0]);
  return 2;
}
