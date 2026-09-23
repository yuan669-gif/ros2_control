// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#include "case_study/travel_registry.hpp"

#include <mutex>
#include <unordered_map>

namespace case_study
{
namespace
{
std::mutex & registry_mutex()
{
  static std::mutex mutex;
  return mutex;
}

std::unordered_map<std::string, TravelSource *> & registry()
{
  static std::unordered_map<std::string, TravelSource *> map;
  return map;
}
}  // namespace

void TravelRegistry::register_source(const std::string & name, TravelSource * source)
{
  std::lock_guard<std::mutex> guard(registry_mutex());
  registry()[name] = source;
}

void TravelRegistry::unregister_source(const std::string & name)
{
  std::lock_guard<std::mutex> guard(registry_mutex());
  registry().erase(name);
}

TravelSource * TravelRegistry::find(const std::string & name)
{
  std::lock_guard<std::mutex> guard(registry_mutex());
  const auto it = registry().find(name);
  return it == registry().end() ? nullptr : it->second;
}

}  // namespace case_study
