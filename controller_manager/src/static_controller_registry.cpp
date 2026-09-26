// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "controller_manager/static_controller_registry.hpp"

#include <stdexcept>
#include <string>
#include <utility>

namespace controller_manager
{

void StaticControllerRegistry::insert(const std::string & type, Entry entry)
{
  if (type.empty())
  {
    throw std::invalid_argument("a static controller type must have a non-empty name");
  }
  if (entries_.find(type) != entries_.end())
  {
    throw std::invalid_argument(
      "static controller type '" + type + "' is already registered");
  }
  entries_.emplace(type, std::move(entry));
}

void StaticControllerRegistry::add_factory(const std::string & type, Factory factory)
{
  if (!factory)
  {
    throw std::invalid_argument(
      "static controller type '" + type + "' needs a factory");
  }
  Entry entry;
  entry.create = std::move(factory);
  insert(type, std::move(entry));
}

bool StaticControllerRegistry::has(const std::string & type) const
{
  return entries_.find(type) != entries_.end();
}

controller_interface::ControllerInterfaceBaseSharedPtr StaticControllerRegistry::create(
  const std::string & type) const
{
  const auto it = entries_.find(type);
  if (it == entries_.end()) {return nullptr;}
  return it->second.create();
}

std::vector<std::string> StaticControllerRegistry::types() const
{
  std::vector<std::string> out;
  out.reserve(entries_.size());
  for (const auto & entry : entries_) {out.push_back(entry.first);}
  return out;
}

bool StaticControllerRegistry::has_manifest(const std::string & type) const
{
  const auto it = entries_.find(type);
  return it != entries_.end() && it->second.has_manifest;
}

std::vector<std::string> StaticControllerRegistry::command_interfaces(
  const std::string & type) const
{
  const auto it = entries_.find(type);
  return it == entries_.end() ? std::vector<std::string>{} : it->second.command_interfaces;
}

std::vector<std::string> StaticControllerRegistry::state_interfaces(
  const std::string & type) const
{
  const auto it = entries_.find(type);
  return it == entries_.end() ? std::vector<std::string>{} : it->second.state_interfaces;
}

}  // namespace controller_manager
