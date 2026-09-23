// Shared name types for the negative corpus.
#pragma once
#include "hierarchical_control/static_topology.hpp"
namespace neg
{
namespace st = hierarchical_control::static_topology;
struct root_name { static constexpr auto value = st::NameOf("root"); };
struct mid_name  { static constexpr auto value = st::NameOf("mid"); };
struct leaf_name { static constexpr auto value = st::NameOf("leaf"); };
using root = st::Root<root_name>;
using mid  = st::Descendant<mid_name, root>;
using leaf = st::Descendant<leaf_name, mid>;
}  // namespace neg
