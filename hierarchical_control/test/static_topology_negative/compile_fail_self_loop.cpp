// MUST FAIL: a child reusing its own parent's name is a self-loop.
#include "_common.hpp"
namespace { struct dup_name { static constexpr auto value = neg::st::NameOf("mid"); }; }
using self_loop = neg::st::Descendant<dup_name, neg::mid>;
int main() { return 0; }
