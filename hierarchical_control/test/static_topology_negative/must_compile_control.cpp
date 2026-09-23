// CONTROL: this file must COMPILE. It exists so the negative test is falsifiable -- if the checker
// reported "rejected" for everything, this file would expose it.
#include "_common.hpp"
struct extra_name { static constexpr auto value = neg::st::NameOf("extra"); };
using extra = neg::st::Descendant<extra_name, neg::leaf>;
int main() { return 0; }
