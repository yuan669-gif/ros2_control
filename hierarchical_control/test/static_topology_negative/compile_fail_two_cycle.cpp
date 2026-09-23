// MUST FAIL: reusing the parent as a descendant closes a 2-cycle (root -> mid -> root).
#include "_common.hpp"
using two_cycle = neg::st::Descendant<neg::root_name, neg::mid>;
int main() { return 0; }
