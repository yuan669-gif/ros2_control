// MUST FAIL: reusing a grandparent closes a 3-cycle (root -> mid -> leaf -> root).
#include "_common.hpp"
using three_cycle = neg::st::Descendant<neg::root_name, neg::leaf>;
int main() { return 0; }
