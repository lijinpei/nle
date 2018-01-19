#include "Bracket.hpp"
#include "NodeLoopEquivalenceInfo.hpp"

#include "llvm/Support/Allocator.h"

using namespace llvm;

int main() {
  llvm::MallocAllocator allocator;
  llvm::NLE_internal::BracketList<NodeLoopEquivalenceTraits<Function>> bracket_list(allocator);
}
