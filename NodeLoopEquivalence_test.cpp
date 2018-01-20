#include "NodeLoopEquivalence.hpp"
#include "NodeLoopEquivalenceInfo.hpp"

#include "llvm/IR/Function.h"

using namespace llvm;

int main() {
  NLE_internal::NodeLoopEquivalence<NodeLoopEquivalenceTraits<Function>> nle(*reinterpret_cast<Function*>(main));
}
