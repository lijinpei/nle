#include "NodeLoopEquivalence.hpp"

#include "llvm/IR/Function.h"
#include "llvm/IR/CFG.h"

using namespace llvm;

int main() {
  NodeLoopEquivalence<Function> nle(*reinterpret_cast<Function*>(main));
}
