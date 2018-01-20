#include "NodeEdge.hpp"

#include "llvm/ADT/DenseMap.h"
#include "llvm/Analysis/RegionInfo.h"

using namespace llvm;

int main() {
  using NodeT = llvm::NLE_internal::Node<RegionTraits<Function>::BlockT>;
  DenseMap<NodeT, size_t> map;
}
