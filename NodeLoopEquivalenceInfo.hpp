#ifndef NODELOOPEQUIVALENCEINFO_HPP
#define NODELOOPEQUIVALENCEINFO_HPP

#include "NodeEdge.hpp"

#include "llvm/Analysis/RegionInfo.h"
#include "llvm/IR/Function.h"

#include <utility>

namespace llvm {

template <class C>
struct NodeLoopEquivalenceTraits {};

template <>
struct NodeLoopEquivalenceTraits<Function> : public RegionTraits<Function> {
  using NodeT = NLE_internal::Node<BlockT>;
  using EdgeT = NLE_internal::Edge<BlockT>;
  using MarkerT = std::pair<NodeT, size_t>;
  // NullMarker is returned when a basic block doesn't have marker
  static MarkerT NullMarker() {
    return {{{}, {}}, {}};
  }
  static bool IsNullMarker(const MarkerT & Ma) {
    return !Ma.first.getBlock();
  }
};

}

#endif  // NODELOOPEQUIVALENCEINFO_HPP
