#ifndef NODELOOPEQUIVALENCEINFO_HPP
#define NODELOOPEQUIVALENCEINFO_HPP

#include "NodeEdge.hpp"

#include "llvm/ADT/GraphTraits.h"
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
  /* We use the size of the bracket list, and top bracket's descendant node as the marker
   * this is different from the original paper(which use the size and both two node of the bracket as the marker)
   */
  using MarkerT = std::pair<NodeT, size_t>;
  // NullMarker is returned when a basic block doesn't have marker
  static MarkerT NullMarker() {
    return {{{}, {}}, {}};
  }
  static bool IsNullMarker(const MarkerT & Ma) {
    return !Ma.first.getBlock();
  }
  using TimeStepT = size_t;
  using SuccItorT = typename GraphTraits<const BlockT*>::ChildIteratorType;
  using PredItorT = typename GraphTraits<Inverse<const BlockT*>>::ChildIteratorType;
};

}

#endif  // NODELOOPEQUIVALENCEINFO_HPP
