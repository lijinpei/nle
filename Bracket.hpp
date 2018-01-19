#ifndef BRACKET_HPP
#define BRACKET_HPP

#include "idlist.hpp"
#include "NodeEdge.hpp"
#include "NodeLoopEquivalenceInfo.hpp"

namespace llvm {
namespace NLE_internal {

/* This class represents a bracket in the list of brackets of the node this
 * bracket targeting to */
template <class NLET>
class Bracket : public idlist<Bracket<NLET>> {
  template <class, class> friend class BracketList;
  using EdgeT = typename NLET::EdgeT;
  EdgeT Edge;
  Bracket(EdgeT Edge_, idlist<Bracket> & prev) : Edge(Edge_) {
    insertAfter(prev);
  }
public:
  ~Bracket() {
    // unlink ourself from the list
    this->dropFromList();
  }

  EdgeT getEdge() { return Edge; }
};

// implemented as a double-linked list
template <class NLET, class AllocatorT = MallocAllocator>
class BracketList {
public:
  using BracketT = Bracket<NLET>;
  using IdlistT = idlist<BracketT>;
  using DeleteHandleT = BracketT*;
  using NodeT = typename NLET::NodeT;
  using EdgeT = typename NLET::EdgeT;
  using MarkerT = typename NLET::MarkerT;
private:
  IdlistT Head, Tail;
  AllocatorT & Allocator;
  size_t Size;
public:
  BracketList(AllocatorT & Allocator_) : Allocator(Allocator_), Size(0) {
    IdlistT::Initiate(Head, Tail);
  }

  DeleteHandleT PushEdge(NodeT CNode, bool IsSplitEdge) {
    DeleteHandleT ret = static_cast<DeleteHandleT>(Allocator.allocate(sizeof(BracketT)));
    new (ret) BracketT(CNode, IsSplitEdge, Head);
    ++Size;
    return ret;
  }

  /* See NodeLoopEquivalence's comment for MarkerT */
  MarkerT getMarker() {
    if (size()) {
      return std::make_pair(Head.next->getEdge().getChildNode(), size());
    } else {
      return NLET::NullMarker();
    }
  }

  void merge(BracketList & RHS) {
    Size += RHS.GetSize();
    RHS.size = 0;
    IdlistT::merge(Head, Tail, RHS.Head, RHS.Tail);
  }

  // Don't use the return value of this method when the bracket list is empty
  EdgeT front() {
    return static_cast<BracketT*>(Head.next)->getEdge();
  }

  size_t size() { return Size; }

  template <class Container>
  void drop(Container & DHs) {
    for (auto dh :  DHs) {
      dh->~BracketListNodeT();
      Allocator.deallocate(dh, sizeof(BracketT));
      --Size;
    }
    DHs.clear();
  }
};

}  // namespace NLE_internal
}  // namespace llvm

#endif  // BRACKET_HPP
