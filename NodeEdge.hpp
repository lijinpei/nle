#ifndef NODEEDGE_HPP
#define NODEEDGE_HPP

#include "llvm/ADT/DenseMapInfo.h"
#include "llvm/ADT/PointerIntPair.h"

namespace llvm {
namespace NLE_internal {

enum NodeKind : bool { NK_Exit = false, NK_Entry = true };
inline NodeKind NKPal(NodeKind NK) {
  return NK == NK_Entry ? NK_Exit : NK_Entry;
}

enum EdgeKind : bool { EK_NonSplitEdge = false, EK_SplitEdge = true };

/* We can not sub class this class in NodeLoopEquivalence because we need to paitial specialize DenseMapInfo for this class */
template <class BlockT>
class Node {
private:
  using UnderLayingT = PointerIntPair<const BlockT*, 1, NodeKind>;
  UnderLayingT Node_;
  static UnderLayingT ToUnderLaying(const Node & N) {
    return N.Node_;
  }
  Node(const UnderLayingT & U) : Node(U.getPointer(), U.getInt()) {}
  friend struct DenseMapInfo<Node>;
public:
  Node(const BlockT *BB, NodeKind NK) : Node_(BB, NK) {}
  bool isEntry() { return Node_.getInt() == NK_Entry; }
  const BlockT* getBlock() const { return Node_.getPointer(); }
  Node getPal() { return {Node_.getPointer(), NKPal(Node_.getInt())}; }
};

template <class BlockT>
class Edge {
private:
  using NodeT = Node<BlockT>;
  // CNode could be the entry or exit node of this Edge
  // The only guarantee is that CNode is the child(or descendant if the edge
  // is a frond) node in the spanning tree of the undirected graph
  PointerIntPair<NodeT, 1, EdgeKind> CNode;
public:
  Edge(NodeT CNode_, EdgeKind EK) : CNode(CNode_, EK) {}
  bool isSplitEdge() { return CNode.getInt() == EK_SplitEdge; }
  NodeT getChildNode() { return CNode.getPointer(); }
  Edge getChildPalEdge() { return {getChildNode().getPal(), EK_SplitEdge}; }
};


}  // namespcae NLE_internal

template <class BlockT>
struct DenseMapInfo<NLE_internal::Node<BlockT>> {
private:
   using T = NLE_internal::Node<BlockT>;
   using DMI1 = DenseMapInfo<typename T::UnderLayingT>;
public:
   static T getEmptyKey() {
     return {DMI1::getEmptyKey()};
   }
 
   static T getTombstoneKey() {
     return {DMI1::getTombstoneKey()};
   }
 
   static unsigned getHashValue(T V) {
     return DMI1::getHashValue(T::ToUnderLaying(V));
   }
 
  static bool isEqual(const T &LHS, const T &RHS) {
    return T::ToUnderLaying(LHS) == T::ToUnderLaying(RHS);
  }
};

}  // namespace llvm

#endif  // NODEEDGE_HPP
