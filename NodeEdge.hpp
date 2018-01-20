#ifndef NODEEDGE_HPP
#define NODEEDGE_HPP

#include "llvm/ADT/DenseMapInfo.h"
#include "llvm/ADT/PointerIntPair.h"
#include "llvm/Support/PointerLikeTypeTraits.h"

namespace llvm {
namespace NLE_internal {

enum NodeKind : bool { NK_Exit = false, NK_Entry = true };
inline NodeKind NKPal(NodeKind NK) {
  return NK == NK_Entry ? NK_Exit : NK_Entry;
}

enum EdgeKind : bool { EK_NonSplitEdge = false, EK_SplitEdge = true };

/* We can not sub class this class in NodeLoopEquivalence because we need to paitial specialize DenseMapInfo for this class */
template <class BlockT>
class Node : public PointerIntPair<const BlockT*, 1, NodeKind> {
public:
  using BaseT = PointerIntPair<const BlockT*, 1, NodeKind>;
  Node(const BlockT *BB, NodeKind NK) : BaseT(BB, NK) {}
  bool isEntry() { return this->getInt() == NK_Entry; }
  const BlockT* getBlock() const { return this->getPointer(); }
  Node getPal() { return {this->getPointer(), NKPal(this->getInt())}; }
private:
  friend class DenseMapInfo<Node>;
  friend class PointerLikeTypeTraits<Node>;
  Node(BaseT B) : BaseT(B) {}
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
  Edge() = default;
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
   using DMI1 = DenseMapInfo<typename T::BaseT>;
public:
   static T getEmptyKey() {
     return {DMI1::getEmptyKey()};
   }
 
   static T getTombstoneKey() {
     return {DMI1::getTombstoneKey()};
   }
 
   static unsigned getHashValue(T V) {
     return DMI1::getHashValue(V);
   }
 
  static bool isEqual(const T &LHS, const T &RHS) {
    return DMI1::isEqual(LHS, RHS);
  }
};

template <typename BlockT>
struct PointerLikeTypeTraits<NLE_internal::Node<BlockT>> {
private:
  using T = NLE_internal::Node<BlockT>;
  using PLTT1 = PointerLikeTypeTraits<typename T::BaseT>;
public:
  static inline void *
  getAsVoidPointer(const T &P) {
    return P.getOpaqueValue();
  }
  static inline T 
  getFromVoidPointer(void *P) {
    return PLTT1::getFromOpaqueValue(P);
  }
  static inline T
  getFromVoidPointer(const void *P) {
    return PLTT1::getFromOpaqueValue(P);
  }
  enum { NumLowBitsAvailable = PLTT1::NumLowBitsAvailable };
};
}  // namespace llvm

#endif  // NODEEDGE_HPP
