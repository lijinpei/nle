#ifndef NODELOOPEQUIVALENCE_HPP
#define NODELOOPEQUIVALENCE_HPP

#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/GraphTraits.h"
#include "llvm/ADT/PointerIntPair.h"

#include <forward_list>
#include <type_traits>

namespace llvm {

template <class GraphType, class AllocatorT = MallocAllocator>
class NodeLoopEquivalence {
public:
  enum NodeKind : bool { NK_Exit = false, NK_Entry = true };
  enum EdgeKind : bool { EK_NonSplitEdge = false, EK_SplitEdge = true };
  using FuncT = std::remove_cv_t<std::remove_reference_t<GraphType>>;
  using NodeRef = typename GraphTraits<const FuncT*>::NodeRef;
  using BlockT = std::remove_cv_t<std::remove_pointer_t<NodeRef>>;
  using SuccItorT = typename GraphTraits<NodeRef>::ChildIteratorType;
  using PredItorT = typename GraphTraits<Inverse<NodeRef>>::ChildIteratorType;
  template <NodeKind NK>
    using ItorT = std::conditional_t<NK == NK_Entry, PredItorT, SuccItorT>;
  using NodeT = PointerIntPair<NodeRef, 1, NodeKind>;
  using EdgeT = PointerIntPair<NodeT, 1, EdgeKind>;
  using TimeStepT = size_t;

  static NodeT getPal(NodeT N) {
    NodeKind NK = N.getInt() == NK_Exit ? NK_Entry : NK_Exit;
    return {N.getPointer(), NK};
  }

  static NodeRef getBB(NodeT N) { return N.getPointer(); }

  static NodeRef getChildBB(EdgeT E) { return getBB(E.getPointer()); }

  static bool isEntry(NodeT N) { return N.getInt() == NK_Entry; }

  static bool childIsEntry(EdgeT E) { return isEntry(E.getPointer()); }

  static bool isSplitEdge(EdgeT E) { return E.getInt() == EK_SplitEdge; }

  static bool isNullEdge(EdgeT E) { return !E.getOpaqueValue(); }

  static void setNullEdge(EdgeT &E) { E.setFromOpaqueValue(nullptr); }

  static EdgeT childNodePalEdge(EdgeT &E) {
    return {getPal(E.getPointer()), EK_SplitEdge};
  }

  struct Bracket {
    EdgeT Edge;
    Bracket * Next;
  };

  Bracket* newBracket(EdgeT E) {
    auto* NB = reinterpret_cast<Bracket*>(Allocator.template Allocate<Bracket>(1));
    NB->Edge = E;
    return NB;
  }

  using MarkerT = std::pair<NodeT, size_t>;

  struct DFSNode {
    EdgeT Edge;
    TimeStepT HighPt;
    Bracket *Head, *Tail;
    size_t Size;
    std::forward_list<Bracket*> BracketsToDelete;

    DFSNode(EdgeT E)
        : Edge(E), HighPt(0), Head(nullptr), Tail(nullptr), Size(0) {}

    void mergeDFSNode(DFSNode & Child, TimeStepT & HighPt2) {
      size_t s = Child.HighPt;
      if (s > HighPt) {
        std::swap(s, HighPt);
      }
      if (s > HighPt2) {
        HighPt2 = s;
      }
      if (Tail) {
        Tail->Next = Child.Head;
        Tail = Child.Tail;
      } else {
        Head = Child.Head;
        Tail = Child.Tail;
      }
      Size += Child.Size;
      Child.Size = 0;
      Child.Head = nullptr;
      Child.Tail = nullptr;
    }

    void addBracket(DFSNode & Ancestor, Bracket* B) {
      if (Head) {
        B->Next = Head;
      } else {
        B->Next = nullptr;
        Tail = B;
      }
      Head = B;
      ++Size;
      Ancestor.BracketsToDelete.push_front(B);
    }

    void dropBrackets() {
      for (auto B : BracketsToDelete) {
        setNullEdge(B->Edge);
      }
    }

    MarkerT getMarker() {
      while (Head && isNullEdge(Head->Edge)) {
        Head = Head->Next;
      }
      if (!Head) {
        Tail = nullptr;
        return NullMarker();
      } else {
        return {Head->Edge.getPointer(), Size};
      }
    }

    Bracket* front() {
      while (Head && isNullEdge(Head->Edge)) {
        Head = Head->Next;
      }
      if (!Head) {
        Tail = nullptr;
        return nullptr;
      }
      return Head;
    }
  };

  struct DFSStackNode {
    // Pointer to DFSNumber and whether pal has been visited
    PointerIntPair<TimeStepT*, 1, bool> DFSNumber;
    // child node and whether (parent, child) is a split-edge
    EdgeT Edge;
    // Mark of current visited children
    union {
      SuccItorT Succ;
      PredItorT Pred;
    };

    DFSStackNode(EdgeT Edge_, TimeStepT & TS, bool PalVisited) : DFSNumber(&TS, PalVisited), Edge(Edge_) {
      if (childIsEntry(Edge)) {
        Pred = pred_begin(getChildBB(Edge));
      } else {
        Succ = succ_begin(getChildBB(Edge));
      }
    }

    void setItor(PredItorT P) { Pred = P; }

    void setItor(SuccItorT S) { Succ = S; }

  };

  // Those are utility function for visiting the stack
  template <NodeKind NK>
  static std::enable_if_t<NK == NK_Entry, PredItorT>
  current_pos(DFSStackNode &Node) {
    return Node.Pred;
  }

  template <NodeKind NK>
  static std::enable_if_t<NK == NK_Entry, PredItorT>
  end_pos(DFSStackNode &Node) {
    return pred_end(getChildBB(Node.Edge));
  }

  template <NodeKind NK>
  static std::enable_if_t<NK == NK_Exit, SuccItorT>
  current_pos(DFSStackNode &Node) {
    return Node.Succ;
  }

  template <NodeKind NK>
  static std::enable_if_t<NK == NK_Exit, SuccItorT>
  end_pos(DFSStackNode &Node) {
    return succ_end(getChildBB(Node.Edge));
  }

  static EdgeT getEdge(PredItorT P) {
    return {NodeT(*P, NK_Exit), EK_NonSplitEdge};
  }

  static EdgeT getEdge(SuccItorT S) {
    return {{*S, NK_Entry}, EK_NonSplitEdge};
  }

private:
  const FuncT & Func;
  AllocatorT Allocator;
  using BlockMakerMapT = DenseMap<const BlockT*, MarkerT>;
  BlockMakerMapT BMM;
  DenseMap<NodeT, TimeStepT> DFSNumbers;
  std::vector<DFSNode> DFSQueue;
  std::vector<DFSStackNode> DFSStack;
  TimeStepT BracketLabel;

  template <NodeKind NK>
  void dfsVisitStackTop() {
    auto tryVisit = [&] (EdgeT Edge, bool PalVisited) -> bool {
      auto insert = DFSNumbers.insert({Edge.getPointer(), 0});
      if (insert.second) {
        DFSStack.emplace_back(Edge, insert.first->second, PalVisited);
        return true;
      }
      return false;
    };
    DFSStackNode & tos = DFSStack.back();
    if (!tos.DFSNumber.getInt()) {
      tos.DFSNumber.setInt(true);
      if (tryVisit(childNodePalEdge(tos.Edge), true)) {
        return;
      }
    }
    for (ItorT<NK> child = current_pos<NK>(tos), end = end_pos<NK>(tos); child != end; ++child) {
      if (tryVisit(getEdge(child), false)) {
        tos.setItor(child);
        return;
      }
    }
    DFSQueue.emplace_back(tos.Edge);
    DFSNumbers[tos.Edge.getPointer()] = DFSQueue.size();
    DFSStack.pop_back();
  }

  // visit all reachable (in the sense that reachable in the undirected graph), generate DFSNumbers and DFSQueue
  void dfsVisit() {
    const size_t splite_node_count = 2 * Func.size() - 1;
    DFSNumbers.clear();
    DFSQueue.clear();
    DFSStack.clear();
    DFSNumbers.reserve(splite_node_count);
    DFSQueue.reserve(splite_node_count);
    DFSStack.reserve(splite_node_count);

    const BlockT & entryBB = Func.getEntryBlock();
    NodeT entry{&entryBB, NK_Exit};
    DFSStack.emplace_back(EdgeT(entry, EK_NonSplitEdge), DFSNumbers[entry], true);
    while (DFSStack.size()) {
      if (childIsEntry(DFSStack.back().Edge)) {
        dfsVisitStackTop<NK_Entry>();
      } else {
        dfsVisitStackTop<NK_Exit>();
      }
    }
    DFSStack.clear();
  }

  void punchMarkerHandleAdjacents(DFSNode &Node, TimeStepT MyNumber,
                                  TimeStepT &HighPt2, TimeStepT &HighPt_tmp) {
    auto visitAdjacent = [&](TimeStepT AdjNumber, EdgeKind EK) {
      DFSNode &Adj = DFSQueue[AdjNumber - 1];
      if (AdjNumber < MyNumber) {
        Node.mergeDFSNode(Adj, HighPt2);
      } else {
        HighPt_tmp = std::max(HighPt_tmp, AdjNumber);
        Node.addBracket(Adj, newBracket({Adj.Edge.getPointer(), EK}));
      }
    };
    NodeT child_node = Node.Edge.getPointer();
    visitAdjacent(DFSNumbers[getPal(child_node)], EK_SplitEdge);
    if (isEntry(child_node)) {
      for (const BlockT *Pred : predecessors(getBB(child_node))) {
        visitAdjacent(DFSNumbers[{Pred, NK_Exit}], EK_NonSplitEdge);
      }
    } else {
      for (const BlockT *Succ : successors(getBB(child_node))) {
        visitAdjacent(DFSNumbers[{Succ, NK_Entry}], EK_NonSplitEdge);
      }
    }
  }

  void punchMarkerFinishVisit(DFSNode &Node, TimeStepT MyNumber, TimeStepT HighPt2, TimeStepT HighPt_tmp) {
    if (HighPt_tmp > Node.HighPt) {
      Node.HighPt = HighPt_tmp;
    }
    if (HighPt2 > MyNumber) {
      auto & Ancestor = DFSQueue[HighPt2 - 1];
      EdgeKind EK = getChildBB(Ancestor.Edge) == getChildBB(Node.Edge) ? EK_SplitEdge : EK_NonSplitEdge;
      Node.addBracket(Ancestor, newBracket({Ancestor.Edge.getPointer(), EK}));
    }
    if (Node.Edge.getInt() == EK_NonSplitEdge) {
      return;
    }
    MarkerT marker = Node.getMarker();
    BMM[getChildBB(Node.Edge)] = marker;
    if (Node.Size == 1) {
      Bracket* F = Node.front();
      if (F->Edge.getInt() == EK_SplitEdge) {
        BMM[getChildBB(F->Edge)] = marker;
      }
    }
  }

  void punchMarkers() {
    BracketLabel = 0;
    for (TimeStepT i = 0, e = DFSQueue.size(); i < e; ++i) {
      DFSNode & dfs_node = DFSQueue[i];
      dfs_node.dropBrackets();
      TimeStepT HighPt_tmp = 0, HighPt2 = 0;
      punchMarkerHandleAdjacents(dfs_node, i + 1, HighPt2, HighPt_tmp);
      punchMarkerFinishVisit(dfs_node, i + 1, HighPt2, HighPt_tmp);
    }
  }

public:

  // valid marker's TimeStep won't be zero
  static NodeT NullNode() { return NodeT(nullptr, NodeKind(false)); }
  static MarkerT NullMarker() { return std::make_pair(NullNode(), 0); }

  static bool isNullMarker(const MarkerT & MA) {
    return !MA.first.getPointer();
  }

  NodeLoopEquivalence(const FuncT & Func_) : Func(Func_) {
    dfsVisit();
    punchMarkers();
  }

  bool isNodeLoopEquivalent(const BlockT* N1, const BlockT* N2) {
    const MarkerT M1 = BMM[N1], M2 = BMM[N2];
    return !isNullMarker(M1) && (M1 == M2);
  }
};

}  // namespace llvm

#endif
