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
  using FuncT = std::remove_cv_t<std::remove_reference_t<<GraphType>>;
  using NodeRef = GraphTraits<const FuncT*>::NodeRef;
  using SuccItorT = GraphTraits<NodeRef>::ChildIteratorType;
  using PredItorT = GraphTraits<Inverse<NodeRef>>::ChildIteratorType;
  using NodeT = PointerIntPair<NodeRef, 1, NodeKind>;
  using EdgeT = PointerIntPair<Node, 1, EdgeKind>;
  using TimeStepT = size_t;

  static NodeRef getBB(NodeT N) {
    return N.getPointer();
  }

  static NodeRef getChildBB(EdgeT E) {
    return getBB(E.getPointer());
  }

  static bool isEntry(NodeT N) {
    return N.getInt() == NK_Entry;
  }

  static bool childIsEntry(EdgeT E) {
    return isEntry(E.getPointer());
  }

  static bool isSplitEdge(EdgeT E) {
    return E.getInt() == EK_SplitEdge;
  }

  static EdgeT childNodePalEdge(Edge & E) {
    return {{getChildBB(E), !childIsEntry(E)}, EK_SplitEdge};
  }

  struct Bracket {
    EdgeT Edge;
    Bracket * Next;
  };

  Bracket* newBracket(Edge E) {
    auto* NB = reinterpret_cast<Bracket*>(Allocator.allocate(sizeof(Bracket)));
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
        B->EdgeLabel = 0;
      }
    }

    MarkerT getMarker() {
      while (Head && !Head->Label_) {
        Head = Head->Next;
      }
      if (!Head) {
        Tail = nullptr;
        return NullMarker;
      } else {
        return {Head->Edge->getPointer(), Size};
      }
    }

    Bracket* front() {
      while (Head && !Head->Label_) {
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

    DFSStackNode(EdgeT Edge_, TimeStepT & TS_, bool PalVisited) : DFSNumber(&TS, PalVisited), Edge(Edge_) {
      if (childIsEntry(Edge)) {
        Pred = pred_begin(getChildBB(Edge));
      } else {
        Succ = succ_begin(getChildBB(Edge));
      }
    }
    // Those are utility function for visiting the stack
    template <NodeKind NK>
    std::enable_if_t<NK == NK_Entry, PredItorT> current_pos() {
      return Pred;
    }

    template <NodeKind NK>
    std::enable_if_t<NK == NK_Entry, PredItorT> end_pos() {
      return pred_end(getChildBB(Edge));
    }

    EdgeT getEdge(PredItorT P) {
      return {{&*P, NK_Exit}, EK_NonSplitEdge};
    }

    void setItor(PredItorT P) { Pred = P; }

    template <NodeKind NK>
    std::enable_if_t<NK == NK_Exit, SuccItorT> current_pos() {
      return Succ;
    }

    template <NodeKind NK>
    std::enable_if_t<NK == NK_Exit, SuccItorT> end_pos() {
      return succ_end(getChildBB(Edge));
    }

    EdgeT getEdge(SuccItorT S) {
      return {{&*S, NK_Entry}, EK_NonSplitEdge};
    }

    void setItor(SuccItorT S) { Succ = S; }

  };

private:
  const FuncT & Func;
  AllocatorT Allocator;
  using BlockMakerMapT = DenseMap<const BlockT*, MarkerT>;
  BlockMakerMapT BMM;
  DenseMap<NodeT, TimeStepT> DFSNumbers;
  std::vector<DFSNodeT> DFSQueue;
  std::vector<DFSStackNodeT> DFSStack;
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
    DFSStackNodeT & tos = DFSStack.back();
    if (!tos.DFSNumber.getInt()) {
      tos.DFSNumber.setInt(true);
      if (tryVisit(childNodePalEdge(tos.Edge), true)) {
        return;
      }
    }
    for (ItorT<NK> child = tos.current_pos<NK>(), end = tos.end_pos<NK>(); child != end; ++child) {
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
    DFSStack[0] = {{entry, EK_NonSplitEdge}, DFSNumbers[entry]};
    // set pal visited
    DFSStack[0].DFSNumber.setInt(true);
    while (DFSStack.size()) {
      if (childIsEntry(DFSStack.back().Edge)) {
        dfsVisitStackTop<NK_Entry>();
      } else {
        dfsVisitStackTop<NK_Exit>();
      }
    }
    DFSStack.clear();
  }

  void punchMarkerHandleAdjacents(DFSNodeT &Node, TimeStepT MyNumber,
                                  TimeStepT &HighPt2, TimeStepT &HighPt_tmp) {
    auto visitAdjacent = [](TimeStepT AdjNumber, EdgeKind EK) {
      DFSNode &Adj = DFSQueue[AdjNumber - 1];
      if (AdjNumber < MyNumber) {
        Node.mergeChild(Adj, HighPt2);
      } else {
        HighPt_tmp = std::max(HighPt_tmp, AdjNumber);
        Node.addBracket(Adj, newBracket({Adj.Edge.getPointer(), EK}));
      }
    };
    NodeT child_node = Node.Edge.getPointer();
    visitAdjacent(DFSNumbers[child_node.getPal()], true);
    if (child_node.isEntry()) {
      for (const BlockT *Pred : predecessors(child_node.getBlock())) {
        visitAdjacent(DFSNumbers[{Pred, NK_Exit}], false);
      }
    } else {
      for (const BlockT *Succ : successors(child_node.getBlock())) {
        visitAdjacent(DFSNumbers[{Succ, NK_Entry}], false);
      }
    }
  }

  void punchMarkerFinishVisit(DFSNodeT &Node, TimeStepT MyNumber, TimeStepT HighPt2, TimeStepT HighPt_tmp) {
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
      DFSNodeT & dfs_node = DFSQueue[i];
      dfs_node.deleteBrackets();
      TimeStepT HighPt_tmp = 0, HighPt2 = 0;
      punchMarkerHandleAdjacents(dfs_node, i + 1, HighPt2, HighPt_tmp);
      punchMarkerFinishVisit(dfs_node, i + 1, HighPt2, HighPt_tmp);
    }
  }

public:

  // valid marker's TimeStep won't be zero
  static const NodeT NullNode = {nullptr, NK_Entry};
  static const MarkerT NullMarker = {NullNode, 0};

  static bool isNullMarker(const MarkerT & MA) {
    return !MA.first;
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
