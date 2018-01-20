#ifndef NODELOOPEQUIVALENCE_HPP
#define NODELOOPEQUIVALENCE_HPP

#include "Bracket.hpp"
#include "llvm/ADT/GraphTraits.h"

#include <forward_list>
#include <type_traits>

namespace llvm {
namespace NLE_internal {

// For a edge (father, child) in the DFS spanning tree
// Edge.getPointer() is child node, Edge.getInt() is whether this is a split-edge
// HighPt is the highest father reachable through one bracket from a desdantent below and including child
// Let child's N children's HighPt be HighPt_0...HighPt_N, HighPt2 is the second largest of the set {HighPt_0, ..., HighPt_N} (if such second highest exists, otherwise zero)
// HighPt_tmp is a working area and implementation detail
// Brackets is the list of brackets starting from child and child's descendants
// BracketsToDelete is a list of brackets targeting child
// DFSNodeT is stored in the DFSQueue array in the order of child's DFSNumber
// Notice that valid DFSNumber starts from 1(i.e. DFSNodeT's child's DFSNumber - 1 == DFSNodeT's index in DFSQueue)

template <class GraphType, class AllocatorT = MallocAllocator>
class NodeLoopEquivalence {
public:
  using FuncT = std::remove_cv_t<std::remove_reference_t<<GraphType>>;
  using NodeRef = GraphTraits<const FuncT*>::NodeRef;
  using NodeT = PointerIntPair<NodeRef, 1, NodeKind>;
  using EdgeT = PointerIntPair<NodeT, 1, EdgeKind>;
  using MarkerT = std::pair<NodeRef, size_t>;
  using TimeStepT = size_t;
  using SuccItorT = GraphTraits<NodeRef>::ChildIteratorType;
  using PredItorT = GraphTraits<Inverse<NodeRef>>::ChildIteratorType;
  using BracketListT = BracketList<NodeRef>;
  using DeleteHandleT = BracketListT::DeleteHandleT;

  static NodeRef getBB(NodeT N) {
    return N.getPointer();
  }

  static NodeRef getChildBB(EdgeT E) {
    return getBB(E.getPointer());
  }

  static bool isEntry(NodeT N) {
    return N.getInt() == NK_Entry;
  }

  static bool childIsEntry(Edge E) {
    return isEntry(E.getPointer());
  }

  static bool isSplitEdge(Edge E) {
    return E.getInt() == EK_SplitEdge;
  }

  struct DFSNode {
    // child node and whether (parent, child) is a split-edge
    EdgeT Edge;
    TimeStepT HighPt;
    BracketListT Brackets; 
    // brackets targeting this node
    std::forward_list<DeleteHandleT> BracketsToDelete;
  };

  // Merge a child's bracket list to the end of our bracket list, update HighPt, HighPt2
  void mergeDFSNode(DFSNode &Parent, DFSNode &Child, TimeStepT &HighPt2) {
    auto update_max2 = [](size_t & v1, size_t & v2, size_t v) {
      if (v > v1) {
        std::swap(v, v1);
      }
      if (v > v2) {
        v2 = v;
      }
    };
    Brackets.merge(Child.Brackets);
    update_max2(HighPt, HighPt2, Child.HighPt);
  }
  // Add a bracket targeting ancestor, update HighPt_tmp
  void addBracketToDFSNode(DFSNode & Ancestor, DFSNode & Desdencant, bool IsSplitEdge) {
    Ancestor.BracketsToDelete.push_front(Brackets.PushEdge(Edge.GetChildNode(), IsSplitEdge));
  }

  void addBracketToDFSNode(DFSNode & Ancestor, DFSNode & Desdencant) {
    bool IsSplitEdge = Ancestor.Edge.getNode().getPal() == Edge.getNode().
    Ancestor.BracketsToDelete.push_front(Brackets.PushEdge(Edge.GetChildNode(), IsSplitEdge));
  }

  void dropBracketsDFSNode(DFSNode & Node) {
    Node.Brackets.drop(Node.BracketsToDelete);
  }

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

    template <NodeKind NK>
    static std::enable_if_t<NK == NK_Exit, SuccItorT> current_pos() {
      return Succ;
    }

    template <NodeKind NK>
    static std::enable_if_t<NK == NK_Exit, SuccItorT> end_pos() {
      return succ_end(getChildBB(Edge));
    }
  };

private:
  using BlockMakerMapT = DenseMap<const BlockT*, MarkerT>;
  BlockMakerMapT BMM;
  using BracketListT = BracketList<NodeLoopEquivalence>;
  const FuncT & Func;
  AllocatorT Allocator;
  // DFSQueue should be indexed starting zero, while DFSNumbers are assigned
  // from 1, therefore DFSQueue[0]'s DFS_Number is 1, etc. DFS_Number being 0
  // means:
  // 1) We pushed a node to stack, but didn't pop it (algorithm's implementation is wrong)
  // 2) After DFSVisit() finished, unreachable node(in the undirected graph) are not in DFSNumbers
  DenseMap<NodeT, TimeStepT> DFSNumbers;
  TimeStepT DFS_Number; // there may be some not connected node even in the undirected graph, so DFS_Number may be less than (2 * Function.size() - 1)
  std::vector<DFSNodeT> DFSQueue;

  std::vector<DFSStackNodeT> DFSStack;
  size_t DFSStackTop;

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
    if (!tos.isPalVisited()) {
      tos.setPalVisited();
      if (tryVisit(tos.getPalEdge(), true)) {
        return;
      }
    }
    for (ItorT<NK> child = current_pos<NK>(tos), end = end_pos<NK>(tos); child != end; ++child) {
      if (tryVisit(getEdge<NK>(child), true)) {
        tos.setItor(child);
        return;
      }
    }
    DFSQueue.emplace_back(tos.getEdge(), Allocator);
    // TODO: save some space
    ++DFS_Number;
    --DFSStackTop;
  }

  // visit all reachable (in the sense that reachable in the undirected graph), generate DFSNumbers and DFSQueue
  void dfsVisit() {
    const size_t splite_node_count = 2 * Func.size() - 1;
    DFSNumbers.clear();
    DFSQueue.clear();
    DFSNumbers.reserve(splite_node_count);
    //DFSQueue.reserve(splite_node_count);
    DFS_Number = 0;
    DFSStack.clear();
    DFSStack.resize(splite_node_count);

    const BlockT & entryBB = Func.getEntryBlock();
    NodeT entry{&entryBB, NK_Exit};
    DFSStack[0] = {{entry, EK_NonSplitEdge}, DFSNumbers[entry]};
    DFSStack[0].setPalVisited();
    DFSStackTop = 1;
    while (DFSStackTop) {
      if (DFSStack[DFSStackTop - 1].getEdge().getChildNode().isEntry()) {
        dfsVisitStackTop<NK_Entry>();
      } else {
        dfsVisitStackTop<NK_Exit>();
      }
    }
    DFSStack.clear();
  }

  void punchMarkerHandleAdjacents(DFSNodeT &Node, TimeStepT MyNumber,
                                  TimeStepT &HighPt2, TimeStepT &HighPt_tmp) {
    auto visitAdjacent = [&](TimeStepT AdjNumber, bool IsSplitEdge) {
      if (AdjNumber < MyNumber) {
        Node.mergeChild(DFSQueue[AdjNumber - 1], HighPt2);
      } else {
        HighPt_tmp = std::max(HighPt_tmp, AdjNumber);
        Node.addBracket(DFSQueue[AdjNumber - 1], IsSplitEdge);
      }
    };
    NodeT child_node = Node.getEdge().getChildNode();
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
    if (HighPt_tmp > Node.getHighPt()) {
      Node.setHighPt(HighPt_tmp);
    }
    if (HighPt2 > MyNumber) {
      Node.addBracket(DFSQueue[HighPt2 - 1]);
    }
    if (!Node.getEdge().isSplitEdge()) {
      return;
    }
    MarkerT marker = Node.getBrackets().getMarker();
    BMM[Node.getEdge().getChildNode().getBlock()] = marker;
    if (Node.getBrackets().size() == 1) {
      EdgeT edge = Node.getBrackets().front();
      if (edge.isSplitEdge()) {
        BMM[edge.getChildNode().getBlock()] = marker;
      }
    }
  }

  void punchMarkers() {
    for (TimeStepT i = 0; i + 1 < DFS_Number; ++i) {
      DFSNodeT & dfs_node = DFSQueue[i];
      dfs_node.deleteBrackets();
      TimeStepT HighPt_tmp = 0, HighPt2 = 0;
      punchMarkerHandleAdjacents(dfs_node, i + 1, HighPt2, HighPt_tmp);
      punchMarkerFinishVisit(dfs_node, i + 1, HighPt2, HighPt_tmp);
    }
  }

public:

  static MarkerT NullMarker() {
    return {{nullptr, false}, 0};
  }

  static bool isNullMarker(const MarkerT & MA) {
    return !MA.second;
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

} // namespace NLE_internal
}  // namespace llvm

#endif
