#include "idlist.hpp"

#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/DenseMapInfo.h"
#include "llvm/ADT/iterator_range.h"
#include "llvm/ADT/PointerIntPair.h"
#include "llvm/Analysis/RegionInfo.h"
#include "llvm/IR/CFG.h"
#include "llvm/IR/Function.h"
#include "llvm/Pass.h"
#include "llvm/PassAnalysisSupport.h"
#include "llvm/Support/Allocator.h"

#include <forward_list>
#include <iterator>
#include <list>
#include <memory>
#include <type_traits>
#include <utility>

#include <cstddef>

namespace llvm {


}
/* RT = RegionTraits */
template <class RT, class AllocatorT = MallocAllocator>
class NodeLoopEquivalence {
public:
  using FuncT = typename RT::FuncT;
  using BlockT = typename RT::BlockT;
  using SuccItorT = typename GraphTraits<BlockT*>::ChildIteratorType;
  using PredItorT = typename GraphTraits<Inverse<BlockT*>>::ChildIteratorType;


  /* We use the size of the bracket list, and top bracket's descendant node as the marker
   * this is different from the original paper(which use the size and both two node of the bracket as the marker)
   */
  using MarkerT = std::pair<NodeT, size_t>;

private:
  using BlockMakerMapT = DenseMap<const BlockT*, MarkerT>;
  BlockMakerMapT BMM;
  using BracketListT = internal::BracketListT<NodeLoopEquivalence>;
  const FuncT & Func;
  AllocatorT Allocator;
  // DFSQueue should be indexed starting zero, while DFSNumbers are assigned
  // from 1, therefore DFSQueue[0]'s DFS_Number is 1, etc. DFS_Number being 0
  // means:
  // 1) We pushed a node to stack, but didn't pop it (algorithm's implementation is wrong)
  // 2) After DFSVisit() finished, unreachable node(in the undirected graph) are not in DFSNumbers
  using TimeStepT = size_t;
  DenseMap<NodeT, TimeStepT> DFSNumbers;
  TimeStepT DFS_Number; // there may be some not connected node even in the undirected graph, so DFS_Number may be less than (2 * Function.size() - 1)

  // For a edge (father, child) in the DFS spanning tree
  // Edge.getPointer() is child node, Edge.getInt() is whether this is a split-edge
  // HighPt is the highest father reachable through one bracket from a desdantent below and including child
  // Let child's N children's HighPt be HighPt_0...HighPt_N, HighPt2 is the second largest of the set {HighPt_0, ..., HighPt_N} (if such second highest exists, otherwise zero)
  // HighPt_tmp is a working area and implementation detail
  // Brackets is the list of brackets starting from child and child's descendants
  // BracketsToDelete is a list of brackets targeting child
  // DFSNodeT is stored in the DFSQueue array in the order of child's DFSNumber
  // Notice that valid DFSNumber starts from 1(i.e. DFSNodeT's child's DFSNumber - 1 == DFSNodeT's index in DFSQueue)
  class DFSNodeT {
  private:
    EdgeT Edge;
    TimeStepT HighPt;
    BracketListT Brackets; 
    std::forward_list<typename BracketListT::DeleteHandleT> BracketsToDelete; // the brackets(or frond) targeting child node
    // Merge a child's bracket list to the end of our bracket list, update HighPt, HighPt2
  public:
    void mergeChild(DFSNodeT & Child, TimeStepT & HighPt2) {
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
    void addBracket(DFSNodeT & Ancestor, bool IsSplitEdge) {
      Ancestor.BracketsToDelete.push_front(Brackets.PushEdge(Edge.GetChildNode(), IsSplitEdge));
    }
    void addBracket(DFSNodeT & Ancestor) {
      bool IsSplitEdge = Ancestor.Edge.getNode().getPal() == Edge.getNode().
      Ancestor.BracketsToDelete.push_front(Brackets.PushEdge(Edge.GetChildNode(), IsSplitEdge));
    }
    DFSNodeT(EdgeT Edge_, AllocatorT & Allocator) : Edge(Edge_), HighPt(0), Brackets(Allocator) {}
    void deleteBrackets() {
      Brackets.drop(BracketsToDelete);
    }
    EdgeT getEdge() { return Edge; }
    size_t getBracketsSize() { return Brackets.size(); }
  };

  class DFSStackNodeT {
  public:
    DFSStackNodeT(EdgeT Edge_, TimeStepT & DFSNumber_, bool PalVisited = false) : Edge(Edge_), DFSNumber(&DFSNumber_, PalVisited) {
      if (Edge.GetChildNode().IsEntry()) {
        Pred = pred_begin(getBB());
      } else {
        Succ = succ_begin(getBB());
      }
    }
    EdgeT getEdge() { return Edge; }
    NodeT getNode() { return getEdge().getChildNode(); }
    BlockT* getBB() { return getNode().getPointer(); }
    bool isPalVisited() { return DFSNumber.getInt(); }
    void setPalVisited() { DFSNumber.setInt(true); }
    NodeT getPal() { return getNode().pal(); }
    EdgeT getPalEdge() { return getNode().getPalEdge(); }
    size_t getDFSNumber() { return *DFSNumber.getPointer(); }
    void setDFSNumber(size_t val) { *DFSNumber.getPointer() = val; }
    SuccItorT getSucc() { return Succ; }
    PredItorT getPred() { return Pred; }
  void setItor(SuccItorT Succ_) { Succ = Succ_; }
  void setItor(PredItorT Pred_) { Pred = Pred_; }

private:
  EdgeT Edge; // child node of this edge is the node we are visiting
  PointerIntPair<size_t *, 1, bool> DFSNumber;
  union {
    SuccItorT Succ;
    PredItorT Pred;
    };
  };


  std::vector<DFSNodeT> DFSQueue;

  std::vector<DFSStackNodeT> DFSStack;
  size_t DFSStackTop;

  template <NodeKind NK>
  using ItorT = std::conditional_t<NK == NK_Entry, PredItorT, SuccItorT>;
  template <NodeKind NK>
  std::enable_if_t<NK == NK_Entry, ItorT<NK>> current(DFSStackNodeT &stack_node) {
    return stack_node.getPred();
  }
  template <NodeKind NK>
  std::enable_if_t<NK == NK_Entry, ItorT<NK>> end(DFSStackNodeT &stack_node) {
    return pred_end(stack_node.getPointer());
  }
  template <NodeKind NK>
  std::enable_if_t<NK == NK_Exit, ItorT<NK>> current(DFSStackNodeT &stack_node) {
    return stack_node.getSucc();
  }
  template <NodeKind NK>
  std::enable_if_t<NK == NK_Exit, ItorT<NK>> end(DFSStackNodeT &stack_node) {
    return succ_end(stack_node.getPointer());
  }
  template <NodeKind NK>
  EdgeT getEdge(ItorT<NK> itor) { return {&*itor, false}; }
  template <NodeKind NK>
  void dfsVisitStackTop() {
    auto tryVisit = [&] (EdgeT Edge, bool PalVisited) -> bool {
      auto insert = DFSNumbers.insert({Edge.getChildNode(), 0});
      if (insert.second) {
        DFSStack[DFSStackTop] = {Edge, *insert.first};
        DFSStack[DFSStackTop].setPalVisited(PalVisited);
        ++DFSStackTop;
        return true;
      }
      return false;
    };
    DFSStackNodeT & tos = DFSStack[DFSStackTop - 1];
    if (!tos.getPalVisited()) {
      tos.setPalVisited();
      if (tryVisit(tos.getPalEdge(), true)) {
        return;
      }
    }
    for (ItorT<NK> child = current<NK>(tos), end = end<NK>(tos); child != end; ++child) {
      if (tryVisit(getEdge(child), true)) {
        tos.setItor(child);
        return;
      }
    }
    DFSQueue[DFS_Number++] = {tos.getEdge(), Allocator};
    --DFSStackTop;
  }

  // visit all reachable (in the sense that reachable in the undirected graph), generate DFSNumbers and DFSQueue
  void dfsVisit() {
    const size_t splite_node_count = 2 * Func.size() - 1;
    DFSNumbers.clear();
    DFSQueue.clear();
    DFSNumbers.reserve(splite_node_count);
    DFSQueue.reserve(splite_node_count);
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

  void punchMarkerHandleAdjacents(DFSNodeT &Node, TimeStepT MyNumber, TimeStepT &HighPt2,
                         TimeStepT &HighPt_tmp) {
    auto VisitAdjacent = [&](TimeStepT AdjNumber, bool IsSplitEdge) {
      if (AdjNumber < MyNumber) {
        mergeChild(DFSQueue[AdjNumber - 1], HighPt2, HighPt_tmp);
      } else {
        HighPt_tmp = std::max(HighPt_tmp, AdjNumber);
        addBracket(DFSQueue[AdjNumber - 1], IsSplitEdge);
      }
    };
    NodeT child_node = Node.getEdge().getChildNode();
    visitAdjacent(DFSNumbers[child_node.getPal()], true);
    if (child_node.isEntry()) {
      for (BlockT* Pred : predecessors(child_node.getBlock())) {
        visitAdjacent(DFSNumbers[{Pred, NK_Exit}], false);
      }
    } else {
      for (BlockT* Succ : successors(child_node.getBlock())) {
        visitAdjacent(DFSNumbers[{Succ, NK_Entry}], false);
      }
    }
  }

  void punchMarkerFinishVisit(DFSNodeT &Node, TimeStepT MyNumber, TimeStepT HighPt2, TimeStepT HighPt_tmp) {
    if (HighPt_tmp > Node.HighPt) {
      Node.HighPt = HighPt_tmp;
    }
    if (HighPt2 > MyNumber) {
      Node.addBracket(DFSQueue[HighPt2 - 1]);
    }
    if (!Node.getEdge().isSplitEdge()) {
      return;
    }
    MarkerT marker = Node.getBrackets().getMarker();
    BMM[Node.getEdge().getChildNode().getPointer()] = marker;
    if (Node.getBracketsSize() == 1) {
      EdgeT edge = Node.getBrackets().front();
      if (edge.isSplitEdge()) {
        BMM[edge.getChildNode().getPointer()] = marker;
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

class NodeLoopEquivalencePrinterPass : public FunctionPass { 
public:
  static char ID;
  NodeLoopEquivalencePrinterPass() : FunctionPass(ID) {}
  bool runOnFunction(Function & Func) override {
    llvm::outs() << "start run on function: " << Func.getName() << '\n';
    NodeLoopEquivalence<RegionTraits<Function>> NLE(Func);
    for (auto itor1 = Func.begin(), end = Func.end(); itor1 != end; ++itor1) {
      for (auto itor2 = std::next(itor1); itor2 != end; ++itor2) {
        BasicBlock * bb1 = &*itor1;
        BasicBlock * bb2 = &*itor2;
        if (NLE.isNodeLoopEquivalent(bb1, bb2)) {
          llvm::outs() << bb1->getName() << ' ' << bb2->getName() << '\n';
        }
      }
    }
    return false;
  }

  void getAnalysisUsage(AnalysisUsage &Info) const override {
    Info.setPreservesAll();
  }
};

}  // namespace llvm
