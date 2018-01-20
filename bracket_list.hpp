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
