#ifndef IDLIST_HPP
#define IDLIST_HPP

namespace llvm {
namespace NLE_internal {

/* intrusive double linked list that supports delete from a iterator */
class idlist_impl {
  idlist_impl *next, *prev;
  // delete this node from the list
  // next and prev are guaranteed to be non-null
public:
  idlist_impl() : next(nullptr), prev(nullptr) {}
  // construct a idlist and insert after p
  idlist_impl(idlist_impl & p) {
    insertAfter(p);
  }

  void insertAfter(idlist_impl & p) {
    next = p.next;
    next->prev = this;
    prev = &p;
    p.next = this;
  }

  /* this won't update list's size, size should be maintained seperately
   */
  void dropFromList() {
    next->prev = prev;
    prev->next = next;
  }
  idlist_impl* getNext() { return next; }
  idlist_impl* getPrev() { return prev; }
  static void Initiate(idlist_impl & head, idlist_impl & tail) {
    head.next = &tail;
    head.prev = nullptr;
    tail.prev = &head;
    tail.next = nullptr;
  }
  static void merge(idlist_impl &t1, idlist_impl & h2, idlist_impl & t2) {
    if (h2.next != &t2) {
      t1.prev->next = h2.next;
      h2.next->prev = t1.prev;
      t1.prev = t2.prev;
      t2.prev->next = &t1;
      h2.next = &t2;
      t2.prev = &h2;
    }
  }
};

/* This class is used to provide a convinent Next()/Prev() method.
 * Cast the begin/end mark of a double linked list won't abort your program(I
 * don't care whether this is UB), only when you dereference the pointer will
 * your program abort()
 */
template <class Derived>
class idlist : public idlist_impl {
public:
  Derived* getNext() { return static_cast<Derived*>(idlist_impl::getNext()); }
  Derived* getPrev() { return static_cast<Derived*>(idlist_impl::getPrev()); }
};

}  // namespace NLE_internal
}  // namespace llvm

#endif  // IDLIST_HPP
