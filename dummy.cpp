#include "dummy.h"

// template <typename T> MyClass<T>::MyClass(T m) { member = m; }

template <typename T> T MyClass<T>::getMember() { return member; }
