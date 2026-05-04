// Copyright 2025 Tenstorrent Corporation or its affiliates.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <atomic>
#include "atomic_ref_fallback.hpp"

#if defined(__cpp_lib_atomic_ref)
using std::atomic_ref;
#else
using compat::atomic_ref;
#endif

class SpinLock {
public:
    SpinLock() = default;

    void lock() {
        while (true) {
            int expected = 0;
            if (atomic_ref(lock_).compare_exchange_strong(expected, 1,
                        std::memory_order_acquire, std::memory_order_relaxed)) {
                return;
            }
        }
    }

    void unlock() {
      atomic_ref(lock_).store(0, std::memory_order_release);
    }

private:
    int lock_ = 0;
};
