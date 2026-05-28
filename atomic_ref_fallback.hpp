#pragma once
#include <atomic>
#include <type_traits>

namespace compat {

template <typename T>
class atomic_ref {
    static_assert(std::is_trivially_copyable_v<T>,
                  "atomic_ref<T> requires trivially copyable type");

    T* ptr_;

    // Reinterpret helper: type-punning is the defining mechanism of std::atomic_ref,
    // which this class polyfills for pre-C++20 toolchains.
    std::atomic<T>* a() const noexcept {
        return reinterpret_cast<std::atomic<T>*>(ptr_);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    }

public:
    using value_type = T;

    explicit atomic_ref(T& obj) noexcept : ptr_(std::addressof(obj)) {}

    // Store/load
    void store(T desired,
               std::memory_order order = std::memory_order_seq_cst) const noexcept {
        a()->store(desired, order);
    }

    T load(std::memory_order order = std::memory_order_seq_cst) const noexcept {
        return a()->load(order);
    }

    // Exchange
    T exchange(T desired,
               std::memory_order order = std::memory_order_seq_cst) const noexcept {
        return a()->exchange(desired, order);
    }

    // Compare-exchange
    bool compare_exchange_strong(T& expected, T desired,
                                 std::memory_order success,
                                 std::memory_order failure) const noexcept {
        return a()->compare_exchange_strong(expected, desired, success, failure);
    }

    bool compare_exchange_strong(T& expected, T desired,
                                 std::memory_order order =
                                     std::memory_order_seq_cst) const noexcept {
        return compare_exchange_strong(expected, desired, order, order);
    }

    // Compare-exchange weak
    bool compare_exchange_weak(T& expected, T desired,
                               std::memory_order success,
                               std::memory_order failure) const noexcept {
        return a()->compare_exchange_weak(expected, desired, success, failure);
    }

    bool compare_exchange_weak(T& expected, T desired,
                               std::memory_order order =
                                   std::memory_order_seq_cst) const noexcept {
        return compare_exchange_weak(expected, desired, order, order);
    }

    // Arithmetic operations (for integral types)
    template <typename U = T>
    std::enable_if_t<std::is_integral_v<U>, U>
    fetch_add(U arg, std::memory_order order = std::memory_order_seq_cst) const noexcept {
        return reinterpret_cast<std::atomic<U>*>(ptr_)->fetch_add(arg, order);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    }

    template <typename U = T>
    std::enable_if_t<std::is_integral_v<U>, U>
    fetch_sub(U arg, std::memory_order order = std::memory_order_seq_cst) const noexcept {
        return reinterpret_cast<std::atomic<U>*>(ptr_)->fetch_sub(arg, order);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    }

    // Increment/decrement
    T operator++() const noexcept { return fetch_add(1) + 1; }
    T operator++(int) const noexcept { return fetch_add(1); }
    T operator--() const noexcept { return fetch_sub(1) - 1; }
    T operator--(int) const noexcept { return fetch_sub(1); }

    operator T() const noexcept { return load(); }
};

} // namespace compat
