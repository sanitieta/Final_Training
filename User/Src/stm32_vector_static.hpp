//
// Created by xuhao on 2025/11/12.
//
// stm32_vector_static.hpp
// A small, heap-free std::vector-like template for STM32 embedded use.
// Uses static buffer with fixed maximum capacity (defined at compile time).
// No dynamic allocation or exceptions.

#ifndef STM32_VECTOR_STATIC_HPP
#define STM32_VECTOR_STATIC_HPP

#include <cstddef>
#include <new>
#include <utility>
#include <cassert>

template<typename T, std::size_t MaxSize>
class Vector {
public:
    using value_type = T;
    using size_type = std::size_t;
    using reference = T&;
    using const_reference = const T&;
    using pointer = T*;
    using const_pointer = const T*;

    Vector() noexcept : size_(0) {}
    ~Vector() { clear(); }

    Vector(const Vector& other) : size_(0) {
        for (size_type i = 0; i < other.size_; ++i) {
            emplace_back(other[i]);
        }
    }

    Vector& operator=(const Vector& other) {
        if (this != &other) {
            clear();
            for (size_type i = 0; i < other.size_; ++i) {
                emplace_back(other[i]);
            }
        }
        return *this;
    }

    Vector(Vector&& other) noexcept : size_(0) {
        for (size_type i = 0; i < other.size_; ++i) {
            emplace_back(std::move(other[i]));
        }
        other.clear();
    }

    Vector& operator=(Vector&& other) noexcept {
        if (this != &other) {
            clear();
            for (size_type i = 0; i < other.size_; ++i) {
                emplace_back(std::move(other[i]));
            }
            other.clear();
        }
        return *this;
    }

    reference operator[](size_type idx) noexcept {
        assert(idx < size_);
        return *ptr(idx);
    }
    const_reference operator[](size_type idx) const noexcept {
        assert(idx < size_);
        return *ptr(idx);
    }

    pointer data() noexcept { return reinterpret_cast<pointer>(storage_); }
    const_pointer data() const noexcept { return reinterpret_cast<const_pointer>(storage_); }

    size_type size() const noexcept { return size_; }
    static constexpr size_type capacity() noexcept { return MaxSize; }
    bool empty() const noexcept { return size_ == 0; }
    bool full() const noexcept { return size_ >= MaxSize; }

    void clear() noexcept {
        for (size_type i = 0; i < size_; ++i) {
            ptr(i)->~T();
        }
        size_ = 0;
    }

    void push_back(const T& value) {
        assert(!full());
        new (ptr(size_)) T(value);
        ++size_;
    }

    void push_back(T&& value) {
        assert(!full());
        new (ptr(size_)) T(std::move(value));
        ++size_;
    }

    template<typename... Args>
    void emplace_back(Args&&... args) {
        assert(!full());
        new (ptr(size_)) T(std::forward<Args>(args)...);
        ++size_;
    }

    void pop_back() noexcept {
        assert(size_ > 0);
        --size_;
        ptr(size_)->~T();
    }

    void insert(size_type index, const T& value) {
        assert(index <= size_ && !full());
        for (size_type i = size_; i > index; --i) {
            new (ptr(i)) T(std::move(*ptr(i - 1)));
            ptr(i - 1)->~T();
        }
        new (ptr(index)) T(value);
        ++size_;
    }

    void erase(size_type index) {
        assert(index < size_);
        ptr(index)->~T();
        for (size_type i = index; i + 1 < size_; ++i) {
            new (ptr(i)) T(std::move(*ptr(i + 1)));
            ptr(i + 1)->~T();
        }
        --size_;
    }

    pointer begin() noexcept { return data(); }
    pointer end() noexcept { return data() + size_; }
    const_pointer begin() const noexcept { return data(); }
    const_pointer end() const noexcept { return data() + size_; }

private:
    alignas(T) unsigned char storage_[sizeof(T) * MaxSize];
    size_type size_;

    pointer ptr(size_type index) noexcept {
        return reinterpret_cast<pointer>(storage_ + sizeof(T) * index);
    }
    const_pointer ptr(size_type index) const noexcept {
        return reinterpret_cast<const_pointer>(storage_ + sizeof(T) * index);
    }
};

#endif // STM32_VECTOR_STATIC_HPP

/* Example usage:

#include "stm32_vector_static.hpp"

Vector<int, 32> nums;  // vector with max 32 ints, no heap

void example() {
    nums.push_back(42);
    nums.emplace_back(13);
    for (auto n : nums) {
        // do something
    }
}
*/
