#pragma once

#include "meta.hpp"

namespace mp {

    template <typename T>
    struct contiguous_range
    {
        contiguous_range() : _begin(nullptr), _size(0) {}
        contiguous_range(T *begin_ptr, T *end_ptr)
            : _begin(begin_ptr), _size(end_ptr - begin_ptr) {}
        contiguous_range(T *begin_ptr, std::size_t size)
            : _begin(begin_ptr), _size(size) {}

        template <std::size_t N>
        contiguous_range(T (&arr)[N]) : contiguous_range(arr, N) {}

    private:
        using underlying_type = std::remove_reference_t<meta::unwrap_reference_t<T>>;
        template <typename C>
        using has_data_size = typename meta::all_true<
            std::is_same<decltype(std::declval<C>().data()), T *>::value,
            std::is_same<decltype(std::declval<C>().size()), std::size_t>::value
        >;

    public:
        template <typename C, typename meta::enable_if_t<has_data_size<C>::value, int> = 0>
        contiguous_range(C &c) : contiguous_range(c.data(), c.size()) {}
        T *begin() { return _begin; }
        T *end() { return _begin + _size; }
        std::size_t size() const { return _size; }

    private:
        T *_begin;
        std::size_t _size;
    };

    
}
