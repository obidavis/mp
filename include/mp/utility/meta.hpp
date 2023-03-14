#pragma once

#include <type_traits>

namespace mp {
namespace meta {

template <bool...> struct bool_pack;

template <bool... bools>
using all_true = std::is_same<bool_pack<true, bools...>, bool_pack<bools..., true>>;

template <typename To, typename... From>
using are_convertible = all_true<std::is_convertible<From, To>::value...>;

} // namespace meta
} // namespace mp
