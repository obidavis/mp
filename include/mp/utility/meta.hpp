#pragma once

#include <type_traits>

namespace mp {
namespace meta {

template <bool...> struct bool_pack;

template <bool... bools>
using all_true = std::is_same<bool_pack<true, bools...>, bool_pack<bools..., true>>;

template <typename To, typename... From>
using are_convertible = all_true<std::is_convertible<From, To>::value...>;

template< bool B, class T = void >
using enable_if_t = typename std::enable_if<B,T>::type;

} // namespace meta
} // namespace mp
