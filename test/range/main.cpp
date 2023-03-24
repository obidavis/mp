#include "../../include/mp/utility/range.hpp"
#include <iostream>
#include <vector>

int main()
{

    int arr[4] = {2, 3, 4, 5};
    for (int i : mp::contiguous_range<int>(arr, 4))
        std:: cout << i << " ";
    std::cout << "\n";
    
    for (int i : mp::contiguous_range<int>(arr))
        std:: cout << i << " ";
    std::cout << "\n";
    std::vector<int> vec = {5, 6, 7, 8};
    for (int i : mp::contiguous_range<int>(vec.data(), vec.size()))
        std:: cout << i << " ";
    std::cout << "\n";
    for (int i : mp::contiguous_range<int>(vec))
        std:: cout << i << " ";
    std::cout << "\n";

    using R = std::reference_wrapper<int>;
    using I = int;
    
    using X = std::remove_reference_t<mp::meta::unwrap_reference_t<R>>;
    static_assert(std::is_same<X, I>::value, "");
    return 0;
}
