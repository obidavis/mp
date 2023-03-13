#include <iostream>
#include "../../src/Vec.hpp"
#include "../../src/Vec_ios.hpp"
#include <tuple>

struct A
{
    A() {}
    A(const A &other) { std::cout << "copy\n"; }
    A(A &&) { std::cout << "move\n"; }
};

int main()
{
    using Vec_t = Vec<3, int>;
    Vec_t a{};
    Vec_t b = {1, 2, 3.5};
    b = {2, 3, 4};
    Vec_t c{3, 4, 5};
    Vec_t d = {4, 3, 4};
    Vec<1, int> e = 7;
    Vec<1, int> f = {3};
    auto g = f;
    Vec<3, bool> h = b;
    Vec<4, Vec<3, int>> i = {a, b, c, d};
    Vec<3, int> j = {b};
    Vec<2, int> k = {6, 7};
    const Vec<2, int> l = {8, 9};
    Vec<4, int> m = {k, l};
    Vec<6, int> n = {k, Vec<2, int>{10, 11}, l};
    int o = e;
    Vec<4, double> p = {b, 1.2};
    Vec<2, std::string> q = {"hello", "world"};
    Vec<2, int> r = q;
    A y, z;
    Vec<1, const A> x{std::move(y)};
    std::cout << "a: " << a << "\n";
    std::cout << "b: " << b << "\n";
    std::cout << "c: " << c << "\n";
    std::cout << "d: " << d << "\n";
    std::cout << "e: " << e << "\n";
    std::cout << "f: " << f << "\n";
    std::cout << "g: " << g << "\n";
    std::cout << "h: " << h << "\n";
    std::cout << "i: " << i << "\n";
    std::cout << "j: " << j << "\n";
    std::cout << "k: " << k << "\n";
    std::cout << "l: " << l << "\n";
    std::cout << "m: " << m << "\n";
    std::cout << "n: " << n << "\n";
    std::cout << "o: " << o << "\n";
    std::cout << "p: " << p << "\n";
    std::cout << "q: " << q << "\n";
    std::cout << "Test Success" << "\n";

    return 0;
}
