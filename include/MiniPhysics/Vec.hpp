#ifndef Vec_h_
#define Vec_h_

#include <numeric>
#include <array>
#include <cmath>
#include <type_traits>
#include <utility>
#include <tuple>

template <int Dim, typename T>
struct Vec
{
    static_assert(Dim > 0, "No Vectors with dimensions <= 0");
    
private:
    template <typename Tuple, std::size_t ...Is>
    Vec(Tuple &&tuple, std::index_sequence<Is ...>) 
        : _data{static_cast<T>(std::get<Is>(std::forward<Tuple>(tuple))) ...} 
    {}
    
    template <typename ...Ts, typename std::enable_if<sizeof...(Ts) == Dim, int>::type = 0>
    Vec(std::tuple<Ts ...> &&tuple) 
        : Vec(std::move(tuple), std::make_index_sequence<sizeof...(Ts)>()) {}

    template <typename ...Ts, typename std::enable_if<(sizeof...(Ts) < Dim), int>::type = 0>
    Vec(std::tuple<Ts ...> &&tuple) { static_assert(sizeof...(Ts) == Dim, "Not enough items in initialiser"); }

    template <typename ...Ts, typename std::enable_if<(sizeof...(Ts) > Dim), int>::type = 0>
    Vec(std::tuple<Ts ...> &&tuple) { static_assert(sizeof...(Ts) == Dim, "Too many items in initialiser"); }
    

    template <typename Val, typename std::enable_if<std::is_convertible<Val, T>::value, int>::type = 0>
    static auto as_tuple(Val &&val)
    {
        return std::make_tuple(std::forward<Val>(val));
    }
    
    template <typename V, std::size_t ...Is>
    static auto as_tuple(V &&ex, std::index_sequence<Is ...>)
    {
        return std::make_tuple(ex[Is]...);
    }

    template <int D, typename U, typename std::enable_if<std::is_convertible<U , T>::value, int>::type = 0>
    static auto as_tuple(const Vec<D, U> &vec)
    {
        return as_tuple(std::forward<const Vec<D, U> &>(vec), std::make_index_sequence<D>());
    }

    template <int D, typename U, typename std::enable_if<!std::is_convertible<U , T>::value, int>::type = 0>
    static auto as_tuple(const Vec<D, U> &vec)
    {
        static_assert(std::is_convertible<U, T>::value, "Incompatible types");
        return as_tuple(Vec<D, T>{}, std::make_index_sequence<D>());
    }
public:
    template <typename ...Args>
    Vec(Args ...args)
        : Vec(std::move(std::tuple_cat(as_tuple(std::forward<Args>(args))...))) {}
    
    Vec() : _data{} {}

    inline T lengthSquared(void) const
    {
        return std::inner_product(_data.cbegin(), _data.cend(), _data.cbegin(), T{0});
    }

    inline T length(void) const
    {
        return std::sqrt(lengthSquared());
    }
    
    inline Vec normalised(void) const
    {
        const T l = length();
        return l > T{0} ? *this / l : *this;
    }
    
    inline T distanceSquared(Vec rhs)
    {
        return (*this - rhs).lengthSquared();
    }
    
    inline T distance(Vec rhs)
    {
        return (*this - rhs).length();
    }
    
    static T dot(Vec &lhs, Vec &rhs)
    {
        return std::inner_product(lhs._data.begin(), lhs._data.end(), rhs._data.begin(), T{});
    }

    // conversions to scalar if dim is 1
    template <int D = Dim, typename = typename std::enable_if<D == 1, int>::type>
    operator T() { return _data[0]; }
    template <int D = Dim, typename = typename std::enable_if<D == 1, int>::type>
    operator const T() const { return _data[0]; }

    T& operator[](int i) { return _data[i]; }
    const T& operator[](int i) const { return _data[i]; }

    template <int D = Dim>
    typename std::enable_if<(D > 0), T&>::type
    x() { return _data[0]; }

    template <int D = Dim>
    typename std::enable_if<(D > 0), const T&>::type
    x() const { return _data[0]; }

    template <int D = Dim>
    typename std::enable_if<(D > 1), T&>::type
    y() { return _data[1]; }

    template <int D = Dim>
    typename std::enable_if<(D > 1), const T&>::type
    y() const { return _data[1]; }

    template <int D = Dim>
    typename std::enable_if<(D > 2), T&>::type
    z() { return _data[2]; }

    template <int D = Dim>
    typename std::enable_if<(D > 2), const T&>::type
    z() const { return _data[2]; }
    
    template <int D = Dim>
    typename std::enable_if<(D > 3), T&>::type
    w() { return _data[3]; }

    template <int D = Dim>
    typename std::enable_if<(D > 3), const T&>::type
    w() const { return _data[3]; }
    
    Vec operator-(void) { return unary_op([](auto &&a) { return -a; }); }

    template <typename Tl, typename Tr>
    friend Vec operator+(Tl &&lhs, Tr &&rhs)
    {
        return binary_op(std::forward<Tl>(lhs), std::forward<Tr>(rhs), [](auto &&a, auto &&b) { return a + b; });
    }
    
    template <typename Tl, typename Tr>
    friend Vec operator-(Tl &&lhs, Tr &&rhs)
    {
        return binary_op(std::forward<Tl>(lhs), std::forward<Tr>(rhs), [](auto &&a, auto &&b) { return a - b; });
    }
    
    template <typename Tl, typename Tr>
    friend Vec operator*(Tl &&lhs, Tr &&rhs)
    {
        return binary_op(std::forward<Tl>(lhs), std::forward<Tr>(rhs), [](auto &&a, auto &&b) { return a * b; });
    }
    
    template <typename Tl, typename Tr>
    friend Vec operator/(Tl &&lhs, Tr &&rhs)
    {
        return binary_op(std::forward<Tl>(lhs), std::forward<Tr>(rhs), [](auto &&a, auto &&b) { return a / b; });
    }
    
    template <typename Tl, typename Tr>
    friend Vec operator%(Tl &&lhs, Tr &&rhs)
    {
        return binary_op(std::forward<Tl>(lhs), std::forward<Tr>(rhs), [](auto &&a, auto &&b) { return a % b; });
    }
    
    template <typename Tr>
    friend void operator+=(Vec &lhs, const Tr &rhs)
    {
        lhs = lhs + rhs;
    }
    
    template <typename Tr>
    friend void operator-=(Vec &lhs, const Tr &rhs)
    {
        lhs = lhs - rhs;
    }
    
    template <typename Tr>
    friend void operator*=(Vec &lhs, const Tr &rhs)
    {
        lhs = lhs * rhs;
    }
    
    template <typename Tr>
    friend void operator/=(Vec &lhs, const Tr &rhs)
    {
        lhs = lhs / rhs;
    }

    template <typename Tr>
    friend void operator%=(Vec &lhs, const Tr &rhs)
    {
        lhs = lhs % rhs;
    }
    
    // access for all other instantiations of Vec
    template <int D, typename U> friend struct Vec;

private:
    std::array<T, Dim> _data;

    template <typename Fun>
    Vec unary_op(Fun fun)
    {
        Vec result;
        for (int i = 0; i < Dim; ++i)
            result[i] = fun((*this)[i]);
        return result;
    }
    
    template <typename Fun>
    friend Vec binary_op(const Vec &lhs, const Vec &rhs, Fun fun)
    {
        Vec result;
        for (int i = 0; i < Dim; ++i)
            result[i] = fun(lhs[i], rhs[i]);
        return result;
    }

    template <typename Fun>
    friend Vec binary_op(const Vec &lhs, const T rhs, Fun fun)
    {
        Vec result;
        for (int i = 0; i < Dim; ++i)
            result[i] = fun(lhs[i], rhs);
        return result;
    }

    template <typename Fun>
    friend Vec binary_op(const T lhs, const Vec &rhs, Fun fun)
    {
        Vec result;
        for (int i = 0; i < Dim; ++i)
            result[i] = fun(lhs, rhs[i]);
        return result;
    }
};


#endif
