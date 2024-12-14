#ifndef HASH_TUPLE_H
#define HASH_TUPLE_H

#include <tuple>
#include <boost/functional/hash.hpp>

namespace std {
    // template<typename... T>
    // struct hash<std::tuple<T...>>
    // {
    //     int operator()(std::tuple<T...> const& arg) const noexcept
    //     {
    //         return boost::hash_value(arg);
    //     }
    // };
    template<>
    struct hash<std::tuple<int, int, int>> {
        size_t operator()(std::tuple<int, int, int> const& arg) const noexcept {
            size_t first = std::get<0>(arg);  
            size_t second = std::get<1>(arg); 
            size_t third = std::get<2>(arg);  

            return first + (second << 11) + (third << 21);
        }
    };
}

#endif // HASH_TUPLE_H
