#ifndef HASH_TUPLE_H
#define HASH_TUPLE_H

#include <tuple>
#include <boost/functional/hash.hpp>

namespace std {
    template<typename... T>
    struct hash<std::tuple<T...>>
    {
        size_t operator()(std::tuple<T...> const& arg) const noexcept
        {
            return boost::hash_value(arg);
        }
    };
}

#endif // HASH_TUPLE_H
