#ifndef ENUM_STRING_CONVERSION_HPP
#define ENUM_STRING_CONVERSION_HPP

#include <boost/preprocessor.hpp>

#define X_DEFINE_ENUM_WITH_STRING_CONVERSIONS_TOSTRING_CASE(r, data, elem)    \
    case data::elem : return BOOST_PP_STRINGIZE(elem);

#define X_DEFINE_STRING_TO_ENUM_CONVERSIONS(r, data, elem)                    \
    if (s.compare(BOOST_PP_STRINGIZE(elem)) == 0) return data::elem;

#define DEFINE_ENUM_WITH_STRING_CONVERSIONS(name, enumerators)                \
    enum class name {                                                         \
        BOOST_PP_SEQ_ENUM(enumerators)                                        \
    };                                                                        \
                                                                              \
    inline const static char* enumToString(name v)                            \
    {                                                                         \
        switch (v)                                                            \
        {                                                                     \
            BOOST_PP_SEQ_FOR_EACH(                                            \
                X_DEFINE_ENUM_WITH_STRING_CONVERSIONS_TOSTRING_CASE,          \
                name,                                                         \
                enumerators                                                   \
            )                                                                 \
            default: return "[Unknown " BOOST_PP_STRINGIZE(name) "]";         \
        }                                                                     \
    }                                                                         \
                                                                              \
    inline const static name stringToEnum(string s)                           \
    {                                                                         \
            BOOST_PP_SEQ_FOR_EACH(                                            \
                X_DEFINE_STRING_TO_ENUM_CONVERSIONS,                          \
                name,                                                         \
                enumerators                                                   \
            )                                                                 \
            return name::BOOST_PP_SEQ_ELEM(0, enumerators);                   \
    }
#endif // ENUM_STRING_CONVERSION_HPP