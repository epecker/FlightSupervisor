#ifndef ENUM_STRING_CONVERSION_HPP
#define ENUM_STRING_CONVERSION_HPP

#include <string>
#include <boost/preprocessor.hpp>

#define X_DEFINE_ENUM_WITH_STRING_CONVERSIONS_TOSTRING_CASE(r, data, elem)    \
    case data::elem : return BOOST_PP_STRINGIZE(elem);

#define X_DEFINE_STRING_TO_ENUM_CONVERSIONS(r, data, elem)                    \
    if (s == BOOST_PP_STRINGIZE(elem)) return data::elem;

#define DEFINE_ENUM_WITH_STRING_CONVERSIONS(name, enumerators)                \
    enum class name {                                                         \
        BOOST_PP_SEQ_ENUM(enumerators)                                        \
    };                                                                        \
                                                                              \
    const static char* enumToString(name v)                            		  \
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
    static name stringToEnum(const std::string& s)                         	  \
    {                                                                         \
            BOOST_PP_SEQ_FOR_EACH(                                            \
                X_DEFINE_STRING_TO_ENUM_CONVERSIONS,                          \
                name,                                                         \
                enumerators                                                   \
            )                                                                 \
            return name::BOOST_PP_SEQ_ELEM(0, enumerators);                   \
    }
#endif // ENUM_STRING_CONVERSION_HPP