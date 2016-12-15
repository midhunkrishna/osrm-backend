#ifndef CONDITIONAL_RESTRICTIONS_HPP
#define CONDITIONAL_RESTRICTIONS_HPP

//#define BOOST_SPIRIT_DEBUG

#include <boost/fusion/include/adapt_struct.hpp>
#include <boost/spirit/include/phoenix.hpp>
#include <boost/spirit/include/qi.hpp>

namespace osrm
{
namespace util
{

// Helper functions for OSM conditional restrictions
// http://wiki.openstreetmap.org/wiki/Conditional_restrictions
// Consitional restrictions is a vector of ConditionalRestriction
// with a restriction value and a condition string
struct ConditionalRestriction
{
    std::string value;
    std::string condition;
};

inline std::ostream &operator<<(std::ostream &stream, const ConditionalRestriction& restriction)
{
    return stream << restriction.value << "=" << restriction.condition;
}
}
}

BOOST_FUSION_ADAPT_STRUCT(osrm::util::ConditionalRestriction, value, condition)

namespace osrm
{
namespace util
{
namespace detail
{

namespace
{
namespace ph = boost::phoenix;
namespace qi = boost::spirit::qi;
}

template <typename Iterator, typename Skipper = qi::blank_type>
struct conditional_restrictions_grammar : qi::grammar<Iterator, Skipper, std::vector<ConditionalRestriction>()>
{
    // http://wiki.openstreetmap.org/wiki/Conditional_restrictions
    conditional_restrictions_grammar() : conditional_restrictions_grammar::base_type(restrictions)
    {
        using qi::_1;
        using qi::_val;
        using qi::lit;

        // clang-format off

        restrictions
            = restriction % ';'
            ;

        restriction
            = value >> '@' >> condition
            ;

        value
            = +(qi::char_ - '@')
            ;

        condition
            = *qi::blank
            >> (lit('(') >> qi::as_string[qi::no_skip[*~lit(')')]][_val = _1] >> lit(')')
                | qi::as_string[qi::no_skip[*~lit(';')]][_val = _1]
               )
            ;

        // clang-format on

        BOOST_SPIRIT_DEBUG_NODES((restrictions)(restriction)(value)(condition));
    }

    qi::rule<Iterator, Skipper, std::vector<ConditionalRestriction>()> restrictions;
    qi::rule<Iterator, Skipper, ConditionalRestriction()> restriction;
    qi::rule<Iterator, Skipper, std::string()> value, condition;
};
}

inline std::vector<ConditionalRestriction> ParseConditionalRestrictions(const std::string &s)
{
    const detail::conditional_restrictions_grammar<std::string::const_iterator> static grammar;

    std::vector<ConditionalRestriction> result;
    std::string::const_iterator iter = s.begin();
    bool ok =
        boost::spirit::qi::phrase_parse(iter, s.end(), grammar, boost::spirit::qi::blank, result);

    // TODO: remove debug
    if (!ok || iter != s.end())
        std::cout << "failed at " << std::string(iter, s.end()) << "\n";
    else
        std::cout << "passed " << s << "\n";

    for (auto x : result)
        std::cout << x << "\n";

    if (!ok || iter != s.end())
        return std::vector<ConditionalRestriction>();

    return result;
}

} // util
} // osrm

#endif // CONDITIONAL_RESTRICTIONS_HPP
