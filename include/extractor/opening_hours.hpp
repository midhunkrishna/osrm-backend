#ifndef OPENING_HOURS_HPP
#define OPENING_HOURS_HPP

//#define BOOST_SPIRIT_DEBUG

#include <boost/spirit/include/phoenix.hpp>
#include <boost/spirit/include/qi.hpp>

#include <boost/algorithm/string/replace.hpp>

#include <algorithm>
#include <cctype>
#include <iterator>
#include <limits>
#include <string>

#include "extractor/guidance/toolkit.hpp"

namespace osrm
{
namespace extractor
{

namespace detail
{

namespace qi = boost::spirit::qi;

template <typename Iterator>
struct openin_hours_grammar : qi::grammar<Iterator, void(), qi::blank_type>
{
    // http://wiki.openstreetmap.org/wiki/Key:opening_hours/specification
    openin_hours_grammar() : openin_hours_grammar::base_type(time_domain)
    {
        using qi::_1;
        using qi::_pass;
        using qi::_val;
        using qi::eoi;
        using qi::lit;
        using qi::char_;
        using qi::uint_;

        // General syntax
        time_domain = rule_sequence % any_rule_separator;

        rule_sequence = (selector_sequence >> -rule_modifier) | comment;

        any_rule_separator = char_(";,") | lit("||");

        // Rule modifiers
        rule_modifier = (lit("unknown") | lit("open") | lit("closed") | lit("off"));

        // Selectors
        selector_sequence = lit("24/7") | (wide_range_selectors || small_range_selectors);

        wide_range_selectors = -year_selector >> (monthday_selector || week_selector);

        small_range_selectors = weekday_selector || time_selector;

        // Time selector
        time_selector = timespan % ',';

        timespan =
            time >> -('+' | ('-' >> extended_time >> -('+' | '/' >> (minute | hour_minutes))));

        time = hour_minutes | variable_time;

        extended_time = extended_hour_minutes | variable_time;

        variable_time = event | ('(' >> event >> plus_or_minus >> hour_minutes >> ')');

        event = lit("dawn") | lit("sunrise") | lit("sunset") | lit("dusk");

        // Weekday selector
        weekday_selector = (holiday_sequence >> -(char_(", ") >> weekday_sequence)) |
                           (weekday_sequence >> -(char_(", ") >> holiday_sequence));

        weekday_sequence = weekday_range % ',';

        weekday_range = wday >> -(('-' >> wday) | (nth_entry >> ',' >> nth_entry >> -day_offset));

        holiday_sequence = (lit("SH") >> -day_offset) | lit("PH");

        nth_entry = nth | nth >> '-' >> nth | '-' >> nth;

        nth = char_("12345");

        day_offset = plus_or_minus >> uint_ >> lit("days");

        // Week selector
        week_selector = (lit("week ") >> week) % ',';

        week = weeknum >> -('-' >> weeknum >> -('/' >> uint_));

        // Month selector
        monthday_selector = monthday_range % ',';

        monthday_range = (date_from >> -date_offset >> '-' >> date_to >> -date_offset) |
                         (date_from >> -(date_offset >> -lit('+')));

        date_offset = (plus_or_minus >> wday) | day_offset;

        date_from = (month || (daynum >> (&~lit(':') | eoi))) | variable_date;

        date_to = date_from | daynum;

        variable_date = lit("easter");

        // Year selector
        year_selector = year_range % ',';

        year_range = year >> -(('-' >> year >> -('/' >> uint_)) | '+');

        // Basic elements
        plus_or_minus = lit('+') | lit('-');

        hour = uint2_p[_pass = bind([](unsigned x) { return x <= 24; }, _1), _val = _1];

        extended_hour = uint2_p[_pass = bind([](unsigned x) { return x <= 48; }, _1), _val = _1];

        minute = uint2_p[_pass = bind([](unsigned x) { return x < 60; }, _1), _val = _1];

        hour_minutes = hour >> ':' >> minute;

        extended_hour_minutes = extended_hour >> ':' >> minute;

        wday.add("Mo", 0)("Tu", 1)("We", 2)("Th", 3)("Fr", 4)("Sa", 5)("Su", 6);

        daynum =
            uint2_p[_pass = bind([](unsigned x) { return 01 <= x && x <= 31; }, _1), _val = _1];

        weeknum =
            uint2_p[_pass = bind([](unsigned x) { return 01 <= x && x <= 53; }, _1), _val = _1];

        month.add("Jan", 0)("Feb", 1)("Mar", 2)("Apr", 3)("May", 4)("Jun", 5)("Jul", 6)("Aug", 7)(
            "Sep", 8)("Oct", 9)("Nov", 10)("Dec", 11);

        year = uint4_p[_pass = bind([](unsigned x) { return x > 1900; }, _1), _val = _1];

        comment = lit('"') >> *(~qi::char_('"')) >> lit('"');

        BOOST_SPIRIT_DEBUG_NODES((time_domain)(rule_sequence)(any_rule_separator)(rule_modifier)(
            selector_sequence)(wide_range_selectors)(small_range_selectors)(time_selector)(
            timespan)(time)(extended_time)(variable_time)(event)(weekday_selector)(
            weekday_sequence)(weekday_range)(holiday_sequence)(nth_entry)(nth)(day_offset)(
            week_selector)(week)(monthday_selector)(monthday_range)(date_offset)(date_from)(
            date_to)(variable_date)(year_selector)(year_range)(plus_or_minus)(hour_minutes)(
            extended_hour_minutes)(comment)(hour)(extended_hour)(minute)(daynum)(weeknum)(year));
    }

    qi::rule<Iterator, void(), qi::blank_type> time_domain, rule_sequence, any_rule_separator,
        rule_modifier, selector_sequence, wide_range_selectors, small_range_selectors,
        time_selector, timespan, time, extended_time, variable_time, event, weekday_selector,
        weekday_sequence, weekday_range, holiday_sequence, nth_entry, nth, day_offset,
        week_selector, week, monthday_selector, monthday_range, date_offset, date_from, date_to,
        variable_date, year_selector, year_range, plus_or_minus, hour_minutes,
        extended_hour_minutes, comment;
    qi::rule<Iterator, unsigned(), qi::blank_type> hour, extended_hour, minute, daynum, weeknum,
        year;
    qi::symbols<char const, unsigned char> wday, month;

    qi::uint_parser<unsigned, 10, 2, 2> uint2_p;
    qi::uint_parser<unsigned, 10, 4, 4> uint4_p;
};
}

inline bool parseOpeningHours(const std::string &s)
{
    detail::openin_hours_grammar<std::string::const_iterator> openin_hours_grammar;

    std::string::const_iterator iter = s.begin();
    bool ok = boost::spirit::qi::phrase_parse(
        iter, s.end(), openin_hours_grammar, boost::spirit::qi::blank);

    if (!ok || iter != s.end())
        std::cout << "failed at " << std::string(iter, s.end()) << "\n";
    else
        std::cout << "passed " << s << "\n";

    return ok && iter == s.end();
}

} // extractor
} // osrm

#endif // OPENING_HOURS_HPP
