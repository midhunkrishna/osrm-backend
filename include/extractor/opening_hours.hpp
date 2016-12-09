#ifndef OPENING_HOURS_HPP
#define OPENING_HOURS_HPP

//#define BOOST_SPIRIT_DEBUG

#include <boost/spirit/include/phoenix.hpp>
#include <boost/spirit/include/qi.hpp>

#include <boost/io/ios_state.hpp>

#include <algorithm>
#include <cctype>
#include <iomanip>
#include <iterator>
#include <limits>
#include <string>

std::ostream &operator<<(std::ostream &stream, const struct tm &value)
{
    char s[128];
    strftime(s, sizeof(s), "%a, %d %b %Y %T %z", &value);
    return stream << s;
}

std::ostream &operator<<(std::ostream &stream, const std::pair<struct tm, struct tm> &value)
{
    return stream << value.first << "-" << value.second;
}

namespace osrm
{
namespace extractor
{

struct OpeningHours
{
    using DateTime = struct tm;

    enum Modifier
    {
        unknown,
        open,
        closed,
        off,
        is24_7
    };

    struct Time
    {
        enum Event : unsigned char
        {
            invalid,
            none,
            dawn,
            sunrise,
            sunset,
            dusk
        };

        Event event;
        char hour;
        char min;
        char positive;

        Time() : event(invalid), hour(0), min(0), positive(false) {}
        Time(Event event) : event(event), hour(0), min(0), positive(true) {}
        Time(char hour, char min) : event(none), hour(hour), min(min), positive(true) {}
        Time(Event event, bool positive, const Time &offset)
            : event(event), hour(offset.hour), min(offset.min), positive(positive)
        {
        }
    };

    struct TimeSpan
    {
        Time from, to;
        TimeSpan() {}
        TimeSpan(const Time &from, const Time &to) : from(from), to(to) {}
    };

    struct WeekdayRange
    {
        char from, to;
        WeekdayRange() : from(0), to(0) {}
        WeekdayRange(char from, char to) : from(from), to(to) {}
    };

    struct Monthday
    {
        int year;
        char month;
        char day;
        Monthday() : year(0), month(0), day(0) {}
        Monthday(int year) : year(year), month(0), day(0) {}
        Monthday(int year, char month, char day) : year(year), month(month), day(day) {}
    };

    struct MonthdayRange
    {
        Monthday from, to;
        MonthdayRange() : from(0, 0, 0), to(0, 0, 0) {}
        MonthdayRange(const Monthday &from, const Monthday &to) : from(from), to(to) {}
    };

    OpeningHours() : modifier(open) {}

    bool IsOpen(const struct tm &tm) const { return false; }

    std::vector<TimeSpan> times;
    std::vector<WeekdayRange> weekdays;
    std::vector<MonthdayRange> monthdays;
    Modifier modifier;
};

std::ostream &operator<<(std::ostream &stream, const OpeningHours::Modifier value)
{
    switch (value)
    {
    case OpeningHours::unknown:
        return stream << "unknown";
    case OpeningHours::open:
        return stream << "open";
    case OpeningHours::closed:
        return stream << "closed";
    case OpeningHours::off:
        return stream << "off";
    case OpeningHours::is24_7:
        return stream << "24/7";
    }
    return stream;
}

std::ostream &operator<<(std::ostream &stream, const OpeningHours::Time::Event value)
{
    switch (value)
    {
    case OpeningHours::Time::dawn:
        return stream << "dawn";
    case OpeningHours::Time::sunrise:
        return stream << "sunrise";
    case OpeningHours::Time::sunset:
        return stream << "sunset";
    case OpeningHours::Time::dusk:
        return stream << "dusk";
    default:
        break;
    }
    return stream;
}

std::ostream &operator<<(std::ostream &stream, const OpeningHours::Time &value)
{
    boost::io::ios_flags_saver ifs(stream);
    if (value.event == OpeningHours::Time::invalid)
        return stream << "???";
    if (value.event == OpeningHours::Time::none)
        return stream << std::setfill('0') << std::setw(2) << (int)value.hour << ":"
                      << std::setfill('0') << std::setw(2) << (int)value.min;
    stream << value.event;
    if (value.hour != 0)
        stream << (value.positive ? '+' : '-') << std::setfill('0') << std::setw(2)
               << (int)value.hour << ":" << std::setfill('0') << std::setw(2) << (int)value.min;
    else if (value.min != 0)
        stream << (value.positive ? '+' : '-') << std::setfill('0') << std::setw(2)
               << (int)value.min;
    return stream;
}

std::ostream &operator<<(std::ostream &stream, const OpeningHours::TimeSpan &value)
{
    return stream << value.from << "-" << value.to;
}

std::ostream &operator<<(std::ostream &stream, const OpeningHours::Monthday &value)
{
    bool empty = true;
    if (value.year != 0)
    {
        stream << (int)value.year;
        empty = false;
    };
    if (value.month != 0)
    {
        stream << (empty ? "" : "/") << (int)value.month;
        empty = false;
    };
    if (value.day != 0)
    {
        stream << (empty ? "" : "/") << (int)value.day;
    };
    return stream;
}

std::ostream &operator<<(std::ostream &stream, const OpeningHours::WeekdayRange &value)
{
    if (value.to == 0)
        return stream << (int)value.from;
    return stream << (int)value.from << "-" << (int)value.to;
}

std::ostream &operator<<(std::ostream &stream, const OpeningHours::MonthdayRange &value)
{
    return stream << value.from << "-" << value.to;
}

std::ostream &operator<<(std::ostream &stream, const OpeningHours &value)
{
    if (value.modifier == OpeningHours::is24_7)
        return stream << OpeningHours::is24_7;

    for (auto x : value.monthdays)
        stream << x << ", ";
    for (auto x : value.weekdays)
        stream << x << ", ";
    for (auto x : value.times)
        stream << x << ", ";
    return stream << " |" << value.modifier << "|";
}

namespace detail
{

namespace
{
namespace ph = boost::phoenix;
namespace qi = boost::spirit::qi;
}

template <typename Iterator, typename Skipper = qi::blank_type>
struct opening_hours_grammar : qi::grammar<Iterator, Skipper, std::vector<OpeningHours>()>
{
    // http://wiki.openstreetmap.org/wiki/Key:opening_hours/specification
    opening_hours_grammar() : opening_hours_grammar::base_type(time_domain)
    {
        using qi::_1;
        using qi::_a;
        using qi::_b;
        using qi::_c;
        using qi::_r1;
        using qi::_pass;
        using qi::_val;
        using qi::eoi;
        using qi::lit;
        using qi::char_;
        using qi::uint_;
        using oh = osrm::extractor::OpeningHours;

        // General syntax
        time_domain = rule_sequence[ph::push_back(_val, _1)] % any_rule_separator;

        rule_sequence =
            lit("24/7")[ph::bind(&oh::modifier, _val) = oh::is24_7] |
            (selector_sequence[_val = _1] >> -rule_modifier[ph::bind(&oh::modifier, _val) = _1]) |
            comment;

        any_rule_separator = char_(";,") | lit("||");

        // Rule modifiers
        rule_modifier.add("unknown", oh::unknown)("open", oh::open)("closed", oh::closed)("off",
                                                                                          oh::off);

        // Selectors
        selector_sequence = (wide_range_selectors(_a) || small_range_selectors(_a))[_val = _a];

        wide_range_selectors = year_selector(_r1) || monthday_selector(_r1) ||
                               week_selector(_r1); // TODO week_selector -> ignored

        small_range_selectors = weekday_selector(_r1) || time_selector(_r1);

        // Time selector
        time_selector = (timespan % ',')[ph::bind(&OpeningHours::times, _r1) = _1];

        timespan =
            (time[_a = _1] >>
             -('+' |
               ('-' >> extended_time[_b = _1] >>
                -('+' |
                  '/' >> (minute |
                          hour_minutes)))))[_val = ph::construct<OpeningHours::TimeSpan>(_a, _b)];

        time = hour_minutes | variable_time;

        extended_time = extended_hour_minutes | variable_time;

        variable_time = event[_val = ph::construct<OpeningHours::Time>(_1)] |
                        ('(' >> event[_a = _1] >> plus_or_minus[_b = _1] >> hour_minutes[_c = _1] >>
                         ')')[_val = ph::construct<OpeningHours::Time>(_a, _b, _c)];

        event.add("dawn", OpeningHours::Time::dawn)("sunrise", OpeningHours::Time::sunrise)(
            "sunset", OpeningHours::Time::sunset)("dusk", OpeningHours::Time::dusk);

        // Weekday selector
        weekday_selector = (holiday_sequence(_r1) >> -(char_(", ") >> weekday_sequence(_r1))) |
                           (weekday_sequence(_r1) >> -(char_(", ") >> holiday_sequence(_r1)));

        weekday_sequence = (weekday_range % ',')[ph::bind(&OpeningHours::weekdays, _r1) = _1];

        weekday_range = wday[ph::bind(&OpeningHours::WeekdayRange::from, _val) = _1] >>
                        -(('-' >> wday[ph::bind(&OpeningHours::WeekdayRange::to, _val) = _1]) |
                          ('[' >> (nth_entry % ',') >> ']' >> -day_offset));

        holiday_sequence = (lit("SH") >> -day_offset) | lit("PH");

        nth_entry = nth | nth >> '-' >> nth | '-' >> nth;

        nth = char_("12345");

        day_offset = plus_or_minus >> uint_ >> lit("days");

        // Week selector
        week_selector = (lit("week ") >> week) % ',';

        week = weeknum >> -('-' >> weeknum >> -('/' >> uint_));

        // Month selector
        monthday_selector = (monthday_range % ',')[ph::bind(&OpeningHours::monthdays, _r1) = _1];

        monthday_range =
            (date_from[ph::bind(&OpeningHours::MonthdayRange::from, _val) = _1] >> -date_offset >>
             '-' >> date_to[ph::bind(&OpeningHours::MonthdayRange::to, _val) = _1] >>
             -date_offset) |
            (date_from[ph::bind(&OpeningHours::MonthdayRange::from, _val) = _1] >>
             -(date_offset >> -lit('+')));

        date_offset = (plus_or_minus >> wday) | day_offset;

        date_from =
            ((-year[_a = _1] >> (month[_b = _1] || (daynum[_c = _1] >> (&~lit(':') | eoi)))) |
             variable_date)[_val = ph::construct<OpeningHours::Monthday>(_a, _b, _c)];

        date_to =
            date_from[_val = _1] | daynum[_val = ph::construct<OpeningHours::Monthday>(0, 0, _1)];

        variable_date = lit("easter");

        // Year selector
        year_selector = (year_range % ',')[ph::bind(&OpeningHours::monthdays, _r1) = _1];

        year_range = year[ph::bind(&OpeningHours::MonthdayRange::from, _val) =
                              ph::construct<OpeningHours::Monthday>(_1)] >>
                     -(('-' >> year[ph::bind(&OpeningHours::MonthdayRange::to, _val) =
                                        ph::construct<OpeningHours::Monthday>(_1)] >>
                        -('/' >> uint_)) |
                       '+');

        // Basic elements
        plus_or_minus = lit('+')[_val = true] | lit('-')[_val = false];

        hour = uint2_p[_pass = bind([](unsigned x) { return x <= 24; }, _1), _val = _1];

        extended_hour = uint2_p[_pass = bind([](unsigned x) { return x <= 48; }, _1), _val = _1];

        minute = uint2_p[_pass = bind([](unsigned x) { return x < 60; }, _1), _val = _1];

        hour_minutes =
            hour[_a = _1] >> ':' >> minute[_val = ph::construct<OpeningHours::Time>(_a, _1)];

        extended_hour_minutes = extended_hour[_a = _1] >> ':' >>
                                minute[_val = ph::construct<OpeningHours::Time>(_a, _1)];

        wday.add("Mo", 1)("Tu", 2)("We", 3)("Th", 4)("Fr", 5)("Sa", 6)("Su", 7);

        daynum =
            uint2_p[_pass = bind([](unsigned x) { return 01 <= x && x <= 31; }, _1), _val = _1];

        weeknum =
            uint2_p[_pass = bind([](unsigned x) { return 01 <= x && x <= 53; }, _1), _val = _1];

        month.add("Jan", 1)("Feb", 2)("Mar", 3)("Apr", 4)("May", 5)("Jun", 6)("Jul", 7)("Aug", 8)(
            "Sep", 9)("Oct", 10)("Nov", 11)("Dec", 12);

        year = uint4_p[_pass = bind([](unsigned x) { return x > 1900; }, _1), _val = _1];

        comment = lit('"') >> *(~qi::char_('"')) >> lit('"');

        BOOST_SPIRIT_DEBUG_NODES((time_domain)(rule_sequence)(any_rule_separator)(
            selector_sequence)(wide_range_selectors)(small_range_selectors)(time_selector)(
            timespan)(time)(extended_time)(variable_time)(weekday_selector)(weekday_sequence)(
            weekday_range)(holiday_sequence)(nth_entry)(nth)(day_offset)(week_selector)(week)(
            monthday_selector)(monthday_range)(date_offset)(date_from)(date_to)(variable_date)(
            year_selector)(year_range)(plus_or_minus)(hour_minutes)(extended_hour_minutes)(comment)(
            hour)(extended_hour)(minute)(daynum)(weeknum)(year));
    }

    qi::rule<Iterator, Skipper, std::vector<OpeningHours>()> time_domain;
    qi::rule<Iterator, Skipper, OpeningHours()> rule_sequence;
    qi::rule<Iterator, Skipper, void()> any_rule_separator;
    qi::rule<Iterator, Skipper, OpeningHours(), qi::locals<OpeningHours>> selector_sequence;
    qi::symbols<char const, OpeningHours::Modifier> rule_modifier;
    qi::rule<Iterator, Skipper, void(OpeningHours &)> wide_range_selectors, small_range_selectors,
        time_selector, weekday_selector, year_selector, monthday_selector, week_selector;

    // Time rules
    qi::rule<Iterator,
             Skipper,
             OpeningHours::TimeSpan(),
             qi::locals<OpeningHours::Time, OpeningHours::Time>>
        timespan;

    qi::rule<Iterator, Skipper, OpeningHours::Time()> time, extended_time;

    qi::rule<Iterator,
             Skipper,
             OpeningHours::Time(),
             qi::locals<OpeningHours::Time::Event, bool, OpeningHours::Time>>
        variable_time;

    qi::rule<Iterator, Skipper, OpeningHours::Time(), qi::locals<unsigned>> hour_minutes,
        extended_hour_minutes;

    qi::symbols<char const, OpeningHours::Time::Event> event;

    qi::rule<Iterator, Skipper, bool()> plus_or_minus;

    // Weekday rules
    qi::rule<Iterator, Skipper, void(OpeningHours &)> weekday_sequence, holiday_sequence;

    qi::rule<Iterator, Skipper, OpeningHours::WeekdayRange(), qi::locals<unsigned>> weekday_range;

    // Monthday rules
    qi::rule<Iterator, Skipper, OpeningHours::MonthdayRange()> monthday_range;

    qi::rule<Iterator, Skipper, OpeningHours::Monthday(), qi::locals<unsigned, unsigned, unsigned>>
        date_from;

    qi::rule<Iterator, Skipper, OpeningHours::Monthday()> date_to;

    // Year rules
    qi::rule<Iterator, Skipper, OpeningHours::MonthdayRange()> year_range;

    // Unused rules
    qi::rule<Iterator, Skipper, void()> nth_entry, nth, day_offset, week, date_offset,
        variable_date, comment;

    // Basic rules and parsers
    qi::rule<Iterator, Skipper, unsigned()> hour, extended_hour, minute, daynum, weeknum, year;
    qi::symbols<char const, unsigned char> wday, month;
    qi::uint_parser<unsigned, 10, 2, 2> uint2_p;
    qi::uint_parser<unsigned, 10, 4, 4> uint4_p;
};
}

inline std::vector<OpeningHours> parseOpeningHours(const std::string &s)
{
    const detail::opening_hours_grammar<std::string::const_iterator> static grammar;

    std::vector<OpeningHours> result;
    std::string::const_iterator iter = s.begin();
    bool ok =
        boost::spirit::qi::phrase_parse(iter, s.end(), grammar, boost::spirit::qi::blank, result);

    if (!ok || iter != s.end())
        std::cout << "failed at " << std::string(iter, s.end()) << "\n";
    else
        std::cout << "passed " << s << "\n";

    for (auto x : result)
        std::cout << "    " << x << "\n";

    return ok && iter == s.end();
}

inline bool checkOpeningHours(const std::vector<OpeningHours>& input)
{
    return true;
}

} // extractor
} // osrm

#endif // OPENING_HOURS_HPP
