#include "extractor/opening_hours.hpp"

#include <boost/test/test_case_template.hpp>
#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(opening_hours)

using namespace osrm;

BOOST_AUTO_TEST_CASE(check_opening_hours_grammar)
{
    BOOST_CHECK(extractor::parseOpeningHours("Apr 10-Jun 15"));
    BOOST_CHECK(extractor::parseOpeningHours("Apr 10-15 off"));
    BOOST_CHECK(extractor::parseOpeningHours("Jun 08:00-14:00"));

    BOOST_CHECK(extractor::parseOpeningHours("24/7"));
    BOOST_CHECK(extractor::parseOpeningHours("Sa-Su 00:00-24:00"));
    BOOST_CHECK(extractor::parseOpeningHours("Mo-Fr 08:30-20:00"));
    BOOST_CHECK(extractor::parseOpeningHours(
        "Mo 10:00-12:00,12:30-15:00; Tu-Fr 08:00-12:00,12:30-15:00; Sa 08:00-12:00"));
    BOOST_CHECK(extractor::parseOpeningHours(
        "Mo-Su 08:00-18:00; Apr 10-15 off; Jun 08:00-14:00; Aug off; Dec 25 off"));
    BOOST_CHECK(extractor::parseOpeningHours("Mo-Sa 10:00-20:00; Tu off"));
    BOOST_CHECK(extractor::parseOpeningHours("Mo-Sa 10:00-20:00; Tu 10:00-14:00"));
    BOOST_CHECK(extractor::parseOpeningHours("sunrise-(sunset-01:30)"));

    BOOST_CHECK(extractor::parseOpeningHours("Su 10:00+"));
    BOOST_CHECK(
        extractor::parseOpeningHours("Mo-Sa 08:00-13:00,14:00-17:00 || \"by appointment\""));
    BOOST_CHECK(extractor::parseOpeningHours(
        "Su-Tu 11:00-01:00, We-Th 11:00-03:00, Fr 11:00-06:00, Sa 11:00-07:00"));
    BOOST_CHECK(
        extractor::parseOpeningHours("week 01-53/2 Fr 09:00-12:00; week 02-52/2 We 09:00-12:00"));
    BOOST_CHECK(extractor::parseOpeningHours("Mo-Su,PH 15:00-03:00; easter -2 days off"));

    BOOST_CHECK(extractor::parseOpeningHours("08:30-12:30,15:30-20:00"));
    BOOST_CHECK(extractor::parseOpeningHours("Tu,Th 16:00-20:00"));

    BOOST_CHECK(extractor::parseOpeningHours("2016 Feb-2017 Dec"));

    BOOST_CHECK(extractor::parseOpeningHours("2016-2017"));
}

BOOST_AUTO_TEST_SUITE_END()
