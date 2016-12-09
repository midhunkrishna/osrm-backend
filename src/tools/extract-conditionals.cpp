#include "util/log.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <osmium/io/any_input.hpp>
#include <osmium/tags/regex_filter.hpp>

#include <shapefil.h>

#include <fstream>
#include <iostream>
#include <unordered_map>

#include <cstdlib>
#include <ctime>

// better place in third_party/libosmium/include/osmium/tags/regex_filter.hpp
namespace osmium
{

namespace tags
{

template <> struct match_key<std::regex>
{
    bool operator()(const std::regex &rule_key, const char *tag_key)
    {
        return std::regex_match(tag_key, rule_key);
    }
}; // struct match_key<std::regex>
}
}

using point_t = boost::geometry::model::
    point<double, 2, boost::geometry::cs::spherical_equatorial<boost::geometry::degree>>;
using polygon_t = boost::geometry::model::polygon<point_t>;
using box_t = boost::geometry::model::box<point_t>;
using rtree_t =
    boost::geometry::index::rtree<std::pair<box_t, size_t>, boost::geometry::index::rstar<8>>;
using local_time_t = std::pair<polygon_t, struct tm>;

struct LocalTimeClock
{
    LocalTimeClock(const time_t utc) : utc(utc) {}

    struct tm operator()(const char *tzname)
    { // Thread safety: MT-Unsafe const:env
        auto it = memo.find(tzname);
        if (it == memo.end())
        {
            struct tm timeinfo;
            setenv("TZ", tzname, 1);
            tzset();
            localtime_r(&utc, &timeinfo);
            it = memo.insert({tzname, timeinfo}).first;
        }

        return it->second;
    }

  private:
    const time_t utc;
    std::unordered_map<std::string, struct tm> memo;
};

#include <osmium/handler/node_locations_for_ways.hpp>
#include <osmium/index/map/sparse_mem_array.hpp>
#include <osmium/relations/collector.hpp>
using index_type =
    osmium::index::map::SparseMemArray<osmium::unsigned_object_id_type, osmium::Location>;
using location_handler_type = osmium::handler::NodeLocationsForWays<index_type>;

class ConditionalRestrictionsCollector
    : public osmium::relations::Collector<ConditionalRestrictionsCollector, true, true, false>
{
  public:
    using action_t = std::function<void(const osmium::Way &, const osmium::Node &, const osmium::Way &, const char *)>;

    ConditionalRestrictionsCollector(filter_t filter, action_t action)
        : tag_filter(false), filter(filter), action(action)
    {
        tag_filter.add(true, std::regex("^restriction.*conditional$"));
    }

    // filter relations by tag
    bool keep_relation(const osmium::Relation &relation) const
    {
        const osmium::TagList &tags = relation.tags();
        typename decltype(tag_filter)::iterator first(tag_filter, tags.begin(), tags.end());
        typename decltype(tag_filter)::iterator last(tag_filter, tags.end(), tags.end());
        return first != last;
    }

    void complete_relation(osmium::relations::RelationMeta &relation_meta)
    {
        const osmium::Relation &relation = this->get_relation(relation_meta);

        const osmium::Node *via = nullptr;
        const osmium::Way *from = nullptr, *to = nullptr;
        const osmium::memory::Buffer &buffer = this->members_buffer();
        for (const auto &member : relation.members())
        {
            if (member.ref() != 0)
            {
                switch (member.type())
                {
                case osmium::item_type::node:
                    if (strcmp(member.role(), "via") == 0)
                        via = &buffer.get<const osmium::Node>(
                            this->get_offset(member.type(), member.ref()));
                    break;
                case osmium::item_type::way:
                    if (strcmp(member.role(), "from") == 0)
                        from = &buffer.get<const osmium::Way>(
                            this->get_offset(member.type(), member.ref()));
                    else if (strcmp(member.role(), "to") == 0)
                        to = &buffer.get<const osmium::Way>(
                            this->get_offset(member.type(), member.ref()));
                    break;
                default:
                    break;
                }
            }
        }

        if (!from || !via || !to)
            return;

        const osmium::TagList &tags = relation.tags();
        typename decltype(tag_filter)::iterator first(tag_filter, tags.begin(), tags.end());
        typename decltype(tag_filter)::iterator last(tag_filter, tags.end(), tags.end());
        for (; first != last; ++first)
        {
            action(*from, *via, *to, first->value());
        }
    }

  private:
    osmium::tags::Filter<std::regex> tag_filter;
    filter_t filter;
    action_t action;
};

int main(int argc, char *argv[])
{
    const time_t utc_time = 1481242873;
    std::string osm_file = "map.osm";
    //"germany-latest.osm.pbf"; /*config.input_path.string()*/
    std::string timezone_file = "tz_world";

    osrm::util::LogPolicy::GetInstance().Unmute();

    // Load time zones shapes and local times of utc_time
    auto shphandle = SHPOpen(timezone_file.c_str(), "rb");
    auto dbfhandle = DBFOpen(timezone_file.c_str(), "rb");

    int num_entities, shape_type;
    SHPGetInfo(shphandle, &num_entities, &shape_type, NULL, NULL);
    if (num_entities != DBFGetRecordCount(dbfhandle))
    {
        std::cout << "ERROR: inconsistent " << timezone_file << ".shp and " << timezone_file
                  << ".dbf files" << std::endl;
        return EXIT_FAILURE;
    }

    const auto tzid = DBFGetFieldIndex(dbfhandle, "TZID");
    if (tzid == -1)
    {
        std::cout << "ERROR: did not find field called 'TZID' in the tz_world.dbf file"
                  << std::endl;
        return EXIT_FAILURE;
    }

    LocalTimeClock local_clock(utc_time);
    std::vector<rtree_t::value_type> polygons;
    std::vector<local_time_t> local_times;
    for (int shape = 0; shape < num_entities; ++shape)
    {
        auto object = SHPReadObject(shphandle, shape);
        if (object && object->nSHPType == SHPT_POLYGON)
        {
            polygon_t polygon;
            for (int vertex = 0; vertex < object->nVertices; ++vertex)
            {
                polygon.outer().emplace_back(object->padfX[vertex], object->padfY[vertex]);
            }

            polygons.emplace_back(boost::geometry::return_envelope<box_t>(polygon),
                                  local_times.size());

            // get time zone name and resolve UTC to local time
            const auto tzname = DBFReadStringAttribute(dbfhandle, shape, tzid);
            local_times.emplace_back(local_time_t{polygon, local_clock(tzname)});

            // std::cout << boost::geometry::dsv(boost::geometry::return_envelope<box_t>(polygon))
            //           << " " << tzname << " " << asctime(&local_times.back().second);
        }

        SHPDestroyObject(object);
    }

    DBFClose(dbfhandle);
    SHPClose(shphandle);

    // Create R-tree for collected shape polygons
    rtree_t rtree(polygons);
    auto get_local_time = [&rtree, &local_times](const point_t &point) {
        std::vector<rtree_t::value_type> result;
        rtree.query(boost::geometry::index::intersects(point), std::back_inserter(result));
        for (const auto v : result)
        {
            const auto index = v.second;
            if (boost::geometry::within(point, local_times[index].first))
                return local_times[index].second;
        }
        return tm{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    };

    const osmium::io::File input_file(osm_file);

    ConditionalRestrictionsCollector collector(
        [&rtree, &get_local_time](const osmium::Way &from, const osmium::Node &via, const osmium::Way &to, const char *condition) {
            const auto &local_time =
                get_local_time(point_t{via.location().lon(), via.location().lat()});
            std::cout << condition << asctime(&local_time);

            const auto& opening_hours = parseOpeningHours(condition);
            if (!checkOpeningHours(opening_hours, local_time))
                return;


            //std::cout << from.nodes().size() << " " << from.nodes().back().location() << "\n";
            //std::cout << via.location() << "\n";
            //std::cout << to.nodes().size() << " " << to.nodes().front().location() << "\n";

            std::cout << "print me!";
        });

    // Read relations
    osmium::io::Reader reader1(
        input_file, osmium::io::read_meta::no, osmium::osm_entity_bits::relation);
    collector.read_relations(reader1);
    reader1.close();

    std::cerr << "Memory:\n";
    collector.used_memory();

    index_type index;

    // The handler that stores all node locations in the index and adds them
    // to the ways.
    location_handler_type location_handler{index};

    // If a location is not available in the index, we ignore it. It might
    // not be needed (if it is not part of a multipolygon relation), so why
    // create an error?
    location_handler.ignore_errors();

    // On the second pass we read all objects and run them first through the
    // node location handler and then the multipolygon collector. The collector
    // will put the areas it has created into the "buffer" which are then
    // fed through our "handler".
    std::cerr << "Pass 2...\n";
    osmium::io::Reader reader2{input_file};
    // osmium::apply(reader2, location_handler,
    // collector.handler([&handler](osmium::memory::Buffer&& buffer) {
    //     osmium::apply(buffer, handler);
    // }));
    osmium::apply(reader2, location_handler, collector.handler([](osmium::memory::Buffer &&buffer) {
        std::cout << std::distance(buffer.begin(), buffer.end()) << "\n";
        // osmium::apply(buffer, handler);
    }));
    reader2.close();
    std::cerr << "Pass 2 done\n";

    std::cerr << "Memory:\n";
    collector.used_memory();

    // osmium::tags::Filter<std::regex> filter(false);
    // // filter.add(true, std::regex("^restriction.*conditional$"));
    // filter.add(true, std::regex(".*conditional$"));

    // auto type = [](auto t) -> std::string {
    //                 switch(t) {
    //                 case osmium::item_type::node: return "<n>";
    //                 case osmium::item_type::way: return "<w>";
    //                 case osmium::item_type::relation: return "<r>";
    //                 default: break;
    //                 }
    //                 return std::string();
    //             };

    // while (const osmium::memory::Buffer buffer = reader.read())
    // {
    //     for (auto iter = std::begin(buffer), end = std::end(buffer); iter != end; ++iter)
    //     {
    //         // osmium::item_type::node osmium::Node
    //         // osmium::item_type::way osmium::Way
    //         //
    //         //if (iter->type() == osmium::item_type::relation)
    //         {
    //             const auto& relation = static_cast<const osmium::Relation &>(*iter);

    //             const osmium::TagList &tag_list = relation.tags();
    //             decltype(filter)::iterator fi_begin(filter, tag_list.begin(), tag_list.end());
    //             const decltype(filter)::iterator fi_end(filter, tag_list.end(), tag_list.end());

    //             if (std::distance(fi_begin, fi_end) == 0)
    //                 continue;

    //             std::cout << type(relation.type()) << relation.id();
    //             if (relation.members().size() > 0)
    //             {
    //                 std::cout << " [";
    //                 for (const auto &member : relation.members())
    //                 {
    //                     std::cout << member.role() << " " << type(member.type()) << member.ref()
    //                     << " ";
    //                 }
    //                 std::cout << "] ";
    //             }

    //             for (; fi_begin != fi_end; ++fi_begin)
    //             {
    //                 const auto& key = fi_begin->key();
    //                 const auto& value = fi_begin->value();

    //                 std::cout << " " << key << "=" << value << " ";
    //             }
    //             std::cout << "\n";
    //         }
    //     }
    // }
}
