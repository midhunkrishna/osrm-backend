#include "extractor/guidance/constants.hpp"
#include "extractor/guidance/suppress_mode_handler.hpp"
#include "extractor/travel_mode.hpp"

#include <algorithm>
#include <boost/assert.hpp>

namespace osrm
{
namespace extractor
{
namespace guidance
{

SuppressModeHandler::SuppressModeHandler(const IntersectionGenerator &intersection_generator,
                                         const util::NodeBasedDynamicGraph &node_based_graph,
                                         const std::vector<QueryNode> &node_info_list,
                                         const util::NameTable &name_table,
                                         const SuffixTable &street_name_suffix_table)
    : IntersectionHandler(node_based_graph,
                          node_info_list,
                          name_table,
                          street_name_suffix_table,
                          intersection_generator)
{
}

bool SuppressModeHandler::canProcess(const NodeID /*nid*/,
                                     const EdgeID via_eid,
                                     const Intersection &intersection) const
{
    // if the approach way is not on the suppression blacklist, there's no ways to suppress by this criteria
    const auto in_mode = node_based_graph.GetEdgeData(via_eid).travel_mode;
    const auto suppress_in_mode =
        std::find(begin(SUPPRESS_MODE_LIST), end(SUPPRESS_MODE_LIST), in_mode);
    return suppress_in_mode != end(SUPPRESS_MODE_LIST);
}

Intersection SuppressModeHandler::
operator()(const NodeID /*nid*/, const EdgeID source_edge_id, Intersection intersection) const
{
    const auto in_mode = node_based_graph.GetEdgeData(intersection[0].eid).travel_mode;
    BOOST_ASSERT(std::find(begin(SUPPRESS_MODE_LIST), end(SUPPRESS_MODE_LIST), in_mode) != SUPPRESS_MODE_LIST.end());

    // if all out ways share the same mode as the approach way, suppress all the turns
    const auto all_share_mode =
        std::all_of(begin(intersection) + 1, end(intersection), [this, &in_mode](ConnectedRoad way) {
            return node_based_graph.GetEdgeData(way.eid).travel_mode == in_mode;
        });

    if (all_share_mode)
    {
        std::for_each(begin(intersection) + 1, end(intersection), [&](ConnectedRoad &out_way) {
            out_way.instruction = {TurnType::Suppressed, getTurnDirection(out_way.angle)};
        });
    }

    return intersection;
}
}
}
}
