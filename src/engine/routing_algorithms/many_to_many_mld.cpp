#include "engine/routing_algorithms/many_to_many.hpp"
#include "engine/routing_algorithms/routing_base_mld.hpp"

#include <boost/assert.hpp>
#include <boost/range/iterator_range_core.hpp>

#include <limits>
#include <memory>
#include <unordered_map>
#include <vector>

namespace osrm
{
namespace engine
{
namespace routing_algorithms
{

namespace mld
{

template <typename MultiLevelPartition>
inline LevelID getNodeQueryLevel(const MultiLevelPartition &partition,
                                 const NodeID node,
                                 const PhantomNode &phantom_node)
{
    auto highest_diffrent_level = [&partition, node](const SegmentID &phantom_node) {
        if (phantom_node.enabled)
            return partition.GetHighestDifferentLevel(phantom_node.id, node);
        return INVALID_LEVEL_ID;
    };

    const auto node_level = std::min(highest_diffrent_level(phantom_node.forward_segment_id),
                                     highest_diffrent_level(phantom_node.reverse_segment_id));

    return node_level;
}

template <typename MultiLevelPartition>
inline LevelID getNodeQueryLevel(const MultiLevelPartition &partition,
                                 const NodeID node,
                                 const PhantomNode &phantom_node,
                                 const LevelID maximal_level)
{
    const auto node_level = getNodeQueryLevel(partition, node, phantom_node);

    if (node_level >= maximal_level)
        return INVALID_LEVEL_ID;

    return node_level;
}

template <typename MultiLevelPartition>
inline LevelID getNodeQueryLevel(const MultiLevelPartition &partition,
                                 NodeID node,
                                 const std::vector<PhantomNode> &phantom_nodes,
                                 const std::size_t phantom_index,
                                 const std::vector<std::size_t> &phantom_indices)
{
    auto min_level = [&partition, node](const PhantomNode &phantom_node) {

        const auto &forward_segment = phantom_node.forward_segment_id;
        const auto forward_level =
            forward_segment.enabled ? partition.GetHighestDifferentLevel(node, forward_segment.id)
                                    : INVALID_LEVEL_ID;

        const auto &reverse_segment = phantom_node.reverse_segment_id;
        const auto reverse_level =
            reverse_segment.enabled ? partition.GetHighestDifferentLevel(node, reverse_segment.id)
                                    : INVALID_LEVEL_ID;

        return std::min(forward_level, reverse_level);
    };

    // Get minimum level over all phantoms of the highest different level with respect to node
    // This is equivalent to min_{∀ source, target} partition.GetQueryLevel(source, node, target)
    auto result = min_level(phantom_nodes[phantom_index]);
    for (const auto &index : phantom_indices)
    {
        result = std::min(result, min_level(phantom_nodes[index]));
    }
    return result;
}

template <bool DIRECTION, typename... Args>
void relaxOutgoingEdges(const DataFacade<mld::Algorithm> &facade,
                        const NodeID node,
                        const EdgeWeight weight,
                        const EdgeDuration duration,
                        typename SearchEngineData<mld::Algorithm>::ManyToManyQueryHeap &query_heap,
                        Args... args)
{
    BOOST_ASSERT(!facade.ExcludeNode(node));

    const auto &partition = facade.GetMultiLevelPartition();

    const auto level = getNodeQueryLevel(partition, node, args...);

    // Break outgoing edges relaxation if node at the restricted level
    if (level == INVALID_LEVEL_ID)
        return;

    const auto &cells = facade.GetCellStorage();
    const auto &metric = facade.GetCellMetric();
    const auto &node_data = query_heap.GetData(node);

    if (level >= 1 && !node_data.from_clique_arc)
    {
        const auto &cell = cells.GetCell(metric, level, partition.GetCell(level, node));
        if (DIRECTION == FORWARD_DIRECTION)
        { // Shortcuts in forward direction
            auto destination = cell.GetDestinationNodes().begin();
            auto shortcut_durations = cell.GetOutDuration(node);
            for (auto shortcut_weight : cell.GetOutWeight(node))
            {
                BOOST_ASSERT(destination != cell.GetDestinationNodes().end());
                BOOST_ASSERT(!shortcut_durations.empty());
                const NodeID to = *destination;

                if (shortcut_weight != INVALID_EDGE_WEIGHT && node != to)
                {
                    const auto to_weight = weight + shortcut_weight;
                    const auto to_duration = duration + shortcut_durations.front();
                    if (!query_heap.WasInserted(to))
                    {
                        query_heap.Insert(to, to_weight, {node, true, to_duration});
                    }
                    else if (std::tie(to_weight, to_duration) <
                             std::tie(query_heap.GetKey(to), query_heap.GetData(to).duration))
                    {
                        query_heap.GetData(to) = {node, true, to_duration};
                        query_heap.DecreaseKey(to, to_weight);
                    }
                }
                ++destination;
                shortcut_durations.advance_begin(1);
            }
            BOOST_ASSERT(shortcut_durations.empty());
        }
        else
        { // Shortcuts in backward direction
            auto source = cell.GetSourceNodes().begin();
            auto shortcut_durations = cell.GetInDuration(node);
            for (auto shortcut_weight : cell.GetInWeight(node))
            {
                BOOST_ASSERT(source != cell.GetSourceNodes().end());
                BOOST_ASSERT(!shortcut_durations.empty());
                const NodeID to = *source;

                if (shortcut_weight != INVALID_EDGE_WEIGHT && node != to)
                {
                    const auto to_weight = weight + shortcut_weight;
                    const auto to_duration = duration + shortcut_durations.front();
                    if (!query_heap.WasInserted(to))
                    {
                        query_heap.Insert(to, to_weight, {node, true, to_duration});
                    }
                    else if (std::tie(to_weight, to_duration) <
                             std::tie(query_heap.GetKey(to), query_heap.GetData(to).duration))
                    {
                        query_heap.GetData(to) = {node, true, to_duration};
                        query_heap.DecreaseKey(to, to_weight);
                    }
                }
                ++source;
                shortcut_durations.advance_begin(1);
            }
            BOOST_ASSERT(shortcut_durations.empty());
        }
    }

    for (const auto edge : facade.GetBorderEdgeRange(level, node))
    {
        const auto &data = facade.GetEdgeData(edge);
        if (DIRECTION == FORWARD_DIRECTION ? data.forward : data.backward)
        {
            const NodeID to = facade.GetTarget(edge);
            if (facade.ExcludeNode(to))
            {
                continue;
            }

            const auto edge_weight = data.weight;
            const auto edge_duration = data.duration;

            BOOST_ASSERT_MSG(edge_weight > 0, "edge_weight invalid");
            const auto to_weight = weight + edge_weight;
            const auto to_duration = duration + edge_duration;

            // New Node discovered -> Add to Heap + Node Info Storage
            if (!query_heap.WasInserted(to))
            {
                query_heap.Insert(to, to_weight, {node, false, to_duration});
            }
            // Found a shorter Path -> Update weight and set new parent
            else if (std::tie(to_weight, to_duration) <
                     std::tie(query_heap.GetKey(to), query_heap.GetData(to).duration))
            {
                query_heap.GetData(to) = {node, false, to_duration};
                query_heap.DecreaseKey(to, to_weight);
            }
        }
    }
}

//
// Unidirectional multi-layer Dijkstra search for 1-to-N and N-to-1 matrices
//
template <bool DIRECTION>
std::pair<std::vector<EdgeDuration>, std::vector<EdgeDistance>>
oneToManySearch(SearchEngineData<Algorithm> &engine_working_data,
                const DataFacade<Algorithm> &facade,
                const std::vector<PhantomNode> &phantom_nodes,
                std::size_t phantom_index,
                const std::vector<std::size_t> &phantom_indices,
                const bool calculate_distance)
{
    (void)calculate_distance; // flag stub to use for calculating distances in matrix in mld in the
                              // future

    std::vector<EdgeWeight> weights(phantom_indices.size(), INVALID_EDGE_WEIGHT);
    std::vector<EdgeDuration> durations(phantom_indices.size(), MAXIMAL_EDGE_DURATION);
    std::vector<EdgeDistance> distances(phantom_indices.size(), INVALID_EDGE_DISTANCE);

    // Collect destination (source) nodes into a map
    std::unordered_multimap<NodeID, std::tuple<std::size_t, EdgeWeight, EdgeDuration>>
        target_nodes_index;
    target_nodes_index.reserve(phantom_indices.size());
    for (std::size_t index = 0; index < phantom_indices.size(); ++index)
    {
        const auto &phantom_index = phantom_indices[index];
        const auto &phantom_node = phantom_nodes[phantom_index];

        if (DIRECTION == FORWARD_DIRECTION)
        {
            if (phantom_node.IsValidForwardTarget())
                target_nodes_index.insert(
                    {phantom_node.forward_segment_id.id,
                     std::make_tuple(index,
                                     phantom_node.GetForwardWeightPlusOffset(),
                                     phantom_node.GetForwardDuration())});
            if (phantom_node.IsValidReverseTarget())
                target_nodes_index.insert(
                    {phantom_node.reverse_segment_id.id,
                     std::make_tuple(index,
                                     phantom_node.GetReverseWeightPlusOffset(),
                                     phantom_node.GetReverseDuration())});
        }
        else if (DIRECTION == REVERSE_DIRECTION)
        {
            if (phantom_node.IsValidForwardSource())
                target_nodes_index.insert(
                    {phantom_node.forward_segment_id.id,
                     std::make_tuple(index,
                                     -phantom_node.GetForwardWeightPlusOffset(),
                                     -phantom_node.GetForwardDuration())});
            if (phantom_node.IsValidReverseSource())
                target_nodes_index.insert(
                    {phantom_node.reverse_segment_id.id,
                     std::make_tuple(index,
                                     -phantom_node.GetReverseWeightPlusOffset(),
                                     -phantom_node.GetReverseDuration())});
        }
    }

    // Initialize query heap
    engine_working_data.InitializeOrClearManyToManyThreadLocalStorage(
        facade.GetNumberOfNodes(), facade.GetMaxBorderNodeID() + 1);
    auto &query_heap = *(engine_working_data.many_to_many_heap);

    // Check if node is in the destinations list and update weights/durations
    auto update_values = [&](NodeID node, EdgeWeight weight, EdgeDuration duration) {
        auto candidates = target_nodes_index.equal_range(node);
        for (auto it = candidates.first; it != candidates.second;)
        {
            std::size_t index;
            EdgeWeight target_weight;
            EdgeDuration target_duration;
            std::tie(index, target_weight, target_duration) = it->second;

            const auto path_weight = weight + target_weight;
            if (path_weight >= 0)
            {
                const auto path_duration = duration + target_duration;

                if (std::tie(path_weight, path_duration) <
                    std::tie(weights[index], durations[index]))
                {
                    weights[index] = path_weight;
                    durations[index] = path_duration;
                }

                // Remove node from destinations list
                it = target_nodes_index.erase(it);
            }
            else
            {
                ++it;
            }
        }
    };

    // Check a single path result and insert adjacent nodes into heap
    auto insert_node = [&](NodeID node, EdgeWeight initial_weight, EdgeDuration initial_duration) {

        // Update single node paths
        update_values(node, initial_weight, initial_duration);

        // Place adjacent nodes into heap
        for (auto edge : facade.GetAdjacentEdgeRange(node))
        {
            const auto &data = facade.GetEdgeData(edge);
            if (DIRECTION == FORWARD_DIRECTION ? data.forward : data.backward)
            {
                query_heap.Insert(facade.GetTarget(edge),
                                  data.weight + initial_weight,
                                  {node, data.duration + initial_duration});
            }
        }
    };

    { // Place source (destination) adjacent nodes into the heap
        const auto &phantom_node = phantom_nodes[phantom_index];

        if (DIRECTION == FORWARD_DIRECTION)
        {

            if (phantom_node.IsValidForwardSource())
                insert_node(phantom_node.forward_segment_id.id,
                            -phantom_node.GetForwardWeightPlusOffset(),
                            -phantom_node.GetForwardDuration());

            if (phantom_node.IsValidReverseSource())
                insert_node(phantom_node.reverse_segment_id.id,
                            -phantom_node.GetReverseWeightPlusOffset(),
                            -phantom_node.GetReverseDuration());
        }
        else if (DIRECTION == REVERSE_DIRECTION)
        {
            if (phantom_node.IsValidForwardTarget())
                insert_node(phantom_node.forward_segment_id.id,
                            phantom_node.GetForwardWeightPlusOffset(),
                            phantom_node.GetForwardDuration());

            if (phantom_node.IsValidReverseTarget())
                insert_node(phantom_node.reverse_segment_id.id,
                            phantom_node.GetReverseWeightPlusOffset(),
                            phantom_node.GetReverseDuration());
        }
    }

    while (!query_heap.Empty() && !target_nodes_index.empty())
    {
        // Extract node from the heap
        const auto node = query_heap.DeleteMin();
        const auto weight = query_heap.GetKey(node);
        const auto duration = query_heap.GetData(node).duration;

        // Update values
        update_values(node, weight, duration);

        // Relax outgoing edges
        relaxOutgoingEdges<DIRECTION>(facade,
                                      node,
                                      weight,
                                      duration,
                                      query_heap,
                                      phantom_nodes,
                                      phantom_index,
                                      phantom_indices);
    }

    return std::make_pair(durations, distances);
}

//
// Bidirectional multi-layer Dijkstra search for M-to-N matrices
//
template <bool DIRECTION>
void forwardRoutingStep(const DataFacade<Algorithm> &facade,
                        const unsigned row_idx,
                        const unsigned number_of_sources,
                        const unsigned number_of_targets,
                        typename SearchEngineData<Algorithm>::ManyToManyQueryHeap &query_heap,
                        const std::vector<NodeBucket> &search_space_with_buckets,
                        std::vector<EdgeWeight> &weights_table,
                        std::vector<EdgeDuration> &durations_table,
                        std::vector<NodeID> &middle_nodes_table,
                        const PhantomNode &phantom_node)
{
    const auto node = query_heap.DeleteMin();
    const auto source_weight = query_heap.GetKey(node);
    const auto source_duration = query_heap.GetData(node).duration;

    // Check if each encountered node has an entry
    const auto &bucket_list = std::equal_range(search_space_with_buckets.begin(),
                                               search_space_with_buckets.end(),
                                               node,
                                               NodeBucket::Compare());
    for (const auto &current_bucket : boost::make_iterator_range(bucket_list))
    {
        // Get target id from bucket entry
        const auto column_idx = current_bucket.column_index;
        const auto target_weight = current_bucket.weight;
        const auto target_duration = current_bucket.duration;

        // Get the value location in the results tables:
        //  * row-major direct (row_idx, column_idx) index for forward direction
        //  * row-major transposed (column_idx, row_idx) for reversed direction
        const auto location = DIRECTION == FORWARD_DIRECTION
                                  ? row_idx * number_of_targets + column_idx
                                  : row_idx + column_idx * number_of_sources;
        auto &current_weight = weights_table[location];
        auto &current_duration = durations_table[location];
        middle_nodes_table[location] = node;

        // Check if new weight is better
        auto new_weight = source_weight + target_weight;
        auto new_duration = source_duration + target_duration;

        if (new_weight >= 0 &&
            std::tie(new_weight, new_duration) < std::tie(current_weight, current_duration))
        {
            current_weight = new_weight;
            current_duration = new_duration;
        }
    }

    relaxOutgoingEdges<DIRECTION>(
        facade, node, source_weight, source_duration, query_heap, phantom_node);
}

template <bool DIRECTION>
void backwardRoutingStep(const DataFacade<Algorithm> &facade,
                         const unsigned column_idx,
                         typename SearchEngineData<Algorithm>::ManyToManyQueryHeap &query_heap,
                         std::vector<NodeBucket> &search_space_with_buckets,
                         const PhantomNode &phantom_node)
{
    const auto node = query_heap.DeleteMin();
    const auto target_weight = query_heap.GetKey(node);
    const auto target_duration = query_heap.GetData(node).duration;
    const auto parent = query_heap.GetData(node).parent;

    // Store settled nodes in search space bucket
    search_space_with_buckets.emplace_back(
        node, parent, column_idx, target_weight, target_duration);

    const auto &partition = facade.GetMultiLevelPartition();
    const auto maximal_level = partition.GetNumberOfLevels() - 1;

    relaxOutgoingEdges<!DIRECTION>(
        facade, node, target_weight, target_duration, query_heap, phantom_node, maximal_level);
}

void retrievePackedPathFromSearchSpace(NodeID middle_node_id,
                                       const unsigned column_idx,
                                       const std::vector<NodeBucket> &search_space_with_buckets,
                                       std::vector<NodeID> &packed_leg)
{

    //     [  0           1          2         3    ]
    //     [ [m0,p=m3],[m1,p=m2],[m2,p=m1], [m3,p=2]]

    //           targets (columns) target_id = column_idx
    //              a   b   c
    //          a  [0,  1,  2],
    // sources  b  [3,  4,  5],
    //  (rows)  c  [6,  7,  8],
    //          d  [9, 10, 11]
    // row_idx * number_of_targets + column_idx
    // a -> c 0 * 3 + 2 = 2
    // c -> c 2 * 3 + 2 = 8
    // d -> c 3 * 3 + 2 = 11

    //   middle_nodes_table = [0 , 1, 2, .........]

    auto bucket_list = std::equal_range(search_space_with_buckets.begin(),
                                        search_space_with_buckets.end(),
                                        middle_node_id,
                                        NodeBucket::ColumnCompare(column_idx));

    NodeID current_node_id = middle_node_id;

    BOOST_ASSERT_MSG(std::distance(bucket_list.first, bucket_list.second) == 1,
                     "The pointers are not pointing to the same element.");

    while (bucket_list.first->parent_node != current_node_id &&
           bucket_list.first != search_space_with_buckets.end())
    {
        current_node_id = bucket_list.first->parent_node;

        packed_leg.emplace_back(current_node_id);

        bucket_list = std::equal_range(search_space_with_buckets.begin(),
                                       search_space_with_buckets.end(),
                                       current_node_id,
                                       NodeBucket::ColumnCompare(column_idx));
    }
}

template <bool DIRECTION>
void calculateDistances(typename SearchEngineData<mld::Algorithm>::ManyToManyQueryHeap &query_heap,
                        const DataFacade<mld::Algorithm> &facade,
                        const std::vector<PhantomNode> &phantom_nodes,
                        const std::vector<std::size_t> &target_indices,
                        const unsigned row_idx,
                        const std::size_t source_index,
                        const PhantomNode &source_phantom,
                        const unsigned number_of_sources,
                        const unsigned number_of_targets,
                        const std::vector<NodeBucket> &search_space_with_buckets,
                        std::vector<EdgeDistance> &distances_table,
                        const std::vector<NodeID> &middle_nodes_table)
{
    std::vector<NodeID> packed_leg;

    (void)facade;
    (void)source_phantom;
    (void)phantom_nodes;

    // std::cout << "middle_nodes_table: ";
    // for (auto middle_node_id = middle_nodes_table.begin();
    //      middle_node_id != middle_nodes_table.end();
    //      ++middle_node_id)
    // {
    //     std::cout << *middle_node_id << ", ";
    // }
    // std::cout << std::endl;

    // TODO: CREATE UNIT TESTS FOR EACH OF THE RETRIEVING PACK PATH FUNCTIONS
    // 1. Recreate packed path
    // 2. Unpack path
    // 3. Offset the durations

    std::cout << "do I get to before path unpacking?" << std::endl;
    for (unsigned column_idx = 0; column_idx < number_of_targets; ++column_idx)
    {
        const auto location = DIRECTION == FORWARD_DIRECTION
                                  ? row_idx * number_of_targets + column_idx
                                  : row_idx + column_idx * number_of_sources;

        auto target_index = target_indices[column_idx];

        if (source_index == target_index)
        {
            distances_table[location] = 0.0;
            continue;
        }
        // const auto &target_phantom = phantom_nodes[target_index];
        NodeID middle_node_id = middle_nodes_table[location];

        // std::cout << "source_phantom f id: " << source_phantom.forward_segment_id.id << " "
        //           << "source_phantom r id: " << source_phantom.reverse_segment_id.id << std::endl;
        // std::cout << "target_phantom f id: " << target_phantom.forward_segment_id.id << " "
        //           << "target_phantom r id: " << target_phantom.reverse_segment_id.id << std::endl;

        if (middle_node_id == SPECIAL_NODEID) // takes care of one-ways
        {
            distances_table[location] = INVALID_EDGE_DISTANCE;
            continue;
        }

        // from_clique_arc tells you if you are not on the base graph

        std::cout << "middle_node_id: " << middle_node_id << std::endl;

        using PackedEdge = std::tuple</*from*/ NodeID, /*to*/ NodeID, /*from_clique_arc*/ bool>; 
        // https://github.com/Project-OSRM/osrm-backend/blob/master/include/engine/routing_algorithms/routing_base_mld.hpp#L363
        // the bool tells you -- is this an overlay edge or not?

        using PackedPath = std::vector<PackedEdge>;

        // Step 1: Find path from source to middle node
        PackedPath packed_path_from_source_to_middle =
            mld::retrievePackedPathFromSingleManyToManyHeap<DIRECTION>(
                query_heap,
                middle_node_id); // packed_leg_from_source_to_middle

        std::cout << "packed_path_from_source_to_middle.size(): "
                  << packed_path_from_source_to_middle.size() << std::endl;
        std::cout << "packed_path_from_source_to_middle: "
                  << packed_path_from_source_to_middle.size() << std::endl;
        for (auto packed_edge : packed_path_from_source_to_middle)
        {
            std::cout << "packed_edge_from: " << std::get<0>(packed_edge)
                      << " packed_edge_to: " << std::get<1>(packed_edge)
                      << " packed_edge_from_clique_arc: " << std::get<2>(packed_edge) << std::endl;
        }
        std::cout << std::endl;

        std::reverse(packed_leg.begin(), packed_leg.end());

        packed_leg.push_back(middle_node_id);

        // Step 2: Find path from middle to target node
        retrievePackedPathFromSearchSpace(middle_node_id,
                                          column_idx,
                                          search_space_with_buckets,
                                          packed_leg); // packed_leg_from_middle_to_target

        std::cout << "packed_leg: ";
        for (auto node_id : packed_leg)
        {
            std::cout << "node_id: " << node_id << ", ";
        }
        std::cout << std::endl;

        // call site for figuring out what node level to start at
        // https://github.com/Project-OSRM/osrm-backend/blob/faff2c774d09a7227c77ae4fa40d22d54bb00b45/include/engine/routing_algorithms/routing_base_mld.hpp#L145

        // unpacking and distance calculations down here
    }
}

template <bool DIRECTION>
std::pair<std::vector<EdgeDuration>, std::vector<EdgeDistance>>
manyToManySearch(SearchEngineData<Algorithm> &engine_working_data,
                 const DataFacade<Algorithm> &facade,
                 const std::vector<PhantomNode> &phantom_nodes,
                 const std::vector<std::size_t> &source_indices,
                 const std::vector<std::size_t> &target_indices,
                 const bool calculate_distance)
{
    const auto number_of_sources = source_indices.size();
    const auto number_of_targets = target_indices.size();
    const auto number_of_entries = number_of_sources * number_of_targets;

    std::vector<EdgeWeight> weights_table(number_of_entries, INVALID_EDGE_WEIGHT);
    std::vector<EdgeDuration> durations_table(number_of_entries, MAXIMAL_EDGE_DURATION);
    std::vector<EdgeDistance> distances_table;
    std::vector<NodeID> middle_nodes_table(number_of_entries, SPECIAL_NODEID);

    std::vector<NodeBucket> search_space_with_buckets;
    std::vector<NodeID> packed_leg;

    // Populate buckets with paths from all accessible nodes to destinations via backward searches
    for (std::uint32_t column_idx = 0; column_idx < target_indices.size(); ++column_idx)
    {
        const auto index = target_indices[column_idx];
        const auto &target_phantom = phantom_nodes[index];

        engine_working_data.InitializeOrClearManyToManyThreadLocalStorage(
            facade.GetNumberOfNodes(), facade.GetMaxBorderNodeID() + 1);
        auto &query_heap = *(engine_working_data.many_to_many_heap);

        if (DIRECTION == FORWARD_DIRECTION)
            insertTargetInHeap(query_heap, target_phantom);
        else
            insertSourceInHeap(query_heap, target_phantom);

        // explore search space
        while (!query_heap.Empty())
        {
            backwardRoutingStep<DIRECTION>(
                facade, column_idx, query_heap, search_space_with_buckets, target_phantom);
        }
    }

    // Order lookup buckets
    std::sort(search_space_with_buckets.begin(), search_space_with_buckets.end());

    // Find shortest paths from sources to all accessible nodes
    for (std::uint32_t row_idx = 0; row_idx < source_indices.size(); ++row_idx)
    {
        const auto source_index = source_indices[row_idx];
        const auto &source_phantom = phantom_nodes[source_index];

        // Clear heap and insert source nodes
        engine_working_data.InitializeOrClearManyToManyThreadLocalStorage(
            facade.GetNumberOfNodes(), facade.GetMaxBorderNodeID() + 1);

        auto &query_heap = *(engine_working_data.many_to_many_heap);

        if (DIRECTION == FORWARD_DIRECTION)
            insertSourceInHeap(query_heap, source_phantom);
        else
            insertTargetInHeap(query_heap, source_phantom);

        // Explore search space
        while (!query_heap.Empty())
        {
            forwardRoutingStep<DIRECTION>(facade,
                                          row_idx,
                                          number_of_sources,
                                          number_of_targets,
                                          query_heap,
                                          search_space_with_buckets,
                                          weights_table,
                                          durations_table,
                                          middle_nodes_table,
                                          source_phantom);
        }
        // find the shortcutted path
        // unpack it
        // calculate the distances while unpacking the path

        if (calculate_distance)
        {
            distances_table.resize(number_of_entries, INVALID_EDGE_DISTANCE);
            calculateDistances<DIRECTION>(query_heap,
                                          facade,
                                          phantom_nodes,
                                          target_indices,
                                          row_idx,
                                          source_index,
                                          source_phantom,
                                          number_of_sources,
                                          number_of_targets,
                                          search_space_with_buckets,
                                          distances_table,
                                          middle_nodes_table);
        }
    }

    return std::make_pair(durations_table, distances_table);
}

} // namespace mld

// Dispatcher function for one-to-many and many-to-one tasks that can be handled by MLD differently:
//
// * one-to-many (many-to-one) tasks use a unidirectional forward (backward) Dijkstra search
//   with the candidate node level `min(GetQueryLevel(phantom_node, node, phantom_nodes)`
//   for all destination (source) phantom nodes
//
// * many-to-many search tasks use a bidirectional Dijkstra search
//   with the candidate node level `min(GetHighestDifferentLevel(phantom_node, node))`
//   Due to pruned backward search space it is always better to compute the durations matrix
//   when number of sources is less than targets. If number of targets is less than sources
//   then search is performed on a reversed graph with phantom nodes with flipped roles and
//   returning a transposed matrix.
template <>
std::pair<std::vector<EdgeDuration>, std::vector<EdgeDistance>>
manyToManySearch(SearchEngineData<mld::Algorithm> &engine_working_data,
                 const DataFacade<mld::Algorithm> &facade,
                 const std::vector<PhantomNode> &phantom_nodes,
                 const std::vector<std::size_t> &source_indices,
                 const std::vector<std::size_t> &target_indices,
                 const bool calculate_distance)
{
    if (source_indices.size() == 1)
    { // TODO: check if target_indices.size() == 1 and do a bi-directional search
        return mld::oneToManySearch<FORWARD_DIRECTION>(engine_working_data,
                                                       facade,
                                                       phantom_nodes,
                                                       source_indices.front(),
                                                       target_indices,
                                                       calculate_distance);
    }

    if (target_indices.size() == 1)
    {
        return mld::oneToManySearch<REVERSE_DIRECTION>(engine_working_data,
                                                       facade,
                                                       phantom_nodes,
                                                       target_indices.front(),
                                                       source_indices,
                                                       calculate_distance);
    }

    if (target_indices.size() < source_indices.size())
    {
        return mld::manyToManySearch<REVERSE_DIRECTION>(engine_working_data,
                                                        facade,
                                                        phantom_nodes,
                                                        target_indices,
                                                        source_indices,
                                                        calculate_distance);
    }

    return mld::manyToManySearch<FORWARD_DIRECTION>(engine_working_data,
                                                    facade,
                                                    phantom_nodes,
                                                    source_indices,
                                                    target_indices,
                                                    calculate_distance);
}

} // namespace routing_algorithms
} // namespace engine
} // namespace osrm
