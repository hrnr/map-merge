#include "graph.h"

#include <algorithm>
#include <cassert>
#include <unordered_set>

size_t numberOfNodesInEstimates(
    const std::vector<TransformEstimate> &pairwise_estimates)
{
  size_t num_nodes = 0;
  for (const auto &est : pairwise_estimates) {
    num_nodes = std::max({num_nodes, est.source_idx + 1, est.target_idx + 1});
  }
  return num_nodes;
}

void DisjointSets::createOneElemSets(size_t n)
{
  rank_.assign(n, 0);
  size.assign(n, 1);
  parent.resize(n);
  for (size_t i = 0; i < n; ++i) {
    parent[i] = i;
  }
}

size_t DisjointSets::findSetByElem(size_t elem)
{
  size_t set = elem;
  while (set != parent[set])
    set = parent[set];
  size_t next;
  while (elem != parent[elem]) {
    next = parent[elem];
    parent[elem] = set;
    elem = next;
  }
  return set;
}

size_t DisjointSets::mergeSets(size_t set1, size_t set2)
{
  if (rank_[set1] < rank_[set2]) {
    parent[set1] = set2;
    size[set2] += size[set1];
    return set2;
  }
  if (rank_[set2] < rank_[set1]) {
    parent[set2] = set1;
    size[set1] += size[set2];
    return set1;
  }
  parent[set1] = set2;
  rank_[set2]++;
  size[set2] += size[set1];
  return set2;
}

void Graph::addEdge(size_t from, size_t to, double weight)
{
  edges_[from].emplace_back(from, to, weight);
}

std::vector<TransformEstimate> largestConnectedComponent(
    const std::vector<TransformEstimate> &pairwise_estimates,
    double conf_threshold)
{
  std::vector<TransformEstimate> estimates_subset;
  const size_t num_nodes = numberOfNodesInEstimates(pairwise_estimates);

  if (num_nodes == 0) {
    return estimates_subset;
  }

  // merge sets that have a common edge
  DisjointSets comps(num_nodes);
  for (const auto &est : pairwise_estimates) {
    if (est.confidence < conf_threshold) {
      continue;
    }
    size_t comp1 = comps.findSetByElem(est.source_idx);
    size_t comp2 = comps.findSetByElem(est.target_idx);
    if (comp1 != comp2) {
      comps.mergeSets(comp1, comp2);
    }
  }

  // index of the first largest component
  size_t max_comp = static_cast<size_t>(
      std::distance(comps.size.begin(),
                    std::max_element(comps.size.begin(), comps.size.end())));

  // create subset of estimates belonging to the largest connected component
  for (const auto &est : pairwise_estimates) {
    // if source index is in the largest component
    if (comps.findSetByElem(est.source_idx) == max_comp) {
      estimates_subset.emplace_back(est);
    }
  }

  return estimates_subset;
}

void findMaxSpanningTree(
    const std::vector<TransformEstimate> &pairwise_estimates, Graph &span_tree,
    std::vector<size_t> &centers)
{
  const size_t num_nodes = numberOfNodesInEstimates(pairwise_estimates);

  Graph graph(num_nodes);
  std::vector<GraphEdge> edges;

  // Construct graph from pairwise_estimates
  for (const auto &est : pairwise_estimates) {
    graph.addEdge(est.source_idx, est.target_idx, est.confidence);
    edges.emplace_back(est.source_idx, est.target_idx, est.confidence);
  }

  DisjointSets comps(num_nodes);
  span_tree.create(num_nodes);
  std::vector<size_t> span_tree_powers(num_nodes, 0);

  // Find maximum spanning tree
  sort(edges.begin(), edges.end(), std::greater<GraphEdge>());
  for (size_t i = 0; i < edges.size(); ++i) {
    size_t comp1 = comps.findSetByElem(edges[i].from);
    size_t comp2 = comps.findSetByElem(edges[i].to);
    if (comp1 != comp2) {
      comps.mergeSets(comp1, comp2);
      span_tree.addEdge(edges[i].from, edges[i].to, edges[i].weight);
      span_tree.addEdge(edges[i].to, edges[i].from, edges[i].weight);
      span_tree_powers[edges[i].from]++;
      span_tree_powers[edges[i].to]++;
    }
  }

  // Find spanning tree leafs
  std::vector<size_t> span_tree_leafs;
  for (size_t i = 0; i < num_nodes; ++i) {
    if (span_tree_powers[i] == 1) {
      span_tree_leafs.push_back(i);
    }
  }

  // Find maximum distance from each spanning tree vertex
  std::vector<size_t> max_dists(num_nodes, 0);
  std::vector<size_t> cur_dists;
  for (size_t i = 0; i < span_tree_leafs.size(); ++i) {
    cur_dists.assign(num_nodes, 0);
    span_tree.walkBreadthFirst(span_tree_leafs[i], [&cur_dists](auto edge) {
      return cur_dists[edge.to] = cur_dists[edge.from] + 1;
    });
    for (size_t j = 0; j < num_nodes; ++j) {
      max_dists[j] = std::max(max_dists[j], cur_dists[j]);
    }
  }

  // Find min-max distance
  size_t min_max_dist = max_dists[0];
  for (size_t i = 1; i < num_nodes; ++i) {
    if (min_max_dist > max_dists[i]) {
      min_max_dist = max_dists[i];
    }
  }

  // Find spanning tree centers
  centers.clear();
  for (size_t i = 0; i < num_nodes; ++i) {
    if (max_dists[i] == min_max_dist) {
      centers.push_back(i);
    }
  }

  assert(centers.size() > 0 && centers.size() <= 2);
}
