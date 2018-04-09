#ifndef MAP_MERGE_GRAPH_H_
#define MAP_MERGE_GRAPH_H_

/*
These graph functions has been partially adapted from OpenCV stitching module
see opencv2/stitching/detail/util.hpp
https://github.com/opencv/opencv/blob/master/modules/stitching/include/opencv2/stitching/detail/util.hpp

Many modifications has been done to fit our pipeline. OpenCV uses the same
license as this project (BSD).
*/

#include <list>
#include <queue>
#include <vector>

#include <Eigen/Core>

using std::size_t;

/**
 * @brief Represents transformation estiamte between 2 clouds
 */
struct TransformEstimate {
  TransformEstimate() = default;
  TransformEstimate(size_t source_idx_, size_t target_idx_)
    : source_idx(source_idx_), target_idx(target_idx_)
  {
  }

  size_t source_idx;  // source cloud index
  size_t target_idx;

  Eigen::Matrix4f transform;
  double confidence = 0.0;
};

class DisjointSets
{
public:
  DisjointSets(size_t elem_count = 0)
  {
    createOneElemSets(elem_count);
  }

  void createOneElemSets(size_t elem_count);
  size_t findSetByElem(size_t elem);
  size_t mergeSets(size_t set1, size_t set2);

  std::vector<size_t> parent;
  std::vector<size_t> size;

private:
  std::vector<size_t> rank_;
};

struct GraphEdge {
  GraphEdge(size_t from, size_t to, double weight);
  bool operator<(const GraphEdge& other) const
  {
    return weight < other.weight;
  }
  bool operator>(const GraphEdge& other) const
  {
    return weight > other.weight;
  }

  size_t from, to;
  double weight;
};

inline GraphEdge::GraphEdge(size_t _from, size_t _to, double _weight)
  : from(_from), to(_to), weight(_weight)
{
}

class Graph
{
public:
  Graph(size_t num_vertices = 0)
  {
    create(num_vertices);
  }
  void create(size_t num_vertices)
  {
    edges_.assign(num_vertices, std::list<GraphEdge>());
  }
  size_t numVertices() const
  {
    return edges_.size();
  }
  void addEdge(size_t from, size_t to, double weight);
  template <typename B>
  B forEach(B body) const;
  template <typename B>
  B walkBreadthFirst(size_t from, B body) const;

private:
  std::vector<std::list<GraphEdge>> edges_;
};

template <typename B>
B Graph::forEach(B body) const
{
  for (size_t i = 0; i < numVertices(); ++i) {
    std::list<GraphEdge>::const_iterator edge = edges_[i].begin();
    for (; edge != edges_[i].end(); ++edge)
      body(*edge);
  }
  return body;
}

template <typename B>
B Graph::walkBreadthFirst(size_t from, B body) const
{
  std::vector<bool> was(numVertices(), false);
  std::queue<size_t> vertices;

  was[from] = true;
  vertices.push(from);

  while (!vertices.empty()) {
    size_t vertex = vertices.front();
    vertices.pop();

    std::list<GraphEdge>::const_iterator edge = edges_[vertex].begin();
    for (; edge != edges_[vertex].end(); ++edge) {
      if (!was[edge->to]) {
        body(*edge);
        was[edge->to] = true;
        vertices.push(edge->to);
      }
    }
  }

  return body;
}

/**
 * @brief Returns the largest connect component of estimates
 * @details Returns subset of estimates that are the largest connected component
 * with edges having larger confidence than conf_threshold.
 *
 * @param pairwise_estimates Pairwise estimated tranforms between nodes
 * @param conf_threshold Confidence threshold. Only edges with higher confidence
 * than this threshold will be considered.
 *
 * @return the largest connected component
 */
std::vector<TransformEstimate> largestConnectedComponent(
    const std::vector<TransformEstimate>& pairwise_estimates,
    double conf_threshold = 0.0);

/**
 * @brief Computes maximum spanning tree in the graph of the estimates
 * @details Uses confidence to weight edges.
 *
 * @param pairwise_estimates estimates graph edges. Must be connected graph.
 * @param span_tree returned spanning tree
 * @param centers returned centers of the spanning tree
 */
void findMaxSpanningTree(
    const std::vector<TransformEstimate>& pairwise_estimates, Graph& span_tree,
    std::vector<size_t>& centers);

size_t numberOfNodesInEstimates(
    const std::vector<TransformEstimate>& pairwise_estimates);

#endif  // MAP_MERGE_GRAPH_H_
