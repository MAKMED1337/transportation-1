#pragma once

#include "algorithms/routing_algorithm.hpp"
#include "graph/graph.hpp"

#include <map>
#include <memory>
#include <string>

namespace transport {

using AlgorithmParams = std::map<std::string, std::string>;

// Helpers for safe parameter extraction
uint32_t param_u32(const AlgorithmParams &p, const std::string &key, uint32_t def);
double param_double(const AlgorithmParams &p, const std::string &key, double def);
std::string param_str(const AlgorithmParams &p, const std::string &key, std::string def);

std::unique_ptr<RoutingAlgorithm> make_routing_algorithm(const std::string &name, const Graph &graph,
                                                          const AlgorithmParams &params = {});

} // namespace transport
