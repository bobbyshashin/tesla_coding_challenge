#include "graph_generator.h"

std::unique_ptr<GraphGenerator> GraphGenerator::Make(
    double discretization_factor, double max_range, double speed) {
  return std::unique_ptr<GraphGenerator>(
      new GraphGenerator(discretization_factor, max_range, speed));
}

GraphGenerator::GraphGenerator(double discretization_factor, double max_range,
                               double speed)
    : num_chargers_(network.size()),
      discretization_factor_(discretization_factor),
      num_nodes_(num_chargers_ * discretization_factor_),
      max_range_(max_range),
      unit_range_(max_range_ / (discretization_factor_ - 1)),
      speed_(speed),
      distances_(num_chargers_, std::vector<double>(num_chargers_, 0.0)),
      costs_(num_nodes_, std::numeric_limits<double>::max()),
      visited_(num_nodes_, false),
      parents_(num_nodes_, -1) {}

void GraphGenerator::generateGraph() {
  // calculate and fill the distance matrix
  for (int i = 0; i < num_chargers_; ++i) {
    for (int j = i + 1; j < num_chargers_; ++j) {
      double d = calcDistance(network[i].lat, network[i].lon, network[j].lat,
                              network[j].lon);
      distances_[i][j] = d;
      distances_[j][i] = d;
    }
  }

  // construct all edges with cost (charging / travelling time in hours)
  for (int i = 0; i < num_nodes_; ++i) {
    for (int j = 0; j < num_nodes_; ++j) {
      int id1 = getChargerID(i);
      int id2 = getChargerID(j);
      int r1 = getRangeID(i);
      int r2 = getRangeID(j);

      if (id1 == id2) {
        // these two nodes correspond to the same charger
        if (r2 - r1 == 1) {
          // we only add an edge if the states are adjacent
          // with the cost as the time it takes to charge for 1 unit range
          edges_[i].push_back({unit_range_ / network[id1].rate, j});
        }
      } else {
        // different chargers
        // we add an edge if and only if:
        // range_curr_charger - range_next_charger >= distance between them
        if ((r1 - r2) * unit_range_ >= distances_[id1][id2]) {
          edges_[i].push_back({distances_[id1][id2] / speed_, j});
        }
      }
    }
  }
}

void GraphGenerator::search(const std::string& start, const std::string& goal) {
  // try to find start and goal supercharger in the network
  int start_id = findChargerID(start);
  int goal_id = findChargerID(goal);

  bool no_start = start_id == -1;
  bool no_goal = goal_id == -1;
  if (no_start || no_goal) {
    std::string s;
    if (no_start && no_goal) {
      s = start + " and " + goal;
    } else {
      s = no_start ? start : goal;
    }
    std::cout << "Error: Cannot find supercharger " << s << " in the network!"
              << std::endl;
    return;
  }

  // initialize the start node
  int start_node_id = (start_id + 1) * discretization_factor_ - 1;
  double h_start = distances_[start_id][goal_id] / speed_;

  costs_[start_node_id] = 0.0;
  visited_[start_node_id] = true;
  open_list_.push({h_start, start_node_id});

  // perform A* search
  while (!open_list_.empty()) {
    // get the CostID pair with smallest F value (F = cost + heuristics)
    int curr_node_id = open_list_.top().second;
    double curr_cost = costs_[curr_node_id];
    open_list_.pop();

    int curr_charger_id = getChargerID(curr_node_id);
    int curr_range = getRangeID(curr_node_id) * unit_range_;
    visited_[curr_node_id] = true;

    // if goal is found, backtrace to get the optimal path
    if (curr_charger_id == goal_id) {
      backtracePath(curr_node_id);
      return;
    }

    for (const auto& next : edges_[curr_node_id]) {
      int next_node_id = next.second;
      int next_charger_id = getChargerID(next_node_id);
      if (!visited_[next_node_id]) {
        // a valid edge and target node not fully explored yet
        double new_cost = curr_cost + next.first;

        if (new_cost < costs_[next_node_id]) {
          // update the state if cost through curr_node is smaller
          costs_[next_node_id] = new_cost;
          double h = calcHeuristics(next_charger_id, goal_id);
          open_list_.push({new_cost + h, next_node_id});
          parents_[next_node_id] = curr_node_id;
        }
      }
    }
  }
  // Failed to find a feasible path
  std::cout << "Failed, cannot find feasible path from " << start << " to "
            << goal << std::endl;
}

double GraphGenerator::calcDistance(double lat1, double lon1, double lat2,
                                    double lon2) const {
  return earth_radius_ * acos(sin(deg2Rad(lat1)) * sin(deg2Rad(lat2)) +
                              cos(deg2Rad(lat1)) * cos(deg2Rad(lat2)) *
                                  cos(deg2Rad(fabs(lon1 - lon2))));
}

double GraphGenerator::calcHeuristics(int id1, int id2) const {
  return distances_[id1][id2] / speed_;
}

int GraphGenerator::findChargerID(const std::string& name) const {
  for (int i = 0; i < network.size(); ++i) {
    if (network[i].name == name) {
      return i;
    }
  }
  return -1;
}

std::string GraphGenerator::findChargerName(int id) const {
  if (id < 0 || id >= network.size()) {
    std::cout << "Error: invalid charger id: " << id << "!" << std::endl;
    return std::string();
  }
  return network[id].name;
}

void GraphGenerator::backtracePath(int goal_node_id) {
  int parent_id = parents_[goal_node_id];
  std::vector<std::string> chargers;
  std::vector<double> charging_time;

  int goal_id = getChargerID(goal_node_id);
  charging_time.push_back(getRangeID(goal_node_id) * unit_range_ /
                          network[goal_id].rate);
  chargers.push_back(network[goal_id].name);

  // backtracing until we reach the starting location
  while (parent_id != -1) {
    int counter = 0;
    while (getChargerID(parents_[parent_id]) == getChargerID(parent_id)) {
      ++counter;
      parent_id = parents_[parent_id];
    }
    int charger_id = getChargerID(parent_id);
    charging_time.push_back(counter * unit_range_ / network[charger_id].rate);
    chargers.push_back(network[charger_id].name);

    parent_id = parents_[parent_id];
  }

  // print the path in required format
  std::cout << chargers[chargers.size() - 1] << ", ";
  for (int i = chargers.size() - 2; i > 0; --i) {
    std::cout << chargers[i] << ", " << charging_time[i] << ", ";
  }
  std::cout << chargers[0];
}