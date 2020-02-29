#ifndef GRAPH_GENERATOR_H_
#define GRAPH_GENERATOR_H_

#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>
#include "network.h"

class GraphGenerator {
  //  public:
  //   static std::shared_ptr<GraphGenerator> Make(
  //       const std::array<row, 303>& network, double discretization_factor,
  //       double max_range);
  // change to private constructor
 public:
  GraphGenerator(const std::array<row, 303>& network,
                 double discretization_factor, double max_range, double speed)
      : network_(network),
        num_chargers_(network.size()),
        discretization_factor_(discretization_factor),
        max_range_(max_range),
        speed_(speed) {
    std::cout << "111" << std::endl;
    num_nodes_ = num_chargers_ * discretization_factor_;
    // initialize the distance matrix with all zeros
    for (int i = 0; i < num_chargers_; ++i) {
      distances_.push_back(std::vector<double>(num_chargers_, 0.0));
    }
    std::cout << "222" << std::endl;
    // initialize the edges with default -1.0 (invalid)
    std::cout << num_nodes_ << std::endl;
    for (int i = 0; i < num_nodes_; ++i) {
      edges_.push_back(std::vector<double>(num_nodes_, -1.0));
      costs_.push_back(std::numeric_limits<double>::max());
      visited_.push_back(false);
    }
    std::cout << "333" << std::endl;
    unit_range_ = max_range_ / (discretization_factor_ - 1);
  }

  void generateGraph() {
    // calculate and fill the distance matrix
    for (int i = 0; i < num_chargers_; ++i) {
      for (int j = i + 1; j < num_chargers_; ++j) {
        // maybe check for error input here
        double d = calcDistance(network[i].lat, network[i].lon, network[j].lat,
                                network[j].lon);
        distances_[i][j] = d;
        distances_[j][i] = d;
      }
    }

    // construct edges with cost
    for (int i = 0; i < num_nodes_; ++i) {
      for (int j = 0; j < num_nodes_; ++j) {
        int id1 = i / discretization_factor_;
        int id2 = j / discretization_factor_;
        int r1 = i % discretization_factor_;
        int r2 = j % discretization_factor_;

        if (id1 == id2) {
          // these two nodes belong to the same charger
          if (r2 - r1 == 1) {
            // the time it takes to charge for 1 unit range
            edges_[i][j] = unit_range_ / network[id1].rate;
          }
        } else {
          // different chargers
          if ((r1 - r2) * unit_range_ >= distances_[id1][id2]) {
            std::cout << "Edge added for " << network_[id1].name << " to "
                      << network_[id2].name
                      << " with cost: " << distances_[id1][id2] / speed_
                      << std::endl;
            edges_[i][j] = distances_[id1][id2] / speed_;
          }
        }
      }
    }
  }

  void search(const std::string& start, const std::string& goal) {
    int start_id = getChargerID(start);
    int goal_id = getChargerID(goal);
    if (start_id == -1 || goal_id == -1) {
      // do something
      std::cout << "error" << std::endl;
      return;
    }
    // check id is not -1
    // initializeHeuristics(goal_id);

    int start_node_id = (start_id + 1) * discretization_factor_ - 1;
    double heuristic = distances_[start_id][goal_id];

    costs_[start_node_id] = 0.0;

    visited_[start_id] = true;
    open_list_.push({heuristic, start_node_id});

    while (!open_list_.empty()) {
      // get the f_id pair with smallest F value (F = G + H)
      int curr_node_id = open_list_.top().second;
      double curr_cost = costs_[curr_node_id];
      open_list_.pop();

      int curr_charger_id = curr_node_id / discretization_factor_;
      int curr_range = curr_node_id % discretization_factor_ * unit_range_;
      visited_[curr_node_id] = true;
      std::cout << network_[curr_charger_id].name << std::endl;
      if (curr_charger_id == goal_id) {
        std::cout << "Goal found!" << std::endl;
        std::cout << "Cost: " << costs_[curr_node_id] << std::endl;
        return;
      }

      for (int i = 0; i < num_nodes_; ++i) {
        int next_charger_id = i / discretization_factor_;
        if (edges_[curr_node_id][i] != -1.0 && !visited_[i]) {
          // a valid edge and target node not fully explored yet
          double new_cost = curr_cost + edges_[curr_node_id][i];
          std::cout << new_cost << std::endl;
          if (new_cost < costs_[i]) {
            // update cost
            costs_[i] = new_cost;
            // push into open_list
            open_list_.push(
                {new_cost + distances_[next_charger_id][goal_id], i});
            // update parent
          }
        }
      }
    }
  }

  int getChargerID(const std::string& name) {
    for (int i = 0; i < network_.size(); ++i) {
      if (network_[i].name == name) {
        return i;
      }
    }
    return -1;
  }

  std::string getChargerName(int id) {
    if (id < 0 || id >= network_.size()) {
      // error
      return std::string();
    }
    return network_[id].name;
  }

 private:
  inline double deg2Rad(double deg) const { return deg * pi_ / 180; }
  double calcDistance(double lat1, double lon1, double lat2, double lon2) const;
  // assuming earth is a sphere with radius 6356.752km
  double earth_radius_ = 6356.752;
  double pi_ = 3.14159265;

  int num_chargers_;
  int num_nodes_;
  int discretization_factor_;
  double max_range_;
  double unit_range_;
  double speed_;
  // charger_name -> {id, charging_rate}
  // std::unordered_map<int, row> charger_dict_;
  std::array<row, 303> network_;
  std::vector<std::vector<double>> distances_;
  std::vector<std::vector<double>> edges_;
  std::vector<double> costs_;
  std::vector<bool> visited_;
  std::priority_queue<std::pair<double, int>,
                      std::vector<std::pair<double, int>>,
                      std::greater<std::pair<double, int>>>
      open_list_;
};

#endif