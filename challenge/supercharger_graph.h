#ifndef SUPERCHARGER_GRAPH_H_
#define SUPERCHARGER_GRAPH_H_

#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>
#include "network.h"

class SuperchargerGraph {
 public:
  // delete the default constructor
  SuperchargerGraph() = delete;
  // static Maker function
  static std::unique_ptr<SuperchargerGraph> Make(double discretization_factor,
                                                 double max_range,
                                                 double speed);

  // Generate the graph
  void generateGraph();

  // Given the start and goal, search for an optimal path
  void search(const std::string& start, const std::string& goal);

 private:
  // Private constructor
  SuperchargerGraph(double discretization_factor, double max_range,
                    double speed);

  // Backtrace to find and output the optimal path
  void backtracePath(int goal_id);

  // Given latitude and longtitude of two superchargers, calculate the distance
  // between them along the great arc of earth surface
  double calcDistance(double lat1, double lon1, double lat2, double lon2) const;

  // Calculate heuristics given two superchargers' id
  double calcHeuristics(int id1, int id2) const;

  // Helper function to query the id of a supercharger given its name
  // return -1 if charger not found in the network
  int findChargerID(const std::string& name) const;

  // Helper function to query the name of a supercharger given its id
  // return empty string if charger not found in the network
  std::string findChargerName(int id) const;

  // Helper function to convert degrees to radians
  inline double deg2Rad(double deg) const { return deg * M_PI / 180; }

  // Helper functions to convert a discretized node id to the corresponding
  // charger id / range id (how much portion it is charged)
  inline int getChargerID(int node_id) const {
    return node_id / discretization_factor_;
  }
  inline int getRangeID(int node_id) const {
    return node_id % discretization_factor_;
  }

  // assuming earth is a sphere with radius 6356.752km
  double earth_radius_ = 6356.752;

  // the total number of superchargers in the network
  int num_chargers_;

  // the resolution of discretization, for example:
  // max_range = 320km, discretization_factor = 81
  // the discretized unit range for charging is 320 / (81-1) = 4km
  // (0, 4, 8, ..., 316, 320)
  int discretization_factor_;
  int num_nodes_;

  // the maximum range (km) of the fully-charged vehicle
  double max_range_;
  // the discretized unit range (km)
  double unit_range_;

  // maximum speed (km/h) of the vehicle while travelling from charger A to B
  double speed_;

  typedef std::pair<double, int> CostID;

  // components for graph searching
  std::vector<std::vector<double>> distances_;
  std::unordered_map<int, std::vector<CostID>> edges_;
  std::vector<double> costs_;
  std::vector<bool> visited_;
  std::vector<int> parents_;
  std::priority_queue<std::pair<double, int>, std::vector<CostID>,
                      std::greater<CostID>>
      open_list_;
};

#endif