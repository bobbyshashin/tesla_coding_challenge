#include "network.h"
#include "supercharger_graph.h"

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cout << "Error: requires initial and final supercharger names"
              << std::endl;
    return -1;
  }

  std::string initial_charger_name = argv[1];
  std::string goal_charger_name = argv[2];

  int discretization_factor = 81;
  int max_range = 320;
  int max_speed = 105;

  auto g = SuperchargerGraph::Make(discretization_factor, max_range, max_speed);
  g->generateGraph();
  g->search(initial_charger_name, goal_charger_name);

  return 0;
}