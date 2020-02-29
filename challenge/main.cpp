#include "graph_generator.h"
#include "network.h"

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cout << "Error: requires initial and final supercharger names"
              << std::endl;
    return -1;
  }

  std::string initial_charger_name = argv[1];
  std::string goal_charger_name = argv[2];

  // std::cout << network.size() << std::endl;
  GraphGenerator* g = new GraphGenerator(network, 321, 320, 105);
  std::cout << "Done creating graph generator" << std::endl;
  g->generateGraph();
  std::cout << "Done generating graph" << std::endl;
  g->search(initial_charger_name, goal_charger_name);
  std::cout << "Done searching graph" << std::endl;
  return 0;
}