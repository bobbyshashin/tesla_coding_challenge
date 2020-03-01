#include "graph_generator.h"
#include "math.h"

double GraphGenerator::calcDistance(double lat1, double lon1, double lat2,
                                    double lon2) const {
  return earth_radius_ * acos(sin(deg2Rad(lat1)) * sin(deg2Rad(lat2)) +
                              cos(deg2Rad(lat1)) * cos(deg2Rad(lat2)) *
                                  cos(deg2Rad(fabs(lon1 - lon2))));
}