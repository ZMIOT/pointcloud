#pragma once
#include "CorrugatedPanel.h"

namespace bwb {
// Uses the selected B branch only as a seed. All fits use original surface samples.
bool recoverSurfaceIntersection(const std::vector<Point> &points, const Config &config,
                                const Hooks &hooks, Result &result);
std::vector<ProjectedPoint> resampleIntersection(const std::vector<ProjectedPoint> &points,
                                                double spacing);
}
