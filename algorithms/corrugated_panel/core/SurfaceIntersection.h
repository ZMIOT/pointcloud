#pragma once
#include "CorrugatedPanel.h"

namespace bwb {
// Uses B only as a seed. Fits use original measured coordinates admitted by surfaceMask.
// An empty mask is allowed for standalone intersection tests/callers (all finite input).
bool recoverSurfaceIntersection(const std::vector<Point> &points, const Config &config,
                                const Hooks &hooks, Result &result);
std::vector<ProjectedPoint> resampleIntersection(const std::vector<ProjectedPoint> &points,
                                                double spacing);
}
