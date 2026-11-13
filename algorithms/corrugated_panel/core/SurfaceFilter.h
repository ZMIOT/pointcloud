#pragma once
#include "CorrugatedPanel.h"

namespace bwb {
// Establishes finite local sheet support before slicing or recovering a curve.
// Indices always refer to the unchanged input vector; measured XYZ is not moved.
bool filterPanelSurfaces(const std::vector<Point> &points, const Config &config,
                         const Hooks &hooks, Result &result);
}
