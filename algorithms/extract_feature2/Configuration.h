#pragma once
#include "SeamTypes.h"
#include <fw/Algorithm.h>
inline weld::DetectorConfig configuration(const fw::ParameterSet& p) {
 weld::DetectorConfig c;
 c.hullSurfaceVoxelSize=float(p["hullSurfaceVoxelSize"].toDouble(1.0));
 c.hullSurfaceCurvature=float(p["hullSurfaceCurvature"].toDouble(0.035));
 c.hullSurfaceDistance=float(p["hullSurfaceDistance"].toDouble(0.6));
 c.hullPreservationMargin=float(p["hullPreservationMargin"].toDouble(3.0));
 c.longitudinalAxis=weld::Axis(QStringLiteral("xyz").indexOf(p["longitudinalAxis"].toString()));
 const auto interior=p["interiorOpeningDirection"].toString();
 c.interiorOpeningMode=interior=="positive_v"?weld::InteriorOpeningMode::PositiveV:
     interior=="negative_v"?weld::InteriorOpeningMode::NegativeV:
     weld::InteriorOpeningMode::Auto;
 c.useHullEnvelopeCrop=p["useHullEnvelopeCrop"].toBool();
 c.hullEnvelopeGridSize=static_cast<float>(p["hullEnvelopeGridSize"].toDouble());
 c.hullEnvelopeMinCellSupport=p["hullEnvelopeMinCellSupport"].toInt();
 c.hullEnvelopeMinThickness=static_cast<float>(p["hullEnvelopeMinThickness"].toDouble());
 c.hullEnvelopeMargin=static_cast<float>(p["hullEnvelopeMargin"].toDouble());
 c.hullEnvelopeSmoothingRadius=p["hullEnvelopeSmoothingRadius"].toInt();
 c.floorPatchSize=static_cast<float>(p["floorPatchSize"].toDouble());
 c.useLocalFloorConstraint=p["useLocalFloorConstraint"].toBool();
 c.floorPatchGuard=static_cast<float>(p["floorPatchGuard"].toDouble());
 c.floorPatchMaxRms=static_cast<float>(p["floorPatchMaxRms"].toDouble());
 c.floorPatchMaxCorrection=static_cast<float>(p["floorPatchMaxCorrection"].toDouble());
 c.voxelSize=static_cast<float>(p["voxelSize"].toDouble());
 c.outlierMeanK=p["outlierMeanK"].toInt();
 c.outlierStddev=static_cast<float>(p["outlierStddev"].toDouble());
 c.sliceStep=static_cast<float>(p["sliceStep"].toDouble());
 c.sliceThickness=static_cast<float>(p["sliceThickness"].toDouble());
 c.profileBinSize=static_cast<float>(p["profileBinSize"].toDouble());
 c.profileSamplingMode=p["profileSamplingMode"].toString()=="stable_grid"
     ?weld::ProfileSamplingMode::StableGrid
     :weld::ProfileSamplingMode::LegacyRolling;
 c.profileReferenceScale=static_cast<float>(p["profileReferenceScale"].toDouble());
 c.fitHalfWindow=p["fitHalfWindow"].toInt();
 c.fitGuardPoints=p["fitGuardPoints"].toInt();
 c.maxBottomSlope=static_cast<float>(p["maxBottomSlope"].toDouble());
 c.minSurfaceAngleDegrees=static_cast<float>(p["minSurfaceAngleDegrees"].toDouble());
 c.minLocalSurfaceAngleDegrees=static_cast<float>(p["minLocalSurfaceAngleDegrees"].toDouble());
 c.localAngleWindow=p["localAngleWindow"].toInt();
 c.maxLocalFitNormalChangeDegrees=static_cast<float>(p["maxLocalFitNormalChangeDegrees"].toDouble());
 c.maxLocalBottomAngleDifferenceDegrees=static_cast<float>(p["maxLocalBottomAngleDifferenceDegrees"].toDouble());
 c.useRegionGrowingEdges=p["useRegionGrowingEdges"].toBool();
 c.useContourRegionGrowing=p["useContourRegionGrowing"].toBool();
 c.minContourCornerAngleDegrees=static_cast<float>(p["minContourCornerAngleDegrees"].toDouble());
 c.regionGrowingWindow=p["regionGrowingWindow"].toInt();
 c.regionGrowingSmoothnessDegrees=static_cast<float>(p["regionGrowingSmoothnessDegrees"].toDouble());
 c.regionGrowingMaxNormalDriftDegrees=static_cast<float>(p["regionGrowingMaxNormalDriftDegrees"].toDouble());
 c.regionGrowingDistance=static_cast<float>(p["regionGrowingDistance"].toDouble());
 c.regionGrowingMaxEdgeGap=static_cast<float>(p["regionGrowingMaxEdgeGap"].toDouble());
 c.surfaceIntersectionTolerance=static_cast<float>(p["surfaceIntersectionTolerance"].toDouble());
 c.minSurfaceClearance=static_cast<float>(p["minSurfaceClearance"].toDouble());
 c.surfaceRmsScale=static_cast<float>(p["surfaceRmsScale"].toDouble());
 c.maxCandidatesPerSlice=p["maxCandidatesPerSlice"].toInt();
 c.minCandidateSpacing=static_cast<float>(p["minCandidateSpacing"].toDouble());
 c.maxBeadWidth=static_cast<float>(p["maxBeadWidth"].toDouble());
 c.minSeamSeparation=static_cast<float>(p["minSeamSeparation"].toDouble());
 c.maxTransitionDistance=static_cast<float>(p["maxTransitionDistance"].toDouble());
 c.smoothnessWeight=static_cast<float>(p["smoothnessWeight"].toDouble());
 c.curvatureWeight=static_cast<float>(p["curvatureWeight"].toDouble());
 c.featureWeight=static_cast<float>(p["featureWeight"].toDouble());
 c.longitudinalDetrendMinSpan=static_cast<float>(p["longitudinalDetrendMinSpan"].toDouble());
 c.maxInterpolatedGapStations=p["maxInterpolatedGapStations"].toInt();
 c.curveRegularizationWeight=static_cast<float>(p["curveRegularizationWeight"].toDouble());
 c.postprocessMode=p["postprocessMode"].toString()=="robust_rts"
     ?weld::SeamPostprocessMode::RobustRts:weld::SeamPostprocessMode::Legacy;
 c.rtsProcessNoise=static_cast<float>(p["rtsProcessNoise"].toDouble());
 c.rtsMeasurementStddev=static_cast<float>(p["rtsMeasurementStddev"].toDouble());
 c.rtsInnovationGate=static_cast<float>(p["rtsInnovationGate"].toDouble());
 c.rtsMaxCorrection=static_cast<float>(p["rtsMaxCorrection"].toDouble());
 c.rtsGeometryTolerance=static_cast<float>(p["rtsGeometryTolerance"].toDouble());
 return c;
}
