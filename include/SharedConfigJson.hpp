#ifndef SHARED_CONFIG_JSON_HPP_
#define SHARED_CONFIG_JSON_HPP_

#include "config.h"
#include "json.hpp"

using json = nlohmann::json;

// Serialize
inline void to_json(json& j, const SharedConfig& cfg) {
    j = json{
        {"startRace", cfg.startRace},
        {"enableCarEngine", cfg.enableCarEngine},
        {"enableCarSteering", cfg.enableCarSteering},
        {"enableCameraThresholdCheck", cfg.enableCameraThresholdCheck},
        {"enableFinishLineDetection", cfg.enableFinishLineDetection},
        {"currentState", cfg.currentState},
        {"thresholdValue", cfg.thresholdValue},
        {"distanceErrorFromChassis", cfg.distanceErrorFromChassis},
        {"lineMinPixelCount", cfg.lineMinPixelCount},
        {"distanceSensorError", cfg.distanceSensorError},
        {"stoppingDistanceBoxFrontEnd", cfg.stoppingDistanceBoxFrontEnd},
        {"interpolatedPointsSetup", cfg.interpolatedPointsSetup},
        {"calibrateTopLinePerc", cfg.calibrateTopLinePerc},
        {"calibrateBottomLinePerc", cfg.calibrateBottomLinePerc},
        {"trackLaneWidthOffset", cfg.trackLaneWidthOffset},
        {"topImageCutPercentage", cfg.topImageCutPercentage},
        {"topCutOffPercentageCustomConnected", cfg.topCutOffPercentageCustomConnected},
        {"lineStartPointY", cfg.lineStartPointY},
        {"line90DegreeAngleRange", cfg.line90DegreeAngleRange},
        {"finishLineAngleRange", cfg.finishLineAngleRange},
        {"servoTurnAdjustmentCoefficient", cfg.servoTurnAdjustmentCoefficient},
        {"corneringSpeedCoefficient", cfg.corneringSpeedCoefficient},
        {"minSpeed", cfg.minSpeed},
        {"maxSpeed", cfg.maxSpeed},
        {"minSpeedAfterFinish", cfg.minSpeedAfterFinish},
        {"maxSpeedAfterFinish", cfg.maxSpeedAfterFinish},
        {"currentEdfFanSpeed", cfg.currentEdfFanSpeed},
        {"curvatureFactor", cfg.curvatureFactor},
        {"rdpEpsilon", cfg.rdpEpsilon},
        {"k_min", cfg.k_min},
        {"k_max", cfg.k_max},
        {"R_minInCm", cfg.R_minInCm},
        {"R_maxInCm", cfg.R_maxInCm},
        {"minLookAheadInCm", cfg.minLookAheadInCm},
        {"maxLookAheadInCm", cfg.maxLookAheadInCm},
        {"waitBeforeStartSeconds", cfg.waitBeforeStartSeconds},
        {"waitBeforeEdfStartSeconds", cfg.waitBeforeEdfStartSeconds},
        {"waitBeforeFinishDetectionSeconds", cfg.waitBeforeFinishDetectionSeconds}
    };
}

// Deserialize
inline void from_json(const json& j, SharedConfig& cfg) {
    j.at("startRace").get_to(cfg.startRace);
    j.at("enableCarEngine").get_to(cfg.enableCarEngine);
    j.at("enableCarSteering").get_to(cfg.enableCarSteering);
    j.at("enableCameraThresholdCheck").get_to(cfg.enableCameraThresholdCheck);
    j.at("enableFinishLineDetection").get_to(cfg.enableFinishLineDetection);
    j.at("currentState").get_to(cfg.currentState);
    j.at("thresholdValue").get_to(cfg.thresholdValue);
    j.at("distanceErrorFromChassis").get_to(cfg.distanceErrorFromChassis);
    j.at("lineMinPixelCount").get_to(cfg.lineMinPixelCount);
    j.at("distanceSensorError").get_to(cfg.distanceSensorError);
    j.at("stoppingDistanceBoxFrontEnd").get_to(cfg.stoppingDistanceBoxFrontEnd);
    j.at("interpolatedPointsSetup").get_to(cfg.interpolatedPointsSetup);
    j.at("calibrateTopLinePerc").get_to(cfg.calibrateTopLinePerc);
    j.at("calibrateBottomLinePerc").get_to(cfg.calibrateBottomLinePerc);
    j.at("trackLaneWidthOffset").get_to(cfg.trackLaneWidthOffset);
    j.at("topImageCutPercentage").get_to(cfg.topImageCutPercentage);
    j.at("topCutOffPercentageCustomConnected").get_to(cfg.topCutOffPercentageCustomConnected);
    j.at("lineStartPointY").get_to(cfg.lineStartPointY);
    j.at("line90DegreeAngleRange").get_to(cfg.line90DegreeAngleRange);
    j.at("finishLineAngleRange").get_to(cfg.finishLineAngleRange);
    j.at("servoTurnAdjustmentCoefficient").get_to(cfg.servoTurnAdjustmentCoefficient);
    j.at("corneringSpeedCoefficient").get_to(cfg.corneringSpeedCoefficient);
    j.at("minSpeed").get_to(cfg.minSpeed);
    j.at("maxSpeed").get_to(cfg.maxSpeed);
    j.at("minSpeedAfterFinish").get_to(cfg.minSpeedAfterFinish);
    j.at("maxSpeedAfterFinish").get_to(cfg.maxSpeedAfterFinish);
    j.at("currentEdfFanSpeed").get_to(cfg.currentEdfFanSpeed);
    j.at("curvatureFactor").get_to(cfg.curvatureFactor);
    j.at("rdpEpsilon").get_to(cfg.rdpEpsilon);
    j.at("k_min").get_to(cfg.k_min);
    j.at("k_max").get_to(cfg.k_max);
    j.at("R_minInCm").get_to(cfg.R_minInCm);
    j.at("R_maxInCm").get_to(cfg.R_maxInCm);
    j.at("minLookAheadInCm").get_to(cfg.minLookAheadInCm);
    j.at("maxLookAheadInCm").get_to(cfg.maxLookAheadInCm);
    j.at("waitBeforeStartSeconds").get_to(cfg.waitBeforeStartSeconds);
    j.at("waitBeforeEdfStartSeconds").get_to(cfg.waitBeforeEdfStartSeconds);
    j.at("waitBeforeFinishDetectionSeconds").get_to(cfg.waitBeforeFinishDetectionSeconds);
}
#endif