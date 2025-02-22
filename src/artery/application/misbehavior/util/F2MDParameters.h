//
// Created by bastian on 10.06.21.
//

#ifndef ARTERY_F2MDPARAMETERS_H
#define ARTERY_F2MDPARAMETERS_H


#include <omnetpp.h>

using namespace omnetpp;
namespace artery {


    struct AttackParameters {
        //Constant Position Attack
        double AttackConstantPositionMinLatitude;
        double AttackConstantPositionMaxLatitude;
        double AttackConstantPositionMinLongitude;
        double AttackConstantPositionMaxLongitude;

        //Constant Position Offset Attack
        double AttackConstantPositionOffsetMaxLatitudeOffset;
        double AttackConstantPositionOffsetMaxLongitudeOffset;

        // Random Position Attack
        double AttackRandomPositionMinLatitude;
        double AttackRandomPositionMaxLatitude;
        double AttackRandomPositionMinLongitude;
        double AttackRandomPositionMaxLongitude;

        // Random Position Offset Attack
        double AttackRandomPositionOffsetMaxLatitudeOffset;
        double AttackRandomPositionOffsetMaxLongitudeOffset;

        // Constant Speed Attack
        double AttackConstantSpeedMin;
        double AttackConstantSpeedMax;

        //Constant Speed Offset Attack
        double AttackConstantSpeedOffsetMax;

        // Random Speed Attack
        double AttackRandomSpeedMin;
        double AttackRandomSpeedMax;

        // Random Speed Offset Attack
        double AttackRandomSpeedOffsetMax;

        // Eventual Stop Attack
        double AttackEventualStopProbabilityThreshold;

        // Disruptive Attack
        int AttackDisruptiveBufferSize;
        int AttackDisruptiveMinimumReceived;

        // Denial of Service Attack
        int AttackDoSInterval;
        bool AttackDoSIgnoreDCC;

        // Stale Messages Attack
        int AttackStaleDelayCount;

        // Grid Sybil Attack
        int AttackGridSybilVehicleCount;
        int AttackGridSybilVehicleCountVariation;
        bool AttackGridSybilSelfSybil;
        double AttackGridSybilDistanceX;
        double AttackGridSybilDistanceY;
        double AttackGridSybilDistanceVariation;
        double AttackGridSybilMaxDistanceFromRoad;

        // Fake Report Attack
        double AttackFakeReportInterval;
    };

    struct DetectionParameters {

        int checkType;
        double misbehaviorThreshold;
        int detectedSenderCamArrayMaxSize;
        int detectedSenderCpmArrayMaxSize;

        double detectLevel4Probability;
        double detectLevel3Probability;
        double detectLevel2Probability;
        double detectLevel1Probability;
        bool detectLevelsLowerThanHighest;

        double maxPlausibleSpeed;
        double maxPlausibleAcceleration;
        double maxPlausibleDeceleration;
        double maxPlausibleRange;

        double maxProximityRangeL;
        double maxProximityRangeW;
        double maxProximityDistance;
        double maxTimeDelta;
        double maxMgtRng;
        double maxMgtRngDown;
        double maxMgtRngUp;
        double maxSuddenAppearanceRange;
        double maxSuddenAppearanceTime;
        double maxCamFrequency;
        double maxCpmFrequency;
        double maxOffroadSpeed;
        double maxDistanceFromRoad;
        double positionHeadingTime;
        double maxHeadingChange;
        double maxIntersectionDeltaTime;

        double maxKalmanTime;
        double kalmanMinPosRange;
        double kalmanMinSpeedRange;
        double kalmanMinHeadingRange;
        double kalmanPosRange;
        double kalmanSpeedRange;
    };

    struct ReportParameters {
        int evidenceContainerMaxCamCount;
        int evidenceContainerMaxCpmCount;
        int omittedReportsCount;
        bool omittedReportsCountPerErrorCode;
        bool broadcastReport;
    };

    struct MisbehaviorAuthorityParameters {
        double maxReportAge;
        int reportScoreThreshold;
        double misbehaviorThreshold;
        int checkType;
        bool enableVectorRecording;

        double reportCleanupInterval;
        double reportCleanupAge;
        double reportCamRetentionTime;
        double reportCamRetentionCleanupInterval;
        double reportCpmRetentionTime;
        double reportCpmRetentionCleanupInterval;

        bool considerReportAge;
        bool considerReportValidity;
        bool considerReporterScore;
        bool considerEvidenceScore;
        int evidenceScoreMaxCamCount;
        int evidenceScoreMaxCpmCount;

        bool enableWebGui;
        std::string webGuiDataUrl;
        double guiJsonDataUpdateInterval;
        double updateTimeStep;
        int displaySteps;
        int recentReportedCount;
    };

    struct MiscParameters {
        // CAM Location Visualizer (PoI)
        bool CamLocationVisualizer;
        int CamLocationVisualizerMaxLength;
        bool CpmLocationVisualizer;
        int CpmLocationVisualizerMaxLength;
    };

    class F2MDParameters {
    public:
        F2MDParameters();

        static AttackParameters attackParameters;
        static DetectionParameters detectionParameters;
        static ReportParameters reportParameters;
        static MisbehaviorAuthorityParameters misbehaviorAuthorityParameters;
        static MiscParameters miscParameters;

    };
} // namespace artery

#endif //ARTERY_F2MDPARAMETERS_H
