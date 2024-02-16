//
// Created by bastian on 15.07.21.
//

#ifndef ARTERY_BASECHECKS_H
#define ARTERY_BASECHECKS_H

#include <vanetza/asn1/cam.hpp>
#include <vanetza/asn1/cpm.hpp>
#include <omnetpp.h>
#include <artery/application/misbehavior/checks/CheckResult.h>
#include <artery/application/misbehavior/checks/kalman/Kalman_SVI.h>
#include <artery/application/misbehavior/checks/kalman/Kalman_SC.h>
#include <artery/application/misbehavior/checks/kalman/Kalman_SI.h>
#include <artery/application/misbehavior/checks/BaseChecks.h>
#include <artery/application/misbehavior/util/DetectionLevels.h>
#include "artery/application/Timer.h"
#include <artery/envmod/LocalEnvironmentModel.h>
#include <artery/envmod/GlobalEnvironmentModel.h>
#include <vanetza/asn1/md/SenderInfoContainer.h>
#include <artery/application/VehicleDataProvider.h>
#include <artery/application/misbehavior/util/F2MDParameters.h>
#include "artery/traci/VehicleController.h"
#include "artery/utility/Geometry.h"
#include "artery/application/misbehavior/fusion/ThresholdFusion.h"


namespace artery {

    class BaseChecks {
    public:
        BaseChecks(std::shared_ptr<const traci::API> traciAPI, GlobalEnvironmentModel *globalEnvironmentModel,
                   DetectionParameters *detectionParameters, const Timer *timer,
                   std::map<detectionLevels::DetectionLevels, bool> checkableDetectionLevels,
                   const std::shared_ptr<vanetza::asn1::Cam> &message);
        BaseChecks(std::shared_ptr<const traci::API> traciAPI, GlobalEnvironmentModel *globalEnvironmentModel,
                   DetectionParameters *detectionParameters, const Timer *timer,
                   std::map<detectionLevels::DetectionLevels, bool> checkableDetectionLevels,
                   const std::shared_ptr<vanetza::asn1::Cpm> &message);
        BaseChecks(std::shared_ptr<const traci::API> traciAPI, GlobalEnvironmentModel *globalEnvironmentModel,
                   DetectionParameters *detectionParameters, const Timer *timer);

        virtual ~BaseChecks() {};

        void initializeKalmanFilters(const std::shared_ptr<vanetza::asn1::Cam> &message);
        void initializeKalmanFilters(const std::shared_ptr<vanetza::asn1::Cpm> &message);

        virtual std::shared_ptr<CheckResult> checkCAM(const VehicleDataProvider *receiverVDP,
                                                      const std::vector<Position> &receiverVehicleOutline,
                                                      const std::shared_ptr<vanetza::asn1::Cam> &currentCam,
                                                      const std::shared_ptr<vanetza::asn1::Cam> &lastCam,
                                                      const std::vector<std::shared_ptr<vanetza::asn1::Cam>> &surroundingCamObjects) = 0;

        virtual std::shared_ptr<CheckResult> checkCPM(const VehicleDataProvider *receiverVDP,
                                                      const std::vector<Position> &receiverVehicleOutline,
                                                      const std::shared_ptr<vanetza::asn1::Cpm> &currentCpm,
                                                      const std::shared_ptr<vanetza::asn1::Cpm> &lastCpm,
                                                      const std::vector<std::shared_ptr<vanetza::asn1::Cpm>> &surroundingCpmObjects) = 0;

        virtual std::bitset<16> checkSemanticLevel1Report(const std::shared_ptr<vanetza::asn1::Cam> &currentCam) = 0;
        virtual std::bitset<16> checkSemanticLevel1Report(const std::shared_ptr<vanetza::asn1::Cpm> &currentCpm) = 0;

        virtual std::bitset<16> checkSemanticLevel2Report(const std::shared_ptr<vanetza::asn1::Cam> &currentCam,
                                                          const std::shared_ptr<vanetza::asn1::Cam> &lastCam) = 0;
        virtual std::bitset<16> checkSemanticLevel2Report(const std::shared_ptr<vanetza::asn1::Cpm> &currentCpm,
                                                          const std::shared_ptr<vanetza::asn1::Cpm> &lastCpm) = 0;

        virtual std::bitset<16> checkSemanticLevel3Report(const std::shared_ptr<vanetza::asn1::Cam> &currentCam,
                                                          const std::vector<std::shared_ptr<vanetza::asn1::Cam>> &neighbourCams) = 0;
        virtual std::bitset<16> checkSemanticLevel3Report(const std::shared_ptr<vanetza::asn1::Cpm> &currentCpm,
                                                          const std::vector<std::shared_ptr<vanetza::asn1::Cpm>> &neighbourCpms) = 0;

        virtual std::bitset<16>
        checkSemanticLevel4Report(const std::shared_ptr<vanetza::asn1::Cam> &currentCam,
                                  const std::vector<std::shared_ptr<vanetza::asn1::Cam>> &neighbourCams,
                                  const SenderInfoContainer_t &senderInfo) = 0;
        virtual std::bitset<16>
        checkSemanticLevel4Report(const std::shared_ptr<vanetza::asn1::Cpm> &currentCpm,
                                  const std::vector<std::shared_ptr<vanetza::asn1::Cpm>> &neighbourCpms,
                                  const SenderInfoContainer_t &senderInfo) = 0;


    protected:
        static bool staticInitializationComplete;
        static GlobalEnvironmentModel *mGlobalEnvironmentModel;
        static traci::Boundary mSimulationBoundary;
        static std::shared_ptr<const traci::API> mTraciAPI;
        static const Timer *mTimer;
        DetectionParameters *detectionParameters;
        ThresholdFusion *mThresholdFusion;
        std::map<detectionLevels::DetectionLevels, bool> mCheckableDetectionLevels;

        Kalman_SVI *kalmanSVI = nullptr;
        Kalman_SC *kalmanSVSI = nullptr;
        Kalman_SI *kalmanSI = nullptr;
        Kalman_SI *kalmanVI = nullptr;

        Position mLastCamPosition;
        PosConfidenceEllipse_t mLastCamPositionConfidence;
        std::vector<Position> mLastCamPositionEllipse;
        double mLastCamEllipseRadius;
        double mLastCamSpeed;
        double mLastCamSpeedConfidence;
        Position mLastCamSpeedVector;
        bool mCheckingFirstCam = true;


        Position mLastCpmPosition;
        PosConfidenceEllipse_t mLastCpmPositionConfidence;
        std::vector<Position> mLastCpmPositionEllipse;
        double mLastCpmEllipseRadius;
        double mLastCpmSpeed;
        double mLastCpmSpeedConfidence;
        Position mLastCpmSpeedVector;
        bool mCheckingFirstCpm = true;


        double FrequencyCheck(const double &deltaTime) const;
        double FrequencyCheckCpm(const double &deltaTime) const;
        void KalmanChecks(const Position &currentCamPosition,
                          const PosConfidenceEllipse_t &currentCamPositionConfidence,
                          const double &currentCamSpeed,
                          const Position &currentCamSpeedVector, const double &currentCamSpeedConfidence,
                          const double &currentCamAcceleration, const Position &currentCamAccelerationVector,
                          const double &currentCamHeading, const Position &lastCamPosition,
                          const Position &lastCamSpeedVector, const double &camDeltaTime,
                          CheckResult &result);
        void KalmanChecksCpm(const Position &currentCpmPosition,
                          const PosConfidenceEllipse_t &currentCpmPositionConfidence,
                          const double &currentCpmSpeed,
                          const Position &currentCpmSpeedVector, const double &currentCpmSpeedConfidence,
                          const double &currentCpmAcceleration, const Position &currentCpmAccelerationVector,
                          const double &currentCpmHeading, const Position &lastCpmPosition,
                          const Position &lastCpmSpeedVector, const double &camDeltaTime,
                          CheckResult &result);


    private:

        void KalmanPositionSpeedConsistencyCheck(const Position &currentPosition,
                                                 const PosConfidenceEllipse_t &currentPositionConfidence,
                                                 const Position &currentSpeed, const Position &currentAcceleration,
                                                 const double &currentSpeedConfidence,
                                                 const double &deltaTime, double *returnValue) const;

        void KalmanPositionSpeedScalarConsistencyCheck(const Position &currentPosition, const Position &oldPosition,
                                                       const PosConfidenceEllipse_t &currentPositionConfidence,
                                                       const double &currentSpeed, const double &currentAcceleration,
                                                       const double &currentSpeedConfidence, const double &deltaTime,
                                                       double *returnValue);

        double KalmanPositionConsistencyCheck(const Position &currentPosition, const Position &oldPosition,
                                              const PosConfidenceEllipse_t &currentPositionConfidence,
                                              const double &deltaTime);

        double KalmanPositionAccConsistencyCheck(const Position &currentPosition, const Position &currentSpeed,
                                                 const PosConfidenceEllipse_t &currentPositionConfidence,
                                                 const double &deltaTime);

        double KalmanSpeedConsistencyCheck(const Position &currentSpeed, const Position &oldSpeed,
                                           const double &currentSpeedConfidence, const Position &currentAcceleration,
                                           const double &deltaTime);

    };

} // namespace artery

#endif //ARTERY_BASECHECKS_H
