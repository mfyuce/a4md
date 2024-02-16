
#include "artery/application/misbehavior/detection/MisbehaviorDetectionServiceCpm.h"
#include "artery/application/misbehavior/util/HelperFunctions.h"
#include "artery/application/misbehavior/fusion/ThresholdFusion.h"
#include "artery/envmod/LocalEnvironmentModel.h"
#include "artery/traci/VehicleController.h"
#include <omnetpp/cmessage.h>
#include <omnetpp/cpacket.h>
#include <vanetza/asn1/cpm.hpp>
#include "artery/application/CpmService.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/misbehavior/util/MisbehaviorTypes.h"
#include <inet/common/ModuleAccess.h>
#include "artery/traci/Cast.h"
#include "artery/application/misbehavior/MisbehaviorCpmService.h"
#include "artery/application/misbehavior/report/MisbehaviorReportObject.h"
#include <bitset>
#include <boost/units/systems/cgs.hpp>
#include <boost/units/make_scaled_unit.hpp>

namespace artery {
    using namespace omnetpp;

    Define_Module(MisbehaviorDetectionServiceCpm);

    namespace {
        auto degree_per_second = vanetza::units::degree / vanetza::units::si::second;

        template<typename T, typename U>
        long round(const boost::units::quantity<T> &q, const U &u) {
            boost::units::quantity<U> v{q};
            return std::round(v.value());
        }
    }

    static const simsignal_t scSignalCpmReceived = cComponent::registerSignal("CpmReceived");
    static const simsignal_t scSignalCpmSent = cComponent::registerSignal("CpmSent");
    static const simsignal_t scSignalMisbehaviorAuthorityNewReport = cComponent::registerSignal(
            "newMisbehaviorReport");

    bool MisbehaviorDetectionServiceCpm::staticInitializationComplete = false;
    std::shared_ptr<const traci::API> MisbehaviorDetectionServiceCpm::mTraciAPI;
    GlobalEnvironmentModel *MisbehaviorDetectionServiceCpm::mGlobalEnvironmentModel;
    traci::Boundary MisbehaviorDetectionServiceCpm::mSimulationBoundary;


    MisbehaviorDetectionServiceCpm::~MisbehaviorDetectionServiceCpm() {
        while (!activePoIs.empty()) {
            mTraciAPI->poi.remove(activePoIs.front());
            activePoIs.pop_front();
        }
        detectedSenders.clear();
    }

    void MisbehaviorDetectionServiceCpm::setCheckableDetectionLevels() {
        mCheckableDetectionLevels[detectionLevels::Level1] = true;
        mCheckableDetectionLevels[detectionLevels::Level2] = true;
        mCheckableDetectionLevels[detectionLevels::Level3] = true;
        mCheckableDetectionLevels[detectionLevels::Level4] = true;

        bool detectLowerThanHighest = F2MDParameters::detectionParameters.detectLevelsLowerThanHighest;
        if (uniform(0, 1) > F2MDParameters::detectionParameters.detectLevel4Probability) {
            mCheckableDetectionLevels[detectionLevels::Level4] = false;
        } else if (detectLowerThanHighest) {
            return;
        }
        if (uniform(0, 1) > F2MDParameters::detectionParameters.detectLevel3Probability) {
            mCheckableDetectionLevels[detectionLevels::Level3] = false;
        } else if (detectLowerThanHighest) {
            return;
        }
        if (uniform(0, 1) > F2MDParameters::detectionParameters.detectLevel2Probability) {
            mCheckableDetectionLevels[detectionLevels::Level2] = false;
        } else if (detectLowerThanHighest) {
            return;
        }
        if (uniform(0, 1) > F2MDParameters::detectionParameters.detectLevel1Probability) {
            mCheckableDetectionLevels[detectionLevels::Level1] = false;
        }
    }

    void MisbehaviorDetectionServiceCpm::initialize() {
        ItsG5BaseService::initialize();
        subscribe(scSignalCpmReceived);
        subscribe(scSignalCpmSent);
        mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
        mVehicleController = &getFacilities().get_const<traci::VehicleController>();
        mLocalEnvironmentModel = getFacilities().get_mutable_ptr<LocalEnvironmentModel>();
        mTimer = &getFacilities().get_const<Timer>();

        if (!staticInitializationComplete) {
            staticInitializationComplete = true;
            mGlobalEnvironmentModel = mLocalEnvironmentModel->getGlobalEnvMod();
            mTraciAPI = getFacilities().get_const<traci::VehicleController>().getTraCI();
            mSimulationBoundary = traci::Boundary{mTraciAPI->simulation.getNetBoundary()};
            initializeParameters();
        }
        setCheckableDetectionLevels();
        mFusionApplication = new ThresholdFusion(F2MDParameters::detectionParameters.misbehaviorThreshold);
    }

    void MisbehaviorDetectionServiceCpm::initializeParameters() {

        F2MDParameters::detectionParameters.checkType = par("checkType");
        F2MDParameters::detectionParameters.misbehaviorThreshold = par("misbehaviorThreshold");
        F2MDParameters::detectionParameters.detectedSenderCpmArrayMaxSize = par("detectedSenderCpmArrayMaxSize");

        F2MDParameters::detectionParameters.detectLevel4Probability = par("detectLevel4Probability");
        F2MDParameters::detectionParameters.detectLevel3Probability = par("detectLevel3Probability");
        F2MDParameters::detectionParameters.detectLevel2Probability = par("detectLevel2Probability");
        F2MDParameters::detectionParameters.detectLevel1Probability = par("detectLevel1Probability");
        F2MDParameters::detectionParameters.detectLevelsLowerThanHighest = par("detectLevelsLowerThanHighest");

        F2MDParameters::detectionParameters.maxPlausibleSpeed = par("maxPlausibleSpeed");
        F2MDParameters::detectionParameters.maxPlausibleAcceleration = par("maxPlausibleAcceleration");
        F2MDParameters::detectionParameters.maxPlausibleDeceleration = par("maxPlausibleDeceleration");
        F2MDParameters::detectionParameters.maxPlausibleRange = par("maxPlausibleRange");

        F2MDParameters::detectionParameters.maxProximityRangeL = par("maxProximityRangeL");
        F2MDParameters::detectionParameters.maxProximityRangeW = par("maxProximityRangeW");
        F2MDParameters::detectionParameters.maxProximityDistance = par("maxProximityDistance");
        F2MDParameters::detectionParameters.maxTimeDelta = par("maxTimeDelta");
        F2MDParameters::detectionParameters.maxMgtRng = par("maxMgtRng");
        F2MDParameters::detectionParameters.maxMgtRngDown = par("maxMgtRngDown");
        F2MDParameters::detectionParameters.maxMgtRngUp = par("maxMgtRngUp");
        F2MDParameters::detectionParameters.maxSuddenAppearanceRange = par("maxSuddenAppearanceRange");
        F2MDParameters::detectionParameters.maxSuddenAppearanceTime = par("maxSuddenAppearanceTime");
        F2MDParameters::detectionParameters.maxCpmFrequency = par("maxCpmFrequency");
        F2MDParameters::detectionParameters.maxOffroadSpeed = par("maxOffroadSpeed");
        F2MDParameters::detectionParameters.maxDistanceFromRoad = par("maxDistanceFromRoad");
        F2MDParameters::detectionParameters.positionHeadingTime = par("positionHeadingTime");
        F2MDParameters::detectionParameters.maxHeadingChange = par("maxHeadingChange");
        F2MDParameters::detectionParameters.maxIntersectionDeltaTime = par("maxIntersectionDeltaTime");

        F2MDParameters::detectionParameters.maxKalmanTime = par("maxKalmanTime");
        F2MDParameters::detectionParameters.kalmanMinPosRange = par("kalmanMinPosRange");
        F2MDParameters::detectionParameters.kalmanMinSpeedRange = par("kalmanMinSpeedRange");
        F2MDParameters::detectionParameters.kalmanMinHeadingRange = par("kalmanMinHeadingRange");
        F2MDParameters::detectionParameters.kalmanPosRange = par("kalmanPosRange");
        F2MDParameters::detectionParameters.kalmanSpeedRange = par("kalmanSpeedRange");

        F2MDParameters::reportParameters.evidenceContainerMaxCpmCount = par("evidenceContainerMaxCpmCount");
        F2MDParameters::reportParameters.omittedReportsCount = par("omittedReportsCount");
        F2MDParameters::reportParameters.omittedReportsCountPerErrorCode = par("omittedReportsCountPerErrorCode");
        F2MDParameters::reportParameters.broadcastReport = par("broadcastReport");

    }

    void MisbehaviorDetectionServiceCpm::indicate(const vanetza::btp::DataIndication &ind, cPacket *packet,
                                               const NetworkInterface &net) {
        Enter_Method("indicate");
        delete (packet);
    }


    void MisbehaviorDetectionServiceCpm::receiveSignal(cComponent *source, simsignal_t signal, cObject *c_obj, cObject *) {
        Enter_Method("receiveSignal");
        if (signal == scSignalCpmReceived) {
            auto *ca = dynamic_cast<CpmObject *>(c_obj);
            std::shared_ptr<vanetza::asn1::Cpm> message = std::make_shared<vanetza::asn1::Cpm>(ca->asn1());
            detectMisbehavior(message);
        } else if (signal == scSignalCpmSent) {
            auto *ca = dynamic_cast<CpmObject *>(c_obj);
            mLastSentCpm = std::make_shared<vanetza::asn1::Cpm>(ca->asn1());
        }
    }

    void MisbehaviorDetectionServiceCpm::detectMisbehavior(const shared_ptr<vanetza::asn1::Cpm> &message) {
        uint32_t senderStationId = (*message)->header.stationID;
        std::shared_ptr<CheckResult> checkResult = checkCpm(message);
        std::vector<std::bitset<16>> detectionLevelErrorCodes = mFusionApplication->checkForReport(*checkResult);
        DetectedSender &detectedSender = *detectedSenders[senderStationId];
        std::string relatedReportId = detectedSender.getPreviousReportId();
        std::bitset<16> reportedErrorCodes = 0;

        for (detectionLevels::DetectionLevels detectionLevel: detectionLevels::DetectionLevelVector) {
            std::bitset<16> errorCode = detectionLevelErrorCodes[(int) detectionLevel];
            if (errorCode.any() && detectedSender.checkOmittedReportsLimit(errorCode)) {
                std::string reportId(
                        generateReportId(senderStationId, mVehicleDataProvider->getStationId(), getRNG(0)));
                Report report(reportId, message, countTaiMilliseconds(mTimer->getTimeFor(simTime())));
                report.setSemanticDetection(detectionLevel, errorCode);
                SenderInfoContainer_t *senderInfoContainer = (vanetza::asn1::allocate<SenderInfoContainer_t>());
                switch (detectionLevel) {
                    case detectionLevels::Level1:
                        break;
                    case detectionLevels::Level2: {
                        int cpmCount = F2MDParameters::reportParameters.evidenceContainerMaxCpmCount >
                                       F2MDParameters::reportParameters.omittedReportsCount
                                       ? F2MDParameters::reportParameters.omittedReportsCount
                                       : F2MDParameters::reportParameters.evidenceContainerMaxCpmCount;
                        while (detectedSender.cpmList.size() - 1 > cpmCount) {
                            detectedSender.cpmList.pop_front();
                        }
                        report.setReportedMessages(detectedSender.getCpmVector(), cpmCount);
                        break;
                    }
                    case detectionLevels::Level3:
                        if (checkResult->intersection < F2MDParameters::detectionParameters.misbehaviorThreshold) {
                            report.evidence.neighbourCpmMessages = getOverlappingCpms(detectedSender);
                        }
                        break;
                    case detectionLevels::Level4: {
                        report.fillSenderInfoContainer(mVehicleDataProvider, mVehicleController);
                        break;
                    }
                    default:
                        break;
                }
                if (!relatedReportId.empty()) {
                    report.setRelatedReport(relatedReportId, F2MDParameters::reportParameters.omittedReportsCount);
                }
                reportedErrorCodes |= errorCode;
                vanetza::asn1::MisbehaviorReport misbehaviorReport = report.encode();
                MisbehaviorReportObject obj(std::move(misbehaviorReport));
                emit(scSignalMisbehaviorAuthorityNewReport, &obj);
                vanetza::asn1::free(asn_DEF_SenderInfoContainer, senderInfoContainer);
                relatedReportId = reportId;
                if (detectedSender.getPreviousReportId().empty()) {
                    detectedSender.setReportId(relatedReportId);
                }
            }
        }
        if (reportedErrorCodes.any()) {
            detectedSender.resetOmittedReports(reportedErrorCodes);
        }
        detectedSender.incrementOmittedReports(detectionLevelErrorCodes, reportedErrorCodes);
    }


    std::shared_ptr<CheckResult> MisbehaviorDetectionServiceCpm::checkCpm(const shared_ptr<vanetza::asn1::Cpm> &message) {

        uint32_t senderStationId = (*message)->header.stationID;
        std::vector<Position> mVehicleOutline = getVehicleOutline(mVehicleDataProvider, mVehicleController);

        std::vector<std::shared_ptr<vanetza::asn1::Cpm>> surroundingCpmObjects = getSurroundingCpms(
                senderStationId);
        if (detectedSenders.find(senderStationId) == detectedSenders.end()) {
            detectedSenders[senderStationId] = std::make_shared<DetectedSender>(mTraciAPI, mGlobalEnvironmentModel,
                                                                                &F2MDParameters::detectionParameters,
                                                                                mTimer, message,
                                                                                mCheckableDetectionLevels);
        }
        std::shared_ptr<CheckResult> result = detectedSenders[senderStationId]->addAndCheckCpm(message,
                                                                                               mVehicleDataProvider,
                                                                                               mVehicleOutline,
                                                                                               surroundingCpmObjects);
        return result;
    }

    std::vector<std::shared_ptr<vanetza::asn1::Cpm>>
    MisbehaviorDetectionServiceCpm::getSurroundingCpms(StationID_t senderStationId) {
        std::vector<std::shared_ptr<vanetza::asn1::Cpm>> surroundingCpmObjects;
        for (const auto &it: detectedSenders) {
            auto detectedSender = it.second;
            if (detectedSender->getStationId() != senderStationId) {
                std::shared_ptr<vanetza::asn1::Cpm> latestCpm = detectedSender->latestCpm;
                uint16_t oldTime = (*latestCpm)->cpm.generationDeltaTime;
                uint16_t currentTime = countTaiMilliseconds(mTimer->getCurrentTime());
                if ((uint16_t) (currentTime - oldTime) <
                    (long) (F2MDParameters::detectionParameters.maxCpmFrequency * 1000)) {
                    surroundingCpmObjects.emplace_back(latestCpm);
                }
            }
        }
        return surroundingCpmObjects;
    }

    std::vector<std::shared_ptr<vanetza::asn1::Cpm>>
    MisbehaviorDetectionServiceCpm::getOverlappingCpms(const DetectedSender &detectedSender) {
        std::vector<Position> senderOutline = getVehicleOutline(*detectedSender.latestCpm,
                                                                mSimulationBoundary,
                                                                mTraciAPI);
        std::vector<std::shared_ptr<vanetza::asn1::Cpm>> overlappingCpms;
        if (boost::geometry::intersects(senderOutline, getVehicleOutline(mVehicleDataProvider,
                                                                         mVehicleController))) {
            overlappingCpms.emplace_back(mLastSentCpm);
        }
        for (const auto &cpm: getSurroundingCpms(detectedSender.getStationId())) {
            std::vector<Position> outline = getVehicleOutline((*cpm.get()), mSimulationBoundary,
                                                              mTraciAPI);
            if (boost::geometry::intersects(senderOutline, outline)) {
                overlappingCpms.emplace_back(cpm);
            }
        }
        return overlappingCpms;
    }

    void MisbehaviorDetectionServiceCpm::visualizeCpmPosition(vanetza::asn1::Cpm cpm, const libsumo::TraCIColor &color,
                                                           const std::string &idPrefix) {
        static int counter = 0;
        traci::TraCIGeoPosition traciGeoPosition = {
                (double) cpm->cpm.cpmParameters.managementContainer.referencePosition.longitude / 10000000.0,
                (double) cpm->cpm.cpmParameters.managementContainer.referencePosition.latitude / 10000000.0};
        traci::TraCIPosition traciPosition = mVehicleController->getTraCI()->convert2D(traciGeoPosition);
        std::string poiId = {
                std::to_string(cpm->header.stationID) + idPrefix + "_CPM_" + std::to_string(cpm->header.messageID) +
                "-" + std::to_string(cpm->cpm.generationDeltaTime) + "-" + std::to_string(counter++)};
        mTraciAPI->poi.add(poiId, traciPosition.x, traciPosition.y, color,
                           poiId, 5, "", 0,
                           0, 0);
        activePoIs.push_back(poiId);
        if (activePoIs.size() > F2MDParameters::miscParameters.CpmLocationVisualizerMaxLength) {
            mTraciAPI->poi.remove(activePoIs.front());
            activePoIs.pop_front();
        }
        int alphaStep = 185 / F2MDParameters::miscParameters.CpmLocationVisualizerMaxLength;
        int currentAlpha = 80;
        for (const auto &poi: activePoIs) {
            mTraciAPI->poi.setColor(poi, color);
            currentAlpha += alphaStep;
        }
    }
} // namespace artery
