//
// Created by bastian on 05.07.21.
//

#include "MisbehaviorAuthority.h"
#include "traci/Core.h"
#include "artery/traci/Cast.h"
#include "artery/application/misbehavior/report/MisbehaviorReportObject.h"
#include "artery/application/misbehavior/util/DetectionLevels.h"
#include "artery/application/misbehavior/util/CheckTypes.h"
#include "artery/application/misbehavior/util/HelperFunctions.h"
#include "artery/application/misbehavior/fusion/ThresholdFusion.h"
#include <artery/application/misbehavior/checks/LegacyChecks.h>
#include <artery/application/misbehavior/checks/CatchChecks.h>
#include <bitset>
#include <chrono>
#include <memory>
#include <numeric>

//#include <rapidjson/stringbuffer.h>
//#include <rapidjson/writer.h>


namespace artery {

    using namespace omnetpp;

    Define_Module(MisbehaviorAuthority)

    MisbehaviorAuthority::MisbehaviorAuthority() {
        curl = curl_easy_init();
        traciInitSignal = cComponent::registerSignal("traci.init");
        traciCloseSignal = cComponent::registerSignal("traci.close");
        maNewReport = cComponent::registerSignal("newMisbehaviorReport");
        maMisbehaviorAnnouncement = cComponent::registerSignal("misbehaviorAuthority.MisbehaviorAnnouncement");
    }

    MisbehaviorAuthority::~MisbehaviorAuthority() {
        curl_easy_cleanup(curl);
        this->clear();
        delete mBaseChecks;
    }

    void MisbehaviorAuthority::clear() {
    }

    void MisbehaviorAuthority::initialize() {
        cModule *traci = this->getParentModule()->getSubmodule("traci");
        traci->subscribe(traciInitSignal, this);
        traci->subscribe(traciCloseSignal, this);
        getSimulation()->getSystemModule()->subscribe(maNewReport, this);
        getSimulation()->getSystemModule()->subscribe(maMisbehaviorAnnouncement, this);

        mTimer.setTimebase(par("datetime"));
        cModule *globalEnvMod = this->getParentModule()->getSubmodule("environmentModel");
        if (globalEnvMod == nullptr) {
            throw cRuntimeError("globalEnvMod not found");
        }
        mGlobalEnvironmentModel = dynamic_cast<GlobalEnvironmentModel *>(globalEnvMod);
        mParameters = &F2MDParameters::misbehaviorAuthorityParameters;

        mParameters->maxReportAge = par("maxReportAge");
        mParameters->reportScoreThreshold = par("reportScoreThreshold");
        mParameters->checkType = par("checkType");

        mParameters->reportCleanupInterval = par("reportCleanupInterval");
        mParameters->reportCleanupAge = par("reportCleanupAge");
        mParameters->reportCamRetentionTime = par("reportCamRetentionTime");
        mParameters->reportCamRetentionCleanupInterval = par(
                "reportCamRetentionCleanupInterval");

        mParameters->reportCpmRetentionTime = par("reportCpmRetentionTime");
        mParameters->reportCpmRetentionCleanupInterval = par(
                "reportCpmRetentionCleanupInterval");

        mParameters->considerReporterScore = par("considerReporterScore");
        mParameters->considerEvidenceScore = par("considerEvidenceScore");
        mParameters->considerReportAge = par("considerReportAge");
        mParameters->considerReportValidity = par("considerReportValidity");
        mParameters->evidenceScoreMaxCamCount = par("evidenceScoreMaxCamCount");
        mParameters->evidenceScoreMaxCpmCount = par("evidenceScoreMaxCpmCount");


        mParameters->misbehaviorThreshold = par("misbehaviorThreshold");
        mParameters->updateTimeStep = par("updateTimeStep");
        mParameters->enableWebGui = par("enableWebGui");
        mParameters->webGuiDataUrl = par("webGuiDataUrl").stdstringValue();
        mParameters->guiJsonDataUpdateInterval = par("guiJsonDataUpdateInterval");
        mParameters->displaySteps = par("displaySteps");
        mParameters->recentReportedCount = par("recentReportedCount");

        mParameters->enableVectorRecording = par("enableVectorRecording");

        if (mParameters->enableWebGui) {
            mMsgGuiUpdate = new cMessage("getDataScheduleMessage");
            scheduleAt(simTime() + mParameters->guiJsonDataUpdateInterval,
                       mMsgGuiUpdate);
        }
        mMsgReportCleanup = new cMessage("reportCleanupMessage");
        scheduleAt(simTime() + mParameters->reportCleanupInterval,
                   mMsgReportCleanup);

        statsValidLevel2ReportEvidenceCount.setName("validLevel2ReportEvidenceCount");
        statsInvalidLevel2ReportEvidenceCount.setName("invalidLevel2ReportEvidenceCount");

    }

    void MisbehaviorAuthority::finish() {
        recordScalar("totalReportCount", mTotalReportCount);
        recordScalar("truePositiveCount", mTruePositiveCount);
        recordScalar("falsePositiveCount", mFalsePositiveCount);
        recordScalar("parsedReportCount", mParsedReportCount);
        recordScalar("validReportCount", mValidReportCount);
        statsValidLevel2ReportEvidenceCount.record();
        statsInvalidLevel2ReportEvidenceCount.record();
//        for(auto& stat : statsValidLevel2ReportSizes){
//            stat.record();
//        }
//        for(auto& stat : statsInvalidLevel2ReportSizes){
//            stat.record();
//        }

        for (const auto &r: mReportedPseudonyms) {
            auto reportedPseudonym = r.second;
            reportedPseudonym->recordStatistics();
            std::string name = "reportedPseudonym_" + std::to_string(reportedPseudonym->getStationId());
            if (reportedPseudonym->falsePositiveCount > 0) {
                recordScalar((name + "_count_FP").c_str(), reportedPseudonym->falsePositiveCount);
                recordScalar((name + "_score_sum_FP").c_str(), reportedPseudonym->falsePositiveScoreSum);
            } else {
                recordScalar((name + "_count_TP").c_str(), reportedPseudonym->truePositiveCount);
            }
            recordScalar((name + "_score_total").c_str(), reportedPseudonym->getTotalScore());
            recordScalar((name + "_valid_total").c_str(), reportedPseudonym->getValidReportCount());
        }
        for (const auto &reportingPseudonym: mReportingPseudonyms) {
            reportingPseudonym.second->recordStatistics();
        }

        for (int i = 1; i <= 20; i++) {
            std::string name = "attack_" + std::to_string(i) + "_" + attackTypes::AttackNames[i] + "_pseudonyms_";
            cHistogram attackTypeReportCounts((name + "reportCounts").c_str());
            cHistogram attackTypeReportScores((name + "reportScores").c_str());
            int detectedPseudonyms = 0;
            int undetectedPseudonyms = 0;
            for (const auto &misbehavingPseudonym: mMisbehavingPseudonyms) {
                if (misbehavingPseudonym.second->getAttackType() == attackTypes::intAttacks[i]) {
                    StationID_t misbehavingStationId = misbehavingPseudonym.second->getStationId();
                    auto it = mReportedPseudonyms.find(misbehavingStationId);
                    if (it != mReportedPseudonyms.end()) {
                        attackTypeReportScores.collect(it->second->getTotalScore());
                        attackTypeReportCounts.collect(it->second->getTotalReportCount());
                        detectedPseudonyms++;
                    } else {
                        attackTypeReportScores.collect(0);
                        attackTypeReportCounts.collect(0);
                        undetectedPseudonyms++;
                    }
                }
            }

            attackTypeReportCounts.record();
            attackTypeReportScores.record();
            recordScalar((name + "detectedCount").c_str(), detectedPseudonyms);
            recordScalar((name + "undetectedCount").c_str(), undetectedPseudonyms);
        }

        for (int i = 1; i <= 20; i++) {
            std::string name = "attack_" + std::to_string(i) + "_" + attackTypes::AttackNames[i] + "_vehicles_";
            cHistogram attackTypeReportCounts((name + "reportCounts").c_str());
            cHistogram attackTypeReportScores((name + "reportScores").c_str());
            int detectedVehicles = 0;
            int undetectedVehicles = 0;
            for (const auto &misbehavingVehicle: mMisbehavingVehiclesByAttackType[attackTypes::AttackTypes(i)]) {
                auto it = mDetectedVehiclesByAttackType[attackTypes::AttackTypes(i)].find(misbehavingVehicle);
                if (it != mDetectedVehiclesByAttackType[attackTypes::AttackTypes(i)].end()) {
                    detectedVehicles++;
                    double vehicleTotalScore = 0;
                    int vehicleTotalCount = 0;
                    for (const auto &misbehavingPseudonym: misbehavingVehicle->getPseudonyms()) {
                        auto it2 = mReportedPseudonyms.find(misbehavingPseudonym->getStationId());
                        if (it2 != mReportedPseudonyms.end()) {
                            vehicleTotalScore += it2->second->getTotalScore();
                            vehicleTotalCount += it2->second->getTotalReportCount();
                        }
                    }
                    attackTypeReportScores.collect(vehicleTotalScore);
                    attackTypeReportCounts.collect(vehicleTotalCount);
                } else {
                    attackTypeReportScores.collect(0);
                    attackTypeReportCounts.collect(0);
                    undetectedVehicles++;
                }
            }
            attackTypeReportCounts.record();
            attackTypeReportScores.record();
            recordScalar((name + "detectedCount").c_str(), detectedVehicles);
            recordScalar((name + "undetectedCount").c_str(), undetectedVehicles);

        }
        cComponent::finish();
    }

    void MisbehaviorAuthority::handleMessage(omnetpp::cMessage *msg) {
        Enter_Method("handleMessage");
        if (msg == mMsgGuiUpdate) {
//            createGuiJsonData();
//            scheduleAt(simTime() + mParameters->guiJsonDataUpdateInterval,
//                       mMsgGuiUpdate);
        } else if (msg == mMsgReportCleanup) {
            removeOldReports();
            scheduleAt(simTime() + mParameters->reportCleanupInterval,
                       mMsgReportCleanup);
        }
    }

    void MisbehaviorAuthority::removeOldReports() {
        for (auto it = mCurrentReports.begin(); it != mCurrentReports.end();) {
            auto report = it->second;
            if (countTaiMilliseconds(mTimer.getTimeFor(simTime())) - report->generationTime >
                uint64_t(mParameters->reportCleanupAge * 1000)) {
                mCurrentReports.erase(it++);
                continue;
            }
            it++;
        }
    }

    void MisbehaviorAuthority::receiveSignal(cComponent *source, simsignal_t signal, const SimTime &,
                                             cObject *) {
        if (signal == traciInitSignal) {
            auto core = check_and_cast<traci::Core *>(source);
            mTraciAPI = core->getAPI();
            mSimulationBoundary = traci::Boundary{mTraciAPI->simulation.getNetBoundary()};
            switch (mParameters->checkType) {
                case checkTypes::LegacyChecks:
                    mBaseChecks = new LegacyChecks(mTraciAPI, mGlobalEnvironmentModel,
                                                   &F2MDParameters::detectionParameters,
                                                   mParameters->misbehaviorThreshold,
                                                   &mTimer);
                    break;
                case checkTypes::CatchChecks:
                    mBaseChecks = new CatchChecks(mTraciAPI, mGlobalEnvironmentModel,
                                                  &F2MDParameters::detectionParameters,
                                                  mParameters->misbehaviorThreshold,
                                                  &mTimer);
            }
        } else if (signal == traciCloseSignal) {
            clear();
        }
    }

    void MisbehaviorAuthority::receiveSignal(cComponent *source, omnetpp::simsignal_t signal, cObject *obj,
                                             cObject *) {
        Enter_Method("receiveSignal");
        if (signal == maNewReport) {
            auto *reportObject = dynamic_cast<MisbehaviorReportObject *>(obj);
            std::shared_ptr<Report> report = std::make_shared<Report>(*reportObject->shared_ptr());
            if (report->successfullyParsed) {
                processReport(report);
            }
        } else if (signal == maMisbehaviorAnnouncement) {
            std::vector<StationID_t> stationIds = *reinterpret_cast<std::vector<StationID_t> *>(obj);
            auto misbehaviorCaService = check_and_cast<MisbehaviorCaService *>(source);
            processMisbehaviorAnnouncement(stationIds, misbehaviorCaService);

            auto misbehaviorCpmService = check_and_cast<MisbehaviorCpmService *>(source);
            processMisbehaviorAnnouncement(stationIds, misbehaviorCpmService);
        }
    }

    void MisbehaviorAuthority::processMisbehaviorAnnouncement(const std::vector<StationID_t> &stationIds,
                                                              MisbehaviorCaService *misbehaviorCaService) {
        if (stationIds.size() > 1) {
            StationID_t vehicleStationId = stationIds.front();
            attackTypes::AttackTypes attackType = misbehaviorCaService->getAttackType();
            std::shared_ptr<MisbehavingVehicle> misbehavingVehicle =
                    std::make_shared<MisbehavingVehicle>(vehicleStationId,
                                                         misbehaviorCaService->getMisbehaviorType(),
                                                         attackType);
            mMisbehavingVehicles[vehicleStationId] = misbehavingVehicle;
            mMisbehavingVehiclesByAttackType[attackType].insert(misbehavingVehicle);
            for (const auto &stationId: stationIds) {
                std::shared_ptr<MisbehavingPseudonym> misbehavingPseudonym =
                        std::make_shared<MisbehavingPseudonym>(stationId,
                                                               misbehaviorCaService->getMisbehaviorType(),
                                                               attackType, misbehavingVehicle);
                mMisbehavingPseudonyms[stationId] = misbehavingPseudonym;
                misbehavingVehicle->addPseudonym(misbehavingPseudonym);
                std::string prefix =
                        "misbehavingPseudonym_" + std::to_string(misbehavingPseudonym->getStationId()) + "_";
                recordScalar((prefix + "misbehaviorType").c_str(), misbehavingPseudonym->getMisbehaviorType());
                recordScalar((prefix + "attackType").c_str(), attackType);
            }
        } else {
            StationID_t vehicleStationId = stationIds.front();
            attackTypes::AttackTypes attackType = misbehaviorCaService->getAttackType();
            std::shared_ptr<MisbehavingVehicle> misbehavingVehicle =
                    std::make_shared<MisbehavingVehicle>(vehicleStationId,
                                                         misbehaviorCaService->getMisbehaviorType(),
                                                         attackType);
            mMisbehavingVehicles[vehicleStationId] = misbehavingVehicle;
            mMisbehavingVehiclesByAttackType[attackType].insert(misbehavingVehicle);

            std::shared_ptr<MisbehavingPseudonym> misbehavingPseudonym =
                    std::make_shared<MisbehavingPseudonym>(vehicleStationId,
                                                           misbehaviorCaService->getMisbehaviorType(),
                                                           attackType, misbehavingVehicle);
            mMisbehavingPseudonyms[vehicleStationId] = misbehavingPseudonym;
            misbehavingVehicle->addPseudonym(misbehavingPseudonym);
            std::string prefix =
                    "misbehavingPseudonym_" + std::to_string(misbehavingPseudonym->getStationId()) + "_";
            recordScalar((prefix + "misbehaviorType").c_str(), misbehavingPseudonym->getMisbehaviorType());
            recordScalar((prefix + "attackType").c_str(), attackType);
        }
    }

    void MisbehaviorAuthority::processMisbehaviorAnnouncement(const std::vector<StationID_t> &stationIds,
                                                              MisbehaviorCpmService *misbehaviorCpmService) {
        if (stationIds.size() > 1) {
            StationID_t vehicleStationId = stationIds.front();
            attackTypes::AttackTypes attackType = misbehaviorCpmService->getAttackType();
            std::shared_ptr<MisbehavingVehicle> misbehavingVehicle =
                    std::make_shared<MisbehavingVehicle>(vehicleStationId,
                                                         misbehaviorCpmService->getMisbehaviorType(),
                                                         attackType);
            mMisbehavingVehicles[vehicleStationId] = misbehavingVehicle;
            mMisbehavingVehiclesByAttackType[attackType].insert(misbehavingVehicle);
            for (const auto &stationId: stationIds) {
                std::shared_ptr<MisbehavingPseudonym> misbehavingPseudonym =
                        std::make_shared<MisbehavingPseudonym>(stationId,
                                                               misbehaviorCpmService->getMisbehaviorType(),
                                                               attackType, misbehavingVehicle);
                mMisbehavingPseudonyms[stationId] = misbehavingPseudonym;
                misbehavingVehicle->addPseudonym(misbehavingPseudonym);
                std::string prefix =
                        "misbehavingPseudonym_" + std::to_string(misbehavingPseudonym->getStationId()) + "_";
                recordScalar((prefix + "misbehaviorType").c_str(), misbehavingPseudonym->getMisbehaviorType());
                recordScalar((prefix + "attackType").c_str(), attackType);
            }
        } else {
            StationID_t vehicleStationId = stationIds.front();
            attackTypes::AttackTypes attackType = misbehaviorCpmService->getAttackType();
            std::shared_ptr<MisbehavingVehicle> misbehavingVehicle =
                    std::make_shared<MisbehavingVehicle>(vehicleStationId,
                                                         misbehaviorCpmService->getMisbehaviorType(),
                                                         attackType);
            mMisbehavingVehicles[vehicleStationId] = misbehavingVehicle;
            mMisbehavingVehiclesByAttackType[attackType].insert(misbehavingVehicle);

            std::shared_ptr<MisbehavingPseudonym> misbehavingPseudonym =
                    std::make_shared<MisbehavingPseudonym>(vehicleStationId,
                                                           misbehaviorCpmService->getMisbehaviorType(),
                                                           attackType, misbehavingVehicle);
            mMisbehavingPseudonyms[vehicleStationId] = misbehavingPseudonym;
            misbehavingVehicle->addPseudonym(misbehavingPseudonym);
            std::string prefix =
                    "misbehavingPseudonym_" + std::to_string(misbehavingPseudonym->getStationId()) + "_";
            recordScalar((prefix + "misbehaviorType").c_str(), misbehavingPseudonym->getMisbehaviorType());
            recordScalar((prefix + "attackType").c_str(), attackType);
        }
    }

    void MisbehaviorAuthority::processMisbehaviorAnnouncement(const std::vector<StationID_t> &stationIds,
                                                              MisbehaviorCpmService *misbehaviorCpmService) {
        if (stationIds.size() > 1) {
            StationID_t vehicleStationId = stationIds.front();
            attackTypes::AttackTypes attackType = misbehaviorCpmService->getAttackType();
            std::shared_ptr<MisbehavingVehicle> misbehavingVehicle =
                    std::make_shared<MisbehavingVehicle>(vehicleStationId,
                                                         misbehaviorCpmService->getMisbehaviorType(),
                                                         attackType);
            mMisbehavingVehicles[vehicleStationId] = misbehavingVehicle;
            mMisbehavingVehiclesByAttackType[attackType].insert(misbehavingVehicle);
            for (const auto &stationId: stationIds) {
                std::shared_ptr<MisbehavingPseudonym> misbehavingPseudonym =
                        std::make_shared<MisbehavingPseudonym>(stationId,
                                                               misbehaviorCpmService->getMisbehaviorType(),
                                                               attackType, misbehavingVehicle);
                mMisbehavingPseudonyms[stationId] = misbehavingPseudonym;
                misbehavingVehicle->addPseudonym(misbehavingPseudonym);
                std::string prefix =
                        "misbehavingPseudonym_" + std::to_string(misbehavingPseudonym->getStationId()) + "_";
                recordScalar((prefix + "misbehaviorType").c_str(), misbehavingPseudonym->getMisbehaviorType());
                recordScalar((prefix + "attackType").c_str(), attackType);
            }
        } else {
            StationID_t vehicleStationId = stationIds.front();
            attackTypes::AttackTypes attackType = misbehaviorCpmService->getAttackType();
            std::shared_ptr<MisbehavingVehicle> misbehavingVehicle =
                    std::make_shared<MisbehavingVehicle>(vehicleStationId,
                                                         misbehaviorCpmService->getMisbehaviorType(),
                                                         attackType);
            mMisbehavingVehicles[vehicleStationId] = misbehavingVehicle;
            mMisbehavingVehiclesByAttackType[attackType].insert(misbehavingVehicle);

            std::shared_ptr<MisbehavingPseudonym> misbehavingPseudonym =
                    std::make_shared<MisbehavingPseudonym>(vehicleStationId,
                                                           misbehaviorCpmService->getMisbehaviorType(),
                                                           attackType, misbehavingVehicle);
            mMisbehavingPseudonyms[vehicleStationId] = misbehavingPseudonym;
            misbehavingVehicle->addPseudonym(misbehavingPseudonym);
            std::string prefix =
                    "misbehavingPseudonym_" + std::to_string(misbehavingPseudonym->getStationId()) + "_";
            recordScalar((prefix + "misbehaviorType").c_str(), misbehavingPseudonym->getMisbehaviorType());
            recordScalar((prefix + "attackType").c_str(), attackType);
        }
    }

    void MisbehaviorAuthority::processReport(const std::shared_ptr<Report> &report) {
        mTotalReportCount++;
        mNewReport = true;

        std::shared_ptr<ReportedPseudonym> reportedPseudonym;
        std::shared_ptr<ReportingPseudonym> reportingPseudonym;
        {
            StationID_t reportedStationId = (*report->reportedMessage)->header.stationID;
            auto it = mReportedPseudonyms.find(reportedStationId);
            if (it != mReportedPseudonyms.end()) {
                reportedPseudonym = it->second;
            } else {
                reportedPseudonym = std::make_shared<ReportedPseudonym>(report);
                mReportedPseudonyms.emplace(reportedStationId, reportedPseudonym);
            }
        }

        {
            StationID_t reporterStationId = std::stoull(split(report->reportId, '_')[1]);
            auto it = mReportingPseudonyms.find(reporterStationId);
            if (it != mReportingPseudonyms.end()) {
                reportingPseudonym = it->second;
            } else {
                reportingPseudonym = std::make_shared<ReportingPseudonym>(reporterStationId,
                                                                          mParameters->enableVectorRecording);
                mReportingPseudonyms.emplace(reporterStationId, reportingPseudonym);
            }
        }
        report->isValid = validateReportReason(report);
        if(report->detectionType.present == detectionTypes::SemanticType &&
           report->detectionType.semantic->detectionLevel == detectionLevels::Level2){
            if (report->isValid) {
                statsValidLevel2ReportEvidenceCount.collect(report->evidence.reportedMessages.size());
            } else {
                statsInvalidLevel2ReportEvidenceCount.collect(report->evidence.reportedMessages.size());
            }
        }
        report->score = scoreReport(report, reportingPseudonym);
        report->reportingPseudonym = reportingPseudonym;
        report->reportedPseudonym = reportedPseudonym;
        reportingPseudonym->addReport(report);
        reportedPseudonym->addReport(report);
        mCurrentReports.emplace(report->reportId, report);

        updateReactionType(reportedPseudonym);
        updateDetectionRates(report, reportedPseudonym, reportingPseudonym);
    }

    double MisbehaviorAuthority::scoreReport(const std::shared_ptr<Report> &report,
                                             const std::shared_ptr<ReportingPseudonym> &reportingPseudonym) {
        uint64_t currentTime = countTaiMilliseconds(mTimer.getTimeFor(simTime()));
        double ageScore = 1;
        double evidenceScore = 1;
        double validityScore = 1;
        double reporterScore = 1;

        if (mParameters->considerReportAge) {
            double reportAge = min((double) (currentTime - report->generationTime) / 1000.0,
                                   mParameters->maxReportAge);
            ageScore = 1 - normalizeValue(reportAge, 0, mParameters->maxReportAge);
        }
        if (mParameters->considerReportValidity) {
            validityScore = report->isValid;
        }

        if (mParameters->considerReporterScore) {
            reporterScore = std::max(0.1, reportingPseudonym->getAverageReportScore());
        }

        if (mParameters->considerEvidenceScore) {
            if (report->detectionType.present == detectionTypes::SemanticType &&
                report->detectionType.semantic->detectionLevel == detectionLevels::Level2) {
                int evidenceCamCount = (int) report->evidence.reportedMessages.size();
                if (evidenceCamCount < mParameters->evidenceScoreMaxCamCount) {
                    evidenceScore = normalizeValue(evidenceCamCount, 1, mParameters->evidenceScoreMaxCamCount);
                }
            }
        }

        double reportScore = ageScore * validityScore * reporterScore * evidenceScore;
        if (isnan(reportScore)) {
            reportScore = 0;
        } else if (isinf(reportScore)) {
            reportScore = 1;
        }
        return reportScore;
    }

    bool compareErrorCodes(std::bitset<16> reportedErrorCodes, std::bitset<16> actualErrorCodes) {
        for (int i = 0; i < actualErrorCodes.size(); i++) {
            if (reportedErrorCodes[i] == 1 && actualErrorCodes[i] == 0) {
                return false;
            }
        }
        return true;
    }

    bool MisbehaviorAuthority::validateSemanticLevel1Report(const std::shared_ptr<Report> &report) {
        std::bitset<16> actualErrorCodes = mBaseChecks->checkSemanticLevel1Report((report->reportedMessage));
        std::bitset<16> reportedErrorCodes = report->detectionType.semantic->errorCode;
        return compareErrorCodes(reportedErrorCodes, actualErrorCodes);
    }

    bool MisbehaviorAuthority::validateSemanticLevel2Report(const std::shared_ptr<Report> &report) {
        std::bitset<16> actualErrorCodes;
        std::bitset<16> reportedErrorCodes = report->detectionType.semantic->errorCode;
        const std::vector<std::shared_ptr<vanetza::asn1::Cam>> &reportedMessages = report->evidence.reportedMessages;
        mBaseChecks->initializeKalmanFilters(reportedMessages.front());
//        for (int i = (int) reportedMessages.size() - 1; i > 0; i--) {
        for (int i = 1; i < (int) reportedMessages.size(); i++) {
            actualErrorCodes |= mBaseChecks->checkSemanticLevel2Report(reportedMessages[i],
                                                                       reportedMessages[i - 1]);
        }
        actualErrorCodes |= mBaseChecks->checkSemanticLevel2Report(report->reportedMessage,
                                                                   reportedMessages.back());
        return compareErrorCodes(reportedErrorCodes, actualErrorCodes);
    }

    bool MisbehaviorAuthority::validateSemanticLevel3Report(const std::shared_ptr<Report> &report) {
        std::bitset<16> actualErrorCodes = mBaseChecks->checkSemanticLevel3Report(report->reportedMessage,
                                                                                  report->evidence.neighbourMessages);
        std::bitset<16> reportedErrorCodes = report->detectionType.semantic->errorCode;

        return compareErrorCodes(reportedErrorCodes, actualErrorCodes);
    }

    bool MisbehaviorAuthority::validateSemanticLevel4Report(const std::shared_ptr<Report> &report) {
        std::bitset<16> actualErrorCodes = mBaseChecks->checkSemanticLevel4Report(report->reportedMessage,
                                                                                  report->evidence.neighbourMessages,
                                                                                  *report->evidence.senderInfo);
        std::bitset<16> reportedErrorCodes = report->detectionType.semantic->errorCode;
        return compareErrorCodes(reportedErrorCodes, actualErrorCodes);
    }

    bool MisbehaviorAuthority::validateReportReason(const std::shared_ptr<Report> &report) {
        if (report->detectionType.semantic != nullptr) {
            switch (report->detectionType.semantic->detectionLevel) {
                case detectionLevels::Level1:
                    return validateSemanticLevel1Report(report);
                case detectionLevels::Level2:
                    return validateSemanticLevel2Report(report);
                case detectionLevels::Level3:
                    return validateSemanticLevel3Report(report);
                case detectionLevels::Level4:
                    return validateSemanticLevel4Report(report);
                default:
                    break;
            }
        }
        return false;
    }

    std::shared_ptr<MisbehavingPseudonym> MisbehaviorAuthority::getMisbehavingPseudonym(const StationID_t &stationId) {
        auto it = mMisbehavingPseudonyms.find(stationId);
        if (it != mMisbehavingPseudonyms.end()) {
            return (*it).second;
        } else {
            return nullptr;
        }
    }

    void MisbehaviorAuthority::updateReactionType(const shared_ptr<ReportedPseudonym> &reportedPseudonym) {
        size_t score = reportedPseudonym->getTotalScore();
        reactionTypes::ReactionTypes newReactionType = reactionTypes::Nothing;
        if (score > 10) {
            newReactionType = reactionTypes::Warning;
        } else if (score > 50) {
            newReactionType = reactionTypes::Ticket;
        } else if (score > 250) {
            newReactionType = reactionTypes::PassiveRevocation;
        } else if (score > 1250) {
            newReactionType = reactionTypes::ActiveRevocation;
        }
        reactionTypes::ReactionTypes oldReactionType = reportedPseudonym->getReactionType();
        if (newReactionType != oldReactionType) {
            mReactionsList[oldReactionType].erase(reportedPseudonym->getStationId());
            mReactionsList[newReactionType].insert(reportedPseudonym->getStationId());
            reportedPseudonym->setReactionType(newReactionType);
        }
    }

    void MisbehaviorAuthority::updateDetectionRates(const std::shared_ptr<Report> &report,
                                                    const std::shared_ptr<ReportedPseudonym> &reportedPseudonym,
                                                    const std::shared_ptr<ReportingPseudonym> &reportingPseudonym) {

        misbehaviorTypes::MisbehaviorTypes predictedMisbehaviorType =
                reportedPseudonym->predictMisbehaviorType();
        misbehaviorTypes::MisbehaviorTypes predictedMisbehaviorTypeAggregated =
                reportedPseudonym->predictMisbehaviorTypeAggregate();

        std::shared_ptr<MisbehavingPseudonym> reportedMisbehavingPseudonym =
                getMisbehavingPseudonym(reportedPseudonym->getStationId());
        std::shared_ptr<MisbehavingPseudonym> reportingMisbehavingPseudonym =
                getMisbehavingPseudonym(reportingPseudonym->getStationId());
        misbehaviorTypes::MisbehaviorTypes actualMisbehaviorType;


        if (reportedMisbehavingPseudonym != nullptr) {
            actualMisbehaviorType = reportedMisbehavingPseudonym->getMisbehaviorType();
        } else {
            actualMisbehaviorType = misbehaviorTypes::Benign;
        }
        if (report->score > 0) {
            if (actualMisbehaviorType == misbehaviorTypes::LocalAttacker) {
                mDetectedVehiclesByAttackType[reportedMisbehavingPseudonym->getAttackType()].insert(
                        reportedMisbehavingPseudonym->getVehicle());
                reportedPseudonym->truePositiveCount++;
            } else if (actualMisbehaviorType == misbehaviorTypes::Benign &&
                       (reportingMisbehavingPseudonym == nullptr ||
                        (reportingMisbehavingPseudonym != nullptr &&
                         reportingMisbehavingPseudonym->getAttackType() != attackTypes::FakeReport))) {
                reportedPseudonym->falsePositiveCount++;
                reportedPseudonym->falsePositiveScoreSum += report->score;
            }
        }


        if (predictedMisbehaviorType == actualMisbehaviorType) {
            mTruePositiveCount++;
        } else {
            mFalsePositiveCount++;
        }
        mDetectionRate = 100 * mTruePositiveCount / (double) (mTruePositiveCount + mFalsePositiveCount);
        if (predictedMisbehaviorTypeAggregated == actualMisbehaviorType) {
            mTrueDetectionAggregateCount++;
        } else {
            mFalseDetectionAggregateCount++;
        }

        mDetectionRateAggregate = 100 * mTrueDetectionAggregateCount /
                                  ((double) mTrueDetectionAggregateCount + mFalseDetectionAggregateCount);
        if (report->generationTime - mLastUpdateTime >
            (long) mParameters->updateTimeStep * 1000) {
            mLastUpdateTime = report->generationTime;
            auto time = (std::time_t) (report->generationTime / 1000 + 1072915200 - 5);
            std::tm t_tm = *std::gmtime(&time);
            std::stringstream ss;
            ss << std::put_time(&t_tm, "%H:%M:%S");
            std::string timeString = ss.str();
            mDetectionAccuracyLabels.emplace_back(timeString);
            int trueDetectionCurrent = mTruePositiveCount - mTrueDetectionCountInst;
            int falseDetectionCurrent = mFalsePositiveCount - mFalseDetectionCountInst;
            if (trueDetectionCurrent + falseDetectionCurrent > 0) {
                mDetectionRateCur = 100 * trueDetectionCurrent / (trueDetectionCurrent + falseDetectionCurrent);
            }
            mTrueDetectionCountInst = mTruePositiveCount;
            mFalseDetectionCountInst = mFalsePositiveCount;
            auto data = std::make_tuple(trueDetectionCurrent, falseDetectionCurrent, mDetectionRateCur);
            mDetectionAccuracyData.emplace_back(data);
            if (mDetectionAccuracyData.size() > mParameters->displaySteps) {
                mDetectionAccuracyData.pop_front();
                mDetectionAccuracyLabels.pop_front();
            }
        }
    }

/*    rapidjson::Value MisbehaviorAuthority::getRadarData(rapidjson::Document::AllocatorType &allocator) {
        rapidjson::Value reactionsData(rapidjson::kObjectType);
        std::vector<int> reactionsBenign(5, 0);
        std::vector<int> reactionsMalicious(5, 0);

        for (const auto &reportedPseudonym: mReportedPseudonyms) {
            misbehaviorTypes::MisbehaviorTypes misbehaviorType = getActualMisbehaviorType(
                    reportedPseudonym.second->getStationId());
            if (misbehaviorType == misbehaviorTypes::Benign) {
                reactionsBenign[static_cast<int>(reportedPseudonym.second->getReactionType())]++;
            } else if (misbehaviorType == misbehaviorTypes::LocalAttacker ||
                       misbehaviorType == misbehaviorTypes::GlobalAttacker) {
                reactionsMalicious[static_cast<int>(reportedPseudonym.second->getReactionType())]++;
            }
        }
        rapidjson::Value reactionsBenignJson;
        rapidjson::Value reactionsMaliciousJson;
        reactionsBenignJson.SetArray();
        reactionsMaliciousJson.SetArray();

        int sumReactionsBenign = std::accumulate(reactionsBenign.begin(), reactionsBenign.end(), 0);
        int sumReactionsMalicious = std::accumulate(reactionsMalicious.begin(), reactionsMalicious.end(), 0);
        if (sumReactionsBenign > 0) {
            for (auto &reaction: reactionsBenign) {
                reactionsBenignJson.PushBack(100.0 * reaction / sumReactionsBenign, allocator);
            }
        } else {
            for (auto &reaction: reactionsBenign) {
                reactionsBenignJson.PushBack(0, allocator);
            }
        }
        if (sumReactionsMalicious > 0) {
            for (auto &reaction: reactionsMalicious) {
                reactionsMaliciousJson.PushBack(100 * reaction / sumReactionsMalicious, allocator);
            }
        } else {
            for (auto &reaction: reactionsMalicious) {
                reactionsMaliciousJson.PushBack(0, allocator);
            }
        }
        reactionsData.AddMember("benign", reactionsBenignJson, allocator);
        reactionsData.AddMember("malicious", reactionsMaliciousJson, allocator);
        return reactionsData;
    }

    rapidjson::Value MisbehaviorAuthority::getRecentReported(rapidjson::Document::AllocatorType &allocator) {
        rapidjson::Value recentlyReportedData(rapidjson::kObjectType);
        rapidjson::Value labels;
        rapidjson::Value data;
        labels.SetArray();
        data.SetArray();
        for (auto r: getRecentReported()) {
            labels.PushBack(r.stationId, allocator);
            data.PushBack(r.reportCount, allocator);
        }
        recentlyReportedData.AddMember("labels", labels, allocator);
        recentlyReportedData.AddMember("data", data, allocator);
        return recentlyReportedData;
    }

    rapidjson::Value MisbehaviorAuthority::getDetectionRates(rapidjson::Document::AllocatorType &allocator) {
        rapidjson::Value detectionRatesData(rapidjson::kObjectType);
        rapidjson::Value detectionRatesLabels;
        rapidjson::Value accurate;
        rapidjson::Value notAccurate;
        rapidjson::Value rate;
        detectionRatesLabels.SetArray();
        accurate.SetArray();
        notAccurate.SetArray();
        rate.SetArray();
        for (const auto &label: mDetectionAccuracyLabels) {
            rapidjson::Value v;
            v.SetString(label.c_str(), label.length(), allocator);
            detectionRatesLabels.PushBack(v, allocator);
        }
        for (const auto &dAData: mDetectionAccuracyData) {
            accurate.PushBack(std::get<0>(dAData), allocator);
            notAccurate.PushBack(std::get<1>(dAData) * -1, allocator);
            rate.PushBack(std::get<2>(dAData), allocator);
        }
        detectionRatesData.AddMember("labels", detectionRatesLabels, allocator);
        detectionRatesData.AddMember("accurate", accurate, allocator);
        detectionRatesData.AddMember("notAccurate", notAccurate, allocator);
        detectionRatesData.AddMember("rate", rate, allocator);
        return detectionRatesData;
    }

    void MisbehaviorAuthority::createGuiJsonData() {
        rapidjson::Document d;
        d.SetObject();
        rapidjson::Document::AllocatorType &allocator = d.GetAllocator();

        d.AddMember("newReport", mNewReport, allocator);
        d.AddMember("totalReports", mTotalReportCount, allocator);
        d.AddMember("cumulativeDetectionRate", mDetectionRate, allocator);

        d.AddMember("reactionsData", getRadarData(allocator), allocator);
        d.AddMember("recentlyReportedData", getRecentReported(allocator), allocator);
        d.AddMember("detectionRatesData", getDetectionRates(allocator), allocator);

        rapidjson::StringBuffer strBuf;
        rapidjson::Writer<rapidjson::StringBuffer> writer(strBuf);
        d.Accept(writer);
        std::string jsonString = strBuf.GetString();
        curl_easy_setopt(curl, CURLOPT_URL, mParameters->webGuiDataUrl.c_str());
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, jsonString.c_str());
        CURLcode curlResponse = curl_easy_perform(curl);
        if (curlResponse != CURLE_OK) {
            std::cout << "request failed: " << curl_easy_strerror(curlResponse) << std::endl;
        }
        mNewReport = false;
    }

    bool sortByGenerationTime(RecentReported &a, RecentReported &b) {
        return a.lastGenerationTime < b.lastGenerationTime;
    }

    bool sortByStationId(RecentReported &a, RecentReported &b) {
        return a.stationId > b.stationId;
    }

    std::vector<RecentReported> MisbehaviorAuthority::getRecentReported() {
        std::vector<RecentReported> recentReported;
        for (const auto &r: mReportedPseudonyms) {
            auto reportedPseudonym = *r.second;
            if (getActualMisbehaviorType(reportedPseudonym.getStationId()) != misbehaviorTypes::Benign) {
                std::sort(recentReported.begin(), recentReported.end(), sortByGenerationTime);
                if (recentReported.size() < mParameters->recentReportedCount) {
                    recentReported.emplace_back(
                            RecentReported{reportedPseudonym.getStationId(), reportedPseudonym.getTotalScore(),
                                           reportedPseudonym.getPreviousReportGenerationTime()});
                } else {
                    if ((*recentReported.begin()).lastGenerationTime <
                        reportedPseudonym.getPreviousReportGenerationTime()) {
                        recentReported.erase(recentReported.begin());
                        recentReported.emplace_back(RecentReported{reportedPseudonym.getStationId(),
                                                                   reportedPseudonym.getTotalScore(),
                                                                   reportedPseudonym.getPreviousReportGenerationTime()});
                    }
                }
            }
        }
        std::sort(recentReported.begin(), recentReported.end(), sortByStationId);
        return recentReported;
    }

    void MisbehaviorAuthority::printReportsPerPseudonym() {
        int attackerCount = 0;
        int benignCount = 0;
        int reportTpCount = 0;
        int reportFpCount = 0;
        double meanReportsPerAttacker = 0;
        double meanReportsPerBenign = 0;
        double reportTpSdSum = 0;
        double reportFpSdSum = 0;
        double sdAttacker = 0;
        double sdBenign = 0;
        for (const auto &r: mReportedPseudonyms) {
            ReportedPseudonym reportedPseudonym = *r.second;
            if (getActualMisbehaviorType(reportedPseudonym.getStationId()) != misbehaviorTypes::Benign) {
                attackerCount++;
                reportTpCount += (int) reportedPseudonym.getValidReportCount();
            } else {
                benignCount++;
                reportFpCount += (int) reportedPseudonym.getValidReportCount();
            }
        }

        if (reportTpCount > 0) {
            meanReportsPerAttacker = (double) reportTpCount / attackerCount;
        }
        if (reportFpCount > 0) {
            meanReportsPerBenign = (double) reportFpCount / benignCount;
        }
        for (const auto &r: mReportedPseudonyms) {
            ReportedPseudonym reportedPseudonym = *r.second;
            if (getActualMisbehaviorType(reportedPseudonym.getStationId()) != misbehaviorTypes::Benign) {
                reportTpSdSum += pow((int) reportedPseudonym.getValidReportCount() - meanReportsPerAttacker, 2);
            } else {
                reportFpSdSum += pow((int) reportedPseudonym.getValidReportCount() - meanReportsPerBenign, 2);
            }
        }
        if (reportTpCount > 0) {
            sdAttacker = sqrt(reportTpSdSum / attackerCount);
        }
        if (reportFpCount > 0) {
            sdBenign = sqrt(reportFpSdSum / benignCount);
        }
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "Reports per malicious pseudonym: " << meanReportsPerAttacker << " StdDev: " << sdAttacker
                  << std::endl;
        std::cout << "Reports per benign pseudonym: " << meanReportsPerBenign << " StdDev: " << sdBenign << std::endl;
    }*/
} // namespace artery