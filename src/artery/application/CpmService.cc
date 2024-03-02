/*
* Artery V2X Simulation Framework
* Copyright 2014-2019 Raphael Riebl et al.
* Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
*/

#include "artery/application/CpmService.h"
#include "artery/application/CpmObject.h"
#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/MultiChannelPolicy.h"
#include "artery/utility/simtime_cast.h"
#include <boost/units/systems/si/prefixes.hpp>

namespace artery {

    using namespace omnetpp;

    namespace {
        auto microdegree = vanetza::units::degree * boost::units::si::micro;
        auto decidegree = vanetza::units::degree * boost::units::si::deci;
        auto degree_per_second = vanetza::units::degree / vanetza::units::si::second;
        auto centimeter_per_second = vanetza::units::si::meter_per_second * boost::units::si::centi;

        static const simsignal_t scSignalCpmReceived = cComponent::registerSignal("CpmReceived");
    }


    Define_Module(CpmService)

    template<typename T, typename U>
    long CpmService::round(const boost::units::quantity<T> &q, const U &u) {
        boost::units::quantity<U> v{q};
        return std::round(v.value());
    }

    void CpmService::initialize() {
        BaseCpmService::initialize();

        // set vehicle color to green (benign)
        getFacilities().get_const<traci::VehicleController>().getTraCI()->vehicle.setColor(
                getFacilities().get_const<traci::VehicleController>().getVehicleId(),
                libsumo::TraCIColor(0, 255, 0, 255));
    }


    bool CpmService::checkTriggeringConditions(const omnetpp::SimTime& now)
    {
        const auto elapsed = now - mLastCpm;
        const auto genCpmDcc = (mWithDccRestriction ? generateCpmDcc() : mGenCpmMin);

        if (elapsed >= genCpmDcc) {
            // Generation Rules or Redundancy Mitigation Rules might need to know
            // the projected generation time of the next CPM
            mGenCpm = genCpmDcc;

            return true;
        }

        return false;
    }

    bool CpmService::checkSensorInfoConditions(const omnetpp::SimTime& now)
    {
        return (now - mLastSensorInfoContainer >= mAddSensorInformation);
    }

    void CpmService::synchronizeLocalTracking(const omnetpp::SimTime& now)
    {
        EV_DEBUG << "Synchronizing local tracking\n";

        // To synchronize the LEM and LocalTracking leverage the fact that both maps use the same key element with the same ordering,
        // this allows to make a parallel, single run over the maps. The filtering does not change the order of the elements.
        // This assert tries to verify this fact, however it is too strict, both comparators do not need to be of the same type
        // but offer the same properties, however this is not so easy to test. Therefor test for the common case that both maps simply use the same comparator.
        static_assert(std::is_same<LocalEnvironmentModel::TrackedObjects::key_compare, cp::LocalTracking::key_compare>::value, "Incompatible comparators");
        // Can't compare the iterators of the two maps directly so use the comparator to compare their keys only
        auto cmp = LocalEnvironmentModel::TrackedObjects::key_compare();
        // The LEM contains not only object detected by physical sensors but also by virtual sensors like CAM or CPM.
        // We only need the objects from the physical sensors, right now only filtering by one category is available.
        auto localLem = filterBySensorCategory(mLocalEnvironmentModel->allObjects(), mLocalTrackingCategory);

        // The LEM enforces which objects are valid, update the local tracking accordingly
        auto lemObject = localLem.begin();
        auto localObject = mLocalTracking.begin();
        for (; lemObject != localLem.end() || localObject != mLocalTracking.end();) {
            if (lemObject == localLem.end()) {
                // End of LEM reached, all remaining local objects are expired, remove them
                // EV_TRACE << "End of LEM\n";
                if (localObject != mLocalTracking.end()) {
                    localObject = mLocalTracking.erase(localObject, mLocalTracking.end());
                }
                continue;
            }
            if (localObject != mLocalTracking.end()) {
                // LEM and Local object present
                if (cmp(lemObject->first, localObject->first)) {
                    // LEM object not present in local, insert
                    // EV_TRACE << "New LEM object: " << lemObject->first.lock().get() << "\n";
                    // Fallthrough to insert case
                } else if (cmp(localObject->first, lemObject->first)) {
                    // Local object not present in LEM, remove
                    // EV_TRACE << "Expired local object: " << localObject->first.lock().get() << "\n";
                    localObject = mLocalTracking.erase(localObject);
                    continue;
                } else {
                    // Both objects are the same, update
                    // EV_TRACE << "Equal objects: LEM " << lemObject->first.lock().get() << " - Local " << localObject->first.lock().get() << "\n";
                    if (lemObject->first.expired()) {
                        // Object expired, remove
                        // Can't modify LEM, just skip
                        ++lemObject;
                        localObject = mLocalTracking.erase(localObject);
                    } else {
                        // Object valid, update and advance
                        ++lemObject;
                        ++localObject;
                    }
                    continue;
                }
            }  // else End of local reached, add LEM object

            // Add only valid objects
            if (!lemObject->first.expired()) {
                // EV_TRACE << "Adding local object: " << lemObject->first.lock().get() << " - station: " <<
                // lemObject->first.lock()->getVehicleData().getStationId()
                //          << "\n";
                localObject = mLocalTracking.emplace_hint(localObject, std::piecewise_construct, std::forward_as_tuple(lemObject->first), std::forward_as_tuple());
                // Advance to prevent processing of the inserted object in next iteration
                ++localObject;
            }
            ++lemObject;
        }
    }

    void CpmService::synchronizeRemoteTracking(const omnetpp::SimTime& now)
    {
        EV_DEBUG << "Synchronizing remote tracking\n";

        // To synchronize RemoteTracking CAM objects leverage the fact that both maps use the same key element with the same ordering,
        // this allows to make a parallel, single run over the maps. The filtering does not change the order of the elements.
        // This assert tries to verify this fact, however it is too strict, both comparators do not need to be of the same type
        // but offer the same properties, however this is not so easy to test. Therefor test for the common case that both maps simply use the same comparator.
        static_assert(std::is_same<RemoteTracking::key_compare, LocalDynamicMap::AwarenessEntries::key_compare>::value, "Incompatible comparators");
        // Can't compare the iterators of the two maps directly so use the comparator to compare their keys only
        auto cmp = RemoteTracking::key_compare();

        const auto& cams = mLocalDynamicMap->allEntries();
        auto remoteObject = mRemoteTracking.begin();
        auto camObject = cams.begin();
        for (; remoteObject != mRemoteTracking.end();) {
            if (camObject != cams.end()) {
                if (cmp(camObject->first, remoteObject->first)) {
                    // No remote object for CAM object, insert
                    remoteObject = mRemoteTracking.emplace_hint(remoteObject, camObject->first, &(camObject->second.cam()));
                    ++remoteObject;
                    ++camObject;

                    continue;
                } else if (cmp(remoteObject->first, camObject->first)) {
                    // No CAM object for remote object, remove CAM
                    if (remoteObject->second.getCam()) {
                        remoteObject->second.setCam(nullptr);
                    }
                    // Fallthrough to check remote object
                } else {
                    // Both objects match, update
                    if (remoteObject->second.getCam() != &(camObject->second.cam())) {
                        remoteObject->second.setCam(&(camObject->second.cam()));
                    }
                    ++camObject;
                    // Fallthrough to check remote object
                }
            } else {
                // No CAM object for remote object, remove CAM
                if (remoteObject->second.getCam()) {
                    remoteObject->second.setCam(nullptr);
                }
                // Fallthrough to check remote object
            }

            remoteObject->second.clearExpired(now);
            if (remoteObject->second.isExpired()) {
                remoteObject = mRemoteTracking.erase(remoteObject);
            } else {
                ++remoteObject;
            }
        }
    }
    void CpmService::trigger()
    {

    bool CpmService::checkTriggeringConditions(const omnetpp::SimTime& now)
    {
        const auto elapsed = now - mLastCpm;
        const auto genCpmDcc = (mWithDccRestriction ? generateCpmDcc() : mGenCpmMin);

        if (elapsed >= genCpmDcc) {
            // Generation Rules or Redundancy Mitigation Rules might need to know
            // the projected generation time of the next CPM
            mGenCpm = genCpmDcc;

            return true;
        }

        return false;
    }

    bool CpmService::checkSensorInfoConditions(const omnetpp::SimTime& now)
    {
        return (now - mLastSensorInfoContainer >= mAddSensorInformation);
    }

    void CpmService::synchronizeLocalTracking(const omnetpp::SimTime& now)
    {
        EV_DEBUG << "Synchronizing local tracking\n";

        // To synchronize the LEM and LocalTracking leverage the fact that both maps use the same key element with the same ordering,
        // this allows to make a parallel, single run over the maps. The filtering does not change the order of the elements.
        // This assert tries to verify this fact, however it is too strict, both comparators do not need to be of the same type
        // but offer the same properties, however this is not so easy to test. Therefor test for the common case that both maps simply use the same comparator.
        static_assert(std::is_same<LocalEnvironmentModel::TrackedObjects::key_compare, cp::LocalTracking::key_compare>::value, "Incompatible comparators");
        // Can't compare the iterators of the two maps directly so use the comparator to compare their keys only
        auto cmp = LocalEnvironmentModel::TrackedObjects::key_compare();
        // The LEM contains not only object detected by physical sensors but also by virtual sensors like CAM or CPM.
        // We only need the objects from the physical sensors, right now only filtering by one category is available.
        auto localLem = filterBySensorCategory(mLocalEnvironmentModel->allObjects(), mLocalTrackingCategory);

        // The LEM enforces which objects are valid, update the local tracking accordingly
        auto lemObject = localLem.begin();
        auto localObject = mLocalTracking.begin();
        for (; lemObject != localLem.end() || localObject != mLocalTracking.end();) {
            if (lemObject == localLem.end()) {
                // End of LEM reached, all remaining local objects are expired, remove them
                // EV_TRACE << "End of LEM\n";
                if (localObject != mLocalTracking.end()) {
                    localObject = mLocalTracking.erase(localObject, mLocalTracking.end());
                }
                continue;
            }
            if (localObject != mLocalTracking.end()) {
                // LEM and Local object present
                if (cmp(lemObject->first, localObject->first)) {
                    // LEM object not present in local, insert
                    // EV_TRACE << "New LEM object: " << lemObject->first.lock().get() << "\n";
                    // Fallthrough to insert case
                } else if (cmp(localObject->first, lemObject->first)) {
                    // Local object not present in LEM, remove
                    // EV_TRACE << "Expired local object: " << localObject->first.lock().get() << "\n";
                    localObject = mLocalTracking.erase(localObject);
                    continue;
                } else {
                    // Both objects are the same, update
                    // EV_TRACE << "Equal objects: LEM " << lemObject->first.lock().get() << " - Local " << localObject->first.lock().get() << "\n";
                    if (lemObject->first.expired()) {
                        // Object expired, remove
                        // Can't modify LEM, just skip
                        ++lemObject;
                        localObject = mLocalTracking.erase(localObject);
                    } else {
                        // Object valid, update and advance
                        ++lemObject;
                        ++localObject;
                    }
                    continue;
                }
            }  // else End of local reached, add LEM object

            // Add only valid objects
            if (!lemObject->first.expired()) {
                // EV_TRACE << "Adding local object: " << lemObject->first.lock().get() << " - station: " <<
                // lemObject->first.lock()->getVehicleData().getStationId()
                //          << "\n";
                localObject = mLocalTracking.emplace_hint(localObject, std::piecewise_construct, std::forward_as_tuple(lemObject->first), std::forward_as_tuple());
                // Advance to prevent processing of the inserted object in next iteration
                ++localObject;
            }
            ++lemObject;
        }
    }

    void CpmService::synchronizeRemoteTracking(const omnetpp::SimTime& now)
    {
        EV_DEBUG << "Synchronizing remote tracking\n";

        // To synchronize RemoteTracking CAM objects leverage the fact that both maps use the same key element with the same ordering,
        // this allows to make a parallel, single run over the maps. The filtering does not change the order of the elements.
        // This assert tries to verify this fact, however it is too strict, both comparators do not need to be of the same type
        // but offer the same properties, however this is not so easy to test. Therefor test for the common case that both maps simply use the same comparator.
        static_assert(std::is_same<RemoteTracking::key_compare, LocalDynamicMap::AwarenessEntries::key_compare>::value, "Incompatible comparators");
        // Can't compare the iterators of the two maps directly so use the comparator to compare their keys only
        auto cmp = RemoteTracking::key_compare();

        const auto& cams = mLocalDynamicMap->allEntries();
        auto remoteObject = mRemoteTracking.begin();
        auto camObject = cams.begin();
        for (; remoteObject != mRemoteTracking.end();) {
            if (camObject != cams.end()) {
                if (cmp(camObject->first, remoteObject->first)) {
                    // No remote object for CAM object, insert
                    remoteObject = mRemoteTracking.emplace_hint(remoteObject, camObject->first, &(camObject->second.cam()));
                    ++remoteObject;
                    ++camObject;

                    continue;
                } else if (cmp(remoteObject->first, camObject->first)) {
                    // No CAM object for remote object, remove CAM
                    if (remoteObject->second.getCam()) {
                        remoteObject->second.setCam(nullptr);
                    }
                    // Fallthrough to check remote object
                } else {
                    // Both objects match, update
                    if (remoteObject->second.getCam() != &(camObject->second.cam())) {
                        remoteObject->second.setCam(&(camObject->second.cam()));
                    }
                    ++camObject;
                    // Fallthrough to check remote object
                }
            } else {
                // No CAM object for remote object, remove CAM
                if (remoteObject->second.getCam()) {
                    remoteObject->second.setCam(nullptr);
                }
                // Fallthrough to check remote object
            }

            remoteObject->second.clearExpired(now);
            if (remoteObject->second.isExpired()) {
                remoteObject = mRemoteTracking.erase(remoteObject);
            } else {
                ++remoteObject;
            }
        }
    }
    void CpmService::trigger()
    {
        Enter_Method("trigger");
        EV_TRACE << "trigger(...)\n";

        const auto now = opp::simTime();
        const auto& senderPosition = mVehicleDataProvider->position();

        // The bounding box processing of the evaluation script requires position statistics for each event number,
        // emit them here so that following code doesn't need to take care of this and to prevent to emit them multiple times
        emit(sc::cpmSent_positionX, cp::convertToUnit(senderPosition.x, vanetza::units::si::meter));
        emit(sc::cpmSent_positionY, cp::convertToUnit(senderPosition.y, vanetza::units::si::meter));

        // Synchronize object data already here to be able to generate statistics
        synchronizeLocalTracking(now);
        synchronizeRemoteTracking(now);

        // Count objects per class for statistics
        std::vector<StationID_t> uniqueObjects;
        uniqueObjects.reserve(mLocalTracking.size() + mRemoteTracking.size());
        size_t camObjects = 0;
        size_t cpmObjects = 0;
        for (const auto& localObject : mLocalTracking) { uniqueObjects.emplace_back(localObject.first.lock()->getVehicleData().getStationId()); }
        for (const auto& remoteObject : mRemoteTracking) {
            uniqueObjects.emplace_back(remoteObject.first);
            if (remoteObject.second.getCam()) {
                ++camObjects;
            }
            if (remoteObject.second.hasCpmDynamics()) {
                ++cpmObjects;
            }
        EV_TRACE << "trigger(...)\n";

        const auto now = simTime();
        const auto& senderPosition = mVehicleDataProvider->position();

        // The bounding box processing of the evaluation script requires position statistics for each event number,
        // emit them here so that following code doesn't need to take care of this and to prevent to emit them multiple times

        // Synchronize object data already here to be able to generate statistics
        synchronizeLocalTracking(now);
        synchronizeRemoteTracking(now);

        // Count objects per class for statistics
        std::vector<StationID_t> uniqueObjects;
        uniqueObjects.reserve(mLocalTracking.size() + mRemoteTracking.size());
        size_t camObjects = 0;
        size_t cpmObjects = 0;
        for (const auto& localObject : mLocalTracking) { uniqueObjects.emplace_back(localObject.first.lock()->getVehicleData().getStationId()); }
        for (const auto& remoteObject : mRemoteTracking) {
            uniqueObjects.emplace_back(remoteObject.first);
            if (remoteObject.second.getCam()) {
                ++camObjects;
            }
            if (remoteObject.second.hasCpmDynamics()) {
                ++cpmObjects;
            }
        }
        std::sort(uniqueObjects.begin(), uniqueObjects.end());
        uniqueObjects.erase(std::unique(uniqueObjects.begin(), uniqueObjects.end()), uniqueObjects.end());
        emit(sc::cpm_10Hz_numSeenByRadar, static_cast<unsigned long>(mLocalTracking.size()));
        emit(sc::cpm_10Hz_numSeenByCam, static_cast<unsigned long>(camObjects));
        emit(sc::cpm_10Hz_numSeenByCpm, static_cast<unsigned long>(cpmObjects));
        emit(sc::cpm_10Hz_numSeenByAll, static_cast<unsigned long>(uniqueObjects.size()));

        if (checkTriggeringConditions(now)) {
            EV_DEBUG << "CPM creation triggered\n";

            // The generation rule requires current and especially expired-pointer-free local tracking data
            // TODO: For now this is already done above
            // synchronizeLocalTracking(now);

            auto pocCandidates = mGenerationRule->getPocCandidates(mLocalTracking, now, mGenCpm);
            const auto generatedPocs = pocCandidates.first.size();
            if (!mDynamicRedundancyMitigation || getCurrentCbr() > mRedundancyLoad) {
                EV_DEBUG << "Applying Redundancy Mitigation\n";

                // The redundancy mitigation rules require current remote tracking data
                // TODO: For now this is already done above
                // synchronizeRemoteTracking(now);

                for (auto& redundancyMitigationRule : mRedundancyMitigationRules) {
                    redundancyMitigationRule->filterPocCandidates(pocCandidates.first, mRemoteTracking, *mVehicleDataProvider, now);
                }
            }
            const auto mitigatedPocs = pocCandidates.first.size();
            auto sic =
                    (checkSensorInfoConditions(now) ? createSensorInformationContainer()
                                                    : make_empty_asn1<SensorInformationContainer>(asn_DEF_SensorInformationContainer));

            emit(sc::cpmSent_generatedPocCount, static_cast<unsigned long>(generatedPocs));
            emit(sc::cpmSent_mitigatedPocCount, static_cast<unsigned long>(mitigatedPocs));

            if (!pocCandidates.first.empty() || pocCandidates.second || sic) {
                generateCollectivePerceptionMessage(now, pocCandidates.first, std::move(sic));
            }
        }
        std::sort(uniqueObjects.begin(), uniqueObjects.end());
        uniqueObjects.erase(std::unique(uniqueObjects.begin(), uniqueObjects.end()), uniqueObjects.end());
        emit(sc::cpm_10Hz_numSeenByRadar, static_cast<unsigned long>(mLocalTracking.size()));
        emit(sc::cpm_10Hz_numSeenByCam, static_cast<unsigned long>(camObjects));
        emit(sc::cpm_10Hz_numSeenByCpm, static_cast<unsigned long>(cpmObjects));
        emit(sc::cpm_10Hz_numSeenByAll, static_cast<unsigned long>(uniqueObjects.size()));

        if (checkTriggeringConditions(now)) {
            EV_DEBUG << "CPM creation triggered\n";

            // The generation rule requires current and especially expired-pointer-free local tracking data
            // TODO: For now this is already done above
            // synchronizeLocalTracking(now);

            auto pocCandidates = mGenerationRule->getPocCandidates(mLocalTracking, now, mGenCpm);
            const auto generatedPocs = pocCandidates.first.size();
            if (!mDynamicRedundancyMitigation || getCurrentCbr() > mRedundancyLoad) {
                EV_DEBUG << "Applying Redundancy Mitigation\n";

                // The redundancy mitigation rules require current remote tracking data
                // TODO: For now this is already done above
                // synchronizeRemoteTracking(now);

                for (auto& redundancyMitigationRule : mRedundancyMitigationRules) {
                    redundancyMitigationRule->filterPocCandidates(pocCandidates.first, mRemoteTracking, *mVehicleDataProvider, now);
                }
            }
            const auto mitigatedPocs = pocCandidates.first.size();
            auto sic =
                    (checkSensorInfoConditions(now) ? createSensorInformationContainer()
                                                    : cp::make_empty_asn1<SensorInformationContainer>(asn_DEF_SensorInformationContainer));

            emit(sc::cpmSent_generatedPocCount, static_cast<unsigned long>(generatedPocs));
            emit(sc::cpmSent_mitigatedPocCount, static_cast<unsigned long>(mitigatedPocs));

            if (!pocCandidates.first.empty() || pocCandidates.second || sic) {
                generateCollectivePerceptionMessage(now, pocCandidates.first, std::move(sic));
            }
        }
    }
    void CpmService::indicate(const vanetza::btp::DataIndication &ind, std::unique_ptr<vanetza::UpPacket> packet, const NetworkInterface& interface) {
        Enter_Method("indicate");

        Asn1PacketVisitor<vanetza::asn1::Cpm> visitor;
        const vanetza::asn1::Cpm *cpm = boost::apply_visitor(visitor, *packet);
        if (cpm && cpm->validate()) {
            CpmObject obj = visitor.shared_wrapper;
            emit(scSignalCpmReceived, &obj);
            mLocalDynamicMap->updateAwareness(obj);
        }
    }

    void CpmService::generateCollectivePerceptionMessage(const omnetpp::SimTime& now, PocCandidates& pocCandidates, SensorInformationContainer_ptr sic)
    {
        EV_DEBUG << "Creating CPM, candidates: " << pocCandidates.size() << ", sic: " << !!sic << "\n";

        auto numberOfPerceivedObjects = mLocalTracking.size();
        if (numberOfPerceivedObjects > de::numberOfPerceivedObjectsMax) {
            EV_ERROR << "Number of Perceived Objects exceeds maximum, capping by " << (numberOfPerceivedObjects - de::numberOfPerceivedObjectsMax) << "\n";
            numberOfPerceivedObjects = de::numberOfPerceivedObjectsMax;
        }
        if (pocCandidates.size() > de::numberOfPerceivedObjectsMax) {
            EV_ERROR << "Perceived Objects exceeding maximum size, removing " << (pocCandidates.size() - de::numberOfPerceivedObjectsMax) << " objects\n";
            pocCandidates.erase(pocCandidates.begin() + de::numberOfPerceivedObjectsMax, pocCandidates.end());
        }
        auto pos = createPerceivedObjects(pocCandidates);

        mLastCpm = now;
        if (sic) {
            mLastSensorInfoContainer = now;
        }

        // Optimize for the common case, try a single CPM first
        if (pos.size() <= df::perceivedObjectContainerMax) {
            auto cpm = createCollectivePerceptionMessage(numberOfPerceivedObjects, false);

            if (sic) {
                asn1_field_set(cpm->cpm.cpmParameters.sensorInformationContainer, sic);
            }
            if (!pos.empty()) {
                cpm->cpm.cpmParameters.perceivedObjectContainer = vanetza::asn1::allocate<PerceivedObjectContainer>();
                while (!asn1_sequence_push(cpm->cpm.cpmParameters.perceivedObjectContainer, pos)) {};
            }

            const auto size = cpm.size();
            if (size <= mMtuCpm) {
                if (opp::getEnvir()->isLoggingEnabled()) {
                    EV_DETAIL << "CPM transmitting, single\n";
                    EV_DETAIL << "Trns sender: " << mVehicleDataProvider->getStationId() << ", time: " << now.ustr()
                              << ", pos: " << mVehicleDataProvider->position().x << " - " << mVehicleDataProvider->position().y
                              << ", delta : " << mVehicleDataProvider->updated().ustr() << "\n";
                    if (opp::cLog::runtimeLogPredicate(this, opp::LOGLEVEL_DEBUG, nullptr)) {
                        asn_fprint(stdout, &asn_DEF_CPM, cpm.operator->());
                    }
                }
                emit(sc::cpmSent_channelBusyRatio, getCurrentCbr());
                emit(sc::cpmSent_segmentCount, static_cast<unsigned long>(1));
                emit(sc::cpmSent_pocIncluded, !pocCandidates.empty());

                std::string error;
                if (!cpm.validate(error)) {
                    throw opp::cRuntimeError("Invalid CPM constructed: %s", error.c_str());
                }

                emit(sc::cpmSent_msgSize, size);
                emit(
                        sc::cpmSent_poCountSegment,
                        static_cast<unsigned long>(cpm->cpm.cpmParameters.perceivedObjectContainer ? cpm->cpm.cpmParameters.perceivedObjectContainer->list.count : 0));
                emit(sc::cpmSent_sicIncludedInSegment, static_cast<unsigned long>(cpm->cpm.cpmParameters.sensorInformationContainer ? 1 : 0));
                emit(sc::cpmSent_sender, static_cast<unsigned long>(cpm->header.stationID));

                transmitCpm(std::move(cpm));
                updateLocalTracking(now, pocCandidates);

                return;
            }

            // Extract the sic and pos to use them for the segmented CPM case, otherwise they would get deleted when cpm goes out of scope
            if (cpm->cpm.cpmParameters.sensorInformationContainer) {
                asn1_field_get(cpm->cpm.cpmParameters.sensorInformationContainer, sic);
            }
            if (cpm->cpm.cpmParameters.perceivedObjectContainer) {
                while (!asn1_sequence_pop(cpm->cpm.cpmParameters.perceivedObjectContainer, pos)) {};
            }
        }

        // Segmented CPM case
        // Sort high priority elements to the end because the elements get processed from the end
        pos.sort([](const PerceivedObject& lhs, const PerceivedObject& rhs) {
            // According to ETSI TR 103 562 V2.1.1 ObjectConfidence_unknown is a valid value, since its value is zero
            // objects with this confidence get always priority value zero, their speed doesn't matter
            // WARNING: The IBR implementation stores the speed directly in the xSpeed and ySpeed variables
            const auto lPriority = (lhs.objectConfidence < ObjectConfidence_unavailable ? lhs.objectConfidence * lhs.xSpeed.value : lhs.xSpeed.value);
            const auto rPriority = (rhs.objectConfidence < ObjectConfidence_unavailable ? rhs.objectConfidence * rhs.xSpeed.value : rhs.xSpeed.value);
            return (lPriority < rPriority);
        });

        // If the MTU is too small to fit even a single Perceived Object this results in an infinite loop
        std::vector<vanetza::asn1::Cpm> cpms;
        cpms.reserve(8);
        while (!pos.empty()) {
            auto cpm = createCollectivePerceptionMessage(numberOfPerceivedObjects, true);

            cpm->cpm.cpmParameters.perceivedObjectContainer = vanetza::asn1::allocate<PerceivedObjectContainer>();
            for (int i = 0; i < df::perceivedObjectContainerMax && !pos.empty(); ++i) {
                asn1_sequence_push(cpm->cpm.cpmParameters.perceivedObjectContainer, pos);
                if (cpm.size() > mMtuCpm) {
                    asn1_sequence_pop(cpm->cpm.cpmParameters.perceivedObjectContainer, pos);

                    break;
                }
            }

            if (sic) {
                asn1_field_set(cpm->cpm.cpmParameters.sensorInformationContainer, sic);
                if (cpm.size() > mMtuCpm) {
                    asn1_field_get(cpm->cpm.cpmParameters.sensorInformationContainer, sic);
                }
            }

            cpms.emplace_back(std::move(cpm));
        }
        if (sic) {
            auto cpm = createCollectivePerceptionMessage(numberOfPerceivedObjects, true);

            asn1_field_set(cpm->cpm.cpmParameters.sensorInformationContainer, sic);

            cpms.emplace_back(std::move(cpm));
        }

        if (cpms.size() > de::segmentCountMax) {
            EV_ERROR << "Number of CPM segments exceeds maximum, capping by " << (cpms.size() - de::segmentCountMax) << "\n";
            cpms.erase(cpms.begin() + de::segmentCountMax, cpms.end());
        }
        for (size_t i = 0; i < cpms.size(); ++i) {
            auto& cpm = cpms[i];
            cpm->cpm.cpmParameters.managementContainer.perceivedObjectContainerSegmentInfo->totalMsgSegments = cpms.size();
            cpm->cpm.cpmParameters.managementContainer.perceivedObjectContainerSegmentInfo->thisSegmentNum = i + 1;
        }

        if (opp::getEnvir()->isLoggingEnabled()) {
            EV_DETAIL << "CPM transmitting, segments: " << cpms.size() << "\n";
            EV_DETAIL << "Trns sender: " << mVehicleDataProvider->getStationId() << ", time: " << now.ustr() << ", pos: " << mVehicleDataProvider->position().x
                      << " - " << mVehicleDataProvider->position().y << ", delta : " << mVehicleDataProvider->updated().ustr() << "\n";
        }
        emit(sc::cpmSent_channelBusyRatio, getCurrentCbr());
        emit(sc::cpmSent_segmentCount, static_cast<unsigned long>(cpms.size()));
        emit(sc::cpmSent_pocIncluded, !pocCandidates.empty());

        std::string error;
        for (auto& cpm : cpms) {
            const auto size = cpm.size();

            if (opp::cLog::runtimeLogPredicate(this, opp::LOGLEVEL_DEBUG, nullptr)) {
                asn_fprint(stdout, &asn_DEF_CPM, cpm.operator->());
            }
            if (!cpm.validate(error)) {
                throw opp::cRuntimeError("Invalid CPM constructed: %s", error.c_str());
            }

            emit(sc::cpmSent_msgSize, size);
            emit(
                    sc::cpmSent_poCountSegment,
                    static_cast<unsigned long>(cpm->cpm.cpmParameters.perceivedObjectContainer ? cpm->cpm.cpmParameters.perceivedObjectContainer->list.count : 0));
            if (cpm->cpm.cpmParameters.sensorInformationContainer) {
                emit(
                        sc::cpmSent_sicIncludedInSegment,
                        static_cast<unsigned long>(cpm->cpm.cpmParameters.managementContainer.perceivedObjectContainerSegmentInfo->thisSegmentNum));
            }
            emit(sc::cpmSent_sender, static_cast<unsigned long>(cpm->header.stationID));

            transmitCpm(std::move(cpm));
        }
        updateLocalTracking(now, pocCandidates);
    }


    void CpmService::updateLocalTracking(const omnetpp::SimTime& now, PocCandidates& pocCandidates)
    {
        EV_DEBUG << "Updating local tracking\n";

        for (auto& cand : pocCandidates) {
            const auto object = cand->first.lock();
            const auto& vehicle = object->getVehicleData();

            cand->second.setPrevDynamics(TrackedDynamics{now, vehicle.position(), vehicle.speed(), vehicle.heading()});
        }
    }


    vanetza::asn1::Cpm CpmService::createCollectivePerceptionMessage(NumberOfPerceivedObjects_t numberOfPerceivedObjects, bool segmented)
    {
        vanetza::asn1::Cpm cpm;

        // ITS PDU Header
        auto& hdr = cpm->header;
        hdr.protocolVersion = ItsPduHeader__protocolVersion_cpm;
        hdr.messageID = ItsPduHeader__messageID_cpm;
        hdr.stationID = mVehicleDataProvider->getStationId();

        // Basic fields
        // Downcast to 16 bit as cheap modulo operation
        cpm->cpm.generationDeltaTime = static_cast<uint16_t>(artery::countTaiMilliseconds(mTimer->getTimeFor(mVehicleDataProvider->updated())));
        cpm->cpm.cpmParameters.numberOfPerceivedObjects = numberOfPerceivedObjects;

        // Management Container
        auto& mgm = cpm->cpm.cpmParameters.managementContainer;
        // The StationType definition of the VehicleDataProvider is identical to the CPM definition
        mgm.stationType = static_cast<StationType_t>(mVehicleDataProvider->getStationType());

        if (segmented) {
            mgm.perceivedObjectContainerSegmentInfo = vanetza::asn1::allocate<PerceivedObjectContainerSegmentInfo>();
            // Use default values of a single segmented CPM in case this container gets also created for such a CPM,
            // this way the caller does not need to initialize the container
            mgm.perceivedObjectContainerSegmentInfo->totalMsgSegments = 1;
            mgm.perceivedObjectContainerSegmentInfo->thisSegmentNum = 1;
        }

        // Set the reference position like the CaService
        // It seems vehicles can leave the world and report a position out of the valid range
        const auto latitude = roundToUnit(mVehicleDataProvider->latitude(), units::microdecidegree);
        if (latitude >= de::latitudeMin && latitude <= de::latitudeMax) {
            mgm.referencePosition.latitude = latitude;
        } else {
            EV_ERROR << "Setting invalid latitude to unavailable: " << latitude << "\n";
            mgm.referencePosition.latitude = Latitude_unavailable;
        }
        const auto longitude = roundToUnit(mVehicleDataProvider->longitude(), units::microdecidegree);
        if (longitude >= de::longitudeMin && longitude <= de::longitudeMax) {
            mgm.referencePosition.longitude = longitude;
        } else {
            EV_ERROR << "Setting invalid longitude to unavailable: " << longitude << "\n";
            mgm.referencePosition.longitude = Longitude_unavailable;
        }
        mgm.referencePosition.positionConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
        mgm.referencePosition.positionConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;
        mgm.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
        mgm.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
        mgm.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;

        // Station Data Container
        // This implementation only supports vehicle type stations, set values like the CaService
        cpm->cpm.cpmParameters.stationDataContainer = vanetza::asn1::allocate<StationDataContainer>();
        cpm->cpm.cpmParameters.stationDataContainer->present = StationDataContainer_PR_originatingVehicleContainer;
        auto& vehicle = cpm->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer;

        // Mandatory fields
        vehicle.heading.headingValue = roundToUnit(mVehicleDataProvider->heading(), units::decidegree);
        vehicle.heading.headingConfidence = HeadingConfidence_equalOrWithinOneDegree;
        const auto speedValue = roundToUnit(abs(mVehicleDataProvider->speed()), units::centimeter_per_second);
        if (speedValue >= de::speedValueMin && speedValue <= de::speedValueMax) {
            vehicle.speed.speedValue = speedValue;
        } else {
            EV_ERROR << "Setting invalid speedValue to unavailable: " << speedValue << "\n";
            vehicle.speed.speedValue = SpeedValue_unavailable;
        }
        vehicle.speed.speedConfidence = roundToUnit(vanetza::units::Velocity(3.0 * units::centimeter_per_second), units::centimeter_per_second);
        vehicle.driveDirection = (mVehicleDataProvider->speed().value() >= 0.0 ? DriveDirection_forward : DriveDirection_backward);

        // Optional fields set from known data
        const auto longitudinalAccelerationValue = convertToUnit(mVehicleDataProvider->acceleration(), units::decimeter_per_second_squared);
        vehicle.longitudinalAcceleration = vanetza::asn1::allocate<LongitudinalAcceleration>();
        // Extreme speed changes can occur when SUMO swaps vehicles between lanes (speed is swapped as well)
        if (longitudinalAccelerationValue >= de::longitudinalAccelerationValueMin && longitudinalAccelerationValue <= de::longitudinalAccelerationValueMax) {
            vehicle.longitudinalAcceleration->longitudinalAccelerationValue = longitudinalAccelerationValue;
        } else {
            EV_ERROR << "Setting invalid longitudinalAccelerationValue to unavailable: " << longitudinalAccelerationValue << "\n";
            vehicle.longitudinalAcceleration->longitudinalAccelerationValue = LongitudinalAccelerationValue_unavailable;
        }
        vehicle.longitudinalAcceleration->longitudinalAccelerationConfidence = AccelerationConfidence_unavailable;
        const auto yawRateValue = roundToUnit(mVehicleDataProvider->yaw_rate(), units::centidegree_per_second);
        vehicle.yawRate = vanetza::asn1::allocate<YawRate>();
        if (yawRateValue >= de::yawRateValueMin && yawRateValue <= de::yawRateValueMax) {
            vehicle.yawRate->yawRateValue = yawRateValue;
        } else {
            EV_ERROR << "Setting invalid yawRateValue to unavailable: " << yawRateValue << "\n";
            vehicle.yawRate->yawRateValue = YawRateValue_unavailable;
        }
        vehicle.yawRate->yawRateConfidence = YawRateConfidence_unavailable;

        // Optional fields set by IBR implementation to increase message size to a more realistic value
        vehicle.vehicleOrientationAngle = vanetza::asn1::allocate<WGS84Angle>();
        vehicle.vehicleOrientationAngle->value = WGS84AngleValue_unavailable;
        vehicle.vehicleOrientationAngle->confidence = AngleConfidence_unavailable;
        vehicle.lateralAcceleration = vanetza::asn1::allocate<LateralAcceleration>();
        vehicle.lateralAcceleration->lateralAccelerationValue = LateralAccelerationValue_unavailable;
        vehicle.lateralAcceleration->lateralAccelerationConfidence = AccelerationConfidence_unavailable;

        return cpm;
    }

    SensorInformationContainer_ptr CpmService::createSensorInformationContainer()
    {
        auto sic = make_asn1<SensorInformationContainer>(asn_DEF_SensorInformationContainer);

        // WARNING: This is from the IBR implementation, a proper solution would be to extract the required information from the LEM
        auto info = make_asn1<SensorInformation>(asn_DEF_SensorInformation);
        info->sensorID = 1;
        info->type = SensorType_radar;
        info->detectionArea.present = DetectionArea_PR_vehicleSensor;
        info->detectionArea.choice.vehicleSensor.refPointId = 0;
        info->detectionArea.choice.vehicleSensor.xSensorOffset = XSensorOffset_negativeZeroPointZeroOneMeter;
        info->detectionArea.choice.vehicleSensor.ySensorOffset = YSensorOffset_zeroPointZeroOneMeter;

        auto prop = make_asn1<VehicleSensorProperties>(asn_DEF_VehicleSensorProperties);
        prop->range = Range_oneMeter;
        prop->horizontalOpeningAngleStart = CartesianAngleValue_oneDegree;
        prop->horizontalOpeningAngleEnd = CartesianAngleValue_oneDegree;
        asn1_sequence_add(&info->detectionArea.choice.vehicleSensor.vehicleSensorPropertyList, prop);

        asn1_sequence_add(sic.get(), info);

        return sic;
    }

    PerceivedObjects CpmService::createPerceivedObjects(const PocCandidates& pocCandidates)
    {
        PerceivedObjects pos;
        pos.reserve(pocCandidates.size());
        const auto& trackedObjects = mLocalEnvironmentModel->allObjects();
        for (const auto& cand : pocCandidates) {
            const auto object = cand->first.lock();
            const auto& vehicle = object->getVehicleData();
            // Local tracking data is synchronized with the LEM so find() will always return a valid result
            const auto trackingRange = aggregateTrackingTime(trackedObjects.find(cand->first)->second.sensors(), mLocalTrackingCategory);

            pos.push_back(vanetza::asn1::allocate<PerceivedObject>());
            auto& po = pos.back();

            po.objectID = vehicle.getStationId();

            // WARNING: This is from the IBR implementation, a proper solution would be to extract the required information from the LEM
            po.sensorIDList = vanetza::asn1::allocate<SensorIdList>();
            auto sensorId = make_asn1<Identifier_t>(asn_DEF_Identifier);
            *sensorId = 1;
            asn1_sequence_add(po.sensorIDList, sensorId);

            auto timeOfMeasurement = (mVehicleDataProvider->updated() - trackingRange.second).inUnit(opp::SIMTIME_MS);
            if (timeOfMeasurement < de::timeOfMeasurementMin) {
                EV_ERROR << "Clamping timeOfMeasurement to minimum: " << timeOfMeasurement << "\n";
                timeOfMeasurement = de::timeOfMeasurementMin;
            } else if (timeOfMeasurement > de::timeOfMeasurementMax) {
                EV_ERROR << "Clamping timeOfMeasurement to maximum: " << timeOfMeasurement << "\n";
                timeOfMeasurement = de::timeOfMeasurementMax;
            }
            po.timeOfMeasurement = timeOfMeasurement;

            auto objectAge = (trackingRange.second - trackingRange.first).inUnit(opp::SIMTIME_MS);
            if (objectAge < de::objectAgeMin) {
                EV_WARN << "Clamping objectAge to minimum: " << objectAge << "\n";
                objectAge = de::objectAgeMin;
            } else if (objectAge > de::objectAgeMax) {
                EV_INFO << "Clamping objectAge to maximum: " << objectAge << "\n";
                objectAge = de::objectAgeMax;
            }
            po.objectAge = vanetza::asn1::allocate<ObjectAge_t>();
            *po.objectAge = objectAge;

            // INFO: Currently hardcoded confidence
            // Don't use ObjectConfidence_unknown or during the calculation of priorities for segmented CPM's
            // all objects will get priority zero. Using ObjectConfidence_onePercent will result in the same priority values
            // of the IBR implementation.
            po.objectConfidence = ObjectConfidence::ObjectConfidence_onePercent;

            auto xDistanceValue = roundToUnit(vehicle.position().x - mVehicleDataProvider->position().x, units::centimeter);
            if (xDistanceValue < de::distanceValueMin) {
                EV_ERROR << "Clamping xDistanceValue to minimum: " << xDistanceValue;
                xDistanceValue = de::distanceValueMin;
            } else if (xDistanceValue > de::distanceValueMax) {
                EV_ERROR << "Clamping xDistanceValue to maximum: " << xDistanceValue;
                xDistanceValue = de::distanceValueMax;
            }
            po.xDistance.value = xDistanceValue;
            po.xDistance.confidence = DistanceConfidence_zeroPointZeroOneMeter;
            auto yDistanceValue = roundToUnit(vehicle.position().y - mVehicleDataProvider->position().y, units::centimeter);
            if (yDistanceValue < de::distanceValueMin) {
                EV_ERROR << "Clamping yDistanceValue to minimum: " << yDistanceValue;
                yDistanceValue = de::distanceValueMin;
            } else if (yDistanceValue > de::distanceValueMax) {
                EV_ERROR << "Clamping yDistanceValue to maximum: " << yDistanceValue;
                yDistanceValue = de::distanceValueMax;
            }
            po.yDistance.value = yDistanceValue;
            po.yDistance.confidence = DistanceConfidence_zeroPointZeroOneMeter;

            // WARNING: This is from the IBR implementation, a proper solution would be to decompose the speed into x- and y- direction
            //          and specify it relative to the ego vehicle speed
            const auto speedValue = roundToUnit(vehicle.speed(), units::centimeter_per_second);
            if (speedValue >= de::speedValueExtendedMin && speedValue <= de::speedValueExtendedMax) {
                po.xSpeed.value = speedValue;
                po.ySpeed.value = speedValue;
            } else {
                EV_ERROR << "Setting invalid speedValueExtended to unavailable: " << speedValue << "\n";
                po.xSpeed.value = SpeedValueExtended_unavailable;
                po.ySpeed.value = SpeedValueExtended_unavailable;
            }
            po.xSpeed.confidence = SpeedConfidence_equalOrWithinOneCentimeterPerSec;
            po.ySpeed.confidence = SpeedConfidence_equalOrWithinOneCentimeterPerSec;

            // INFO: Currently hardcoded values
            po.objectRefPoint = ObjectRefPoint_mid;
            po.dynamicStatus = vanetza::asn1::allocate<DynamicStatus_t>();
            *po.dynamicStatus = DynamicStatus_dynamic;

            // INFO: Currently hardcoded confidence
            const auto objectClass = getObjectClass(vehicle.getStationType());
            if (objectClass.first != ObjectClass__class_PR_NOTHING) {
                po.classification = vanetza::asn1::allocate<ObjectClassDescription>();
                auto oc = make_asn1<ObjectClass>(asn_DEF_ObjectClass);
                oc->confidence = ClassConfidence_oneHundredPercent;
                oc->Class.present = objectClass.first;
                switch (oc->Class.present) {
                    case ObjectClass__class_PR_vehicle:
                        oc->Class.choice.vehicle.type = objectClass.second;
                        oc->Class.choice.vehicle.confidence = ClassConfidence_oneHundredPercent;
                        break;
                    case ObjectClass__class_PR_person:
                        oc->Class.choice.person.type = objectClass.second;
                        oc->Class.choice.person.confidence = ClassConfidence_oneHundredPercent;
                        break;
                    case ObjectClass__class_PR_animal:
                        oc->Class.choice.animal.type = objectClass.second;
                        oc->Class.choice.animal.confidence = ClassConfidence_oneHundredPercent;
                        break;
                    case ObjectClass__class_PR_other:
                        oc->Class.choice.other.type = objectClass.second;
                        oc->Class.choice.other.confidence = ClassConfidence_oneHundredPercent;
                        break;
                }
                asn1_sequence_add(po.classification, oc);
            }

            // Optional fields set by IBR implementation to increase message size to a more realistic value
            po.yawAngle = vanetza::asn1::allocate<CartesianAngle>();
            po.yawAngle->value = CartesianAngleValue_unavailable;
            po.yawAngle->confidence = AngleConfidence_unavailable;
            po.planarObjectDimension1 = vanetza::asn1::allocate<ObjectDimension>();
            po.planarObjectDimension1->value = ObjectDimensionValue_oneMeter;
            po.planarObjectDimension1->confidence = ObjectDimensionConfidence_oneMeter;
            po.planarObjectDimension2 = vanetza::asn1::allocate<ObjectDimension>();
            po.planarObjectDimension2->value = ObjectDimensionValue_oneMeter;
            po.planarObjectDimension2->confidence = ObjectDimensionConfidence_oneMeter;
        }

        return pos;
    }


    void CpmService::transmitCpm(vanetza::asn1::Cpm&& cpm)
    {
        vanetza::btp::DataRequestB request;
        auto port = getPortNumber(mPrimaryChannel);
        request.destination_port = vanetza::host_cast<artery::PortNumber>(port);
        request.gn.its_aid = vanetza::aid::CP;
        request.gn.transport_type = vanetza::geonet::TransportType::SHB;
        request.gn.maximum_lifetime = vanetza::geonet::Lifetime{vanetza::geonet::Lifetime::Base::One_Second, 1};
        request.gn.communication_profile = vanetza::geonet::CommunicationProfile::ITS_G5;

        // This is the reverse from what is done during initialize()
        request.gn.traffic_class.tc_id(static_cast<unsigned int>(mDccProfile));

        CpObject obj(std::move(cpm));
        emit(evt::cpmSent, &obj);

        using CpmByteBuffer = vanetza::convertible::byte_buffer_impl<vanetza::asn1::Cpm>;
        std::unique_ptr<vanetza::geonet::DownPacket> payload{new vanetza::geonet::DownPacket()};
        std::unique_ptr<vanetza::convertible::byte_buffer> buffer{new CpmByteBuffer(obj.shared_ptr())};
        payload->layer(vanetza::OsiLayer::Application) = std::move(buffer);
        this->request(request, std::move(payload));
    }


    double CpmService::getCurrentCbr() const
    {
        auto ifc = mNetworkInterfaceTable->select(mPrimaryChannel);
        if (!ifc) {
            throw opp::cRuntimeError("No NetworkInterface found for CP's primary channel %i", mPrimaryChannel);
        }
        auto* scpp = dynamic_cast<vanetza::dcc::SmoothingChannelProbeProcessor*>(ifc->getDccEntity().getChannelProbeProcessor());
        if (!scpp) {
            throw opp::cRuntimeError("No SmoothingChannelProbeProcessor found for CP's primary channel %i", mPrimaryChannel);
        }

        return scpp->channel_load().value();
    }

    omnetpp::SimTime CpmService::generateCpmDcc()
    {
        // This is copied from CaService
        auto ifc = mNetworkInterfaceTable->select(mPrimaryChannel);
        auto* trc = (ifc ? ifc->getDccEntity().getTransmitRateThrottle() : nullptr);
        if (!trc) {
            throw opp::cRuntimeError("No DCC TRC found for CP's primary channel %i", mPrimaryChannel);
        }

        static const vanetza::dcc::TransmissionLite ca_tx(mDccProfile, 0);
        auto interval = trc->interval(ca_tx);
        opp::SimTime dcc(std::chrono::duration_cast<std::chrono::milliseconds>(interval).count(), opp::SIMTIME_MS);

        return std::min(mGenCpmMax, std::max(mGenCpmMin, dcc));
    }


    void CpmService::emitRemoteObjectStatistics(
            const RemoteDynamics& object, const boost::optional<CpmDynamics> prevDynamics, const CpmDynamics& currentDynamics)
    {
        if (prevDynamics) {
            emit(sc::cpmRecv_timeSinceLastObjectUpdate, static_cast<long>((currentDynamics.received - prevDynamics->received).inUnit(opp::SIMTIME_MS)));
            emit(
                    sc::cpmRecv_distanceSinceLastObjectUpdate,
                    convertToUnit(artery::distance(currentDynamics.trackedDynamics.position, prevDynamics->trackedDynamics.position), vanetza::units::si::meter));
        }
        emit(sc::cpmRecv_senderRedundancy, static_cast<unsigned long>(object.getCpmDynamics().size()));
        emit(sc::cpmRecv_poRedundancy, static_cast<unsigned long>(object.getCpmDynamicsCount()));
    }

}  // namespace artery

} // namespace artery
