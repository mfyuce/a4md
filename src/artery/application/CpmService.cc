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

    void CpService::trigger()
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

    void CpmService::sendCpm(const SimTime &T_now) {
        uint16_t genDeltaTimeMod = countTaiMilliseconds(mTimer->getTimeFor(mVehicleDataProvider->updated()));
        auto cpm = createCollectivePerceptionMessage(genDeltaTimeMod);
        finalizeAndSendCpm(cpm, T_now);
    }

} // namespace artery
