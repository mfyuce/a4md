//
// Created by bastian on 06.07.21.
//

#include "artery/application/BaseCpmService.h"
#include "artery/application/misbehavior/util/HelperFunctions.h"
#include "artery/utility/simtime_cast.h"
#include <artery/traci/Cast.h>
#include <vanetza/facilities/cam_functions.hpp>
#include <vanetza/btp/ports.hpp>
#include <vanetza/dcc/transmission.hpp>
#include <vanetza/dcc/transmit_rate_control.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <boost/units/cmath.hpp>
#include <chrono>

namespace artery {

    using namespace omnetpp;

    Define_Module(BaseCpmService)

    namespace {
        auto microdegree = vanetza::units::degree * boost::units::si::micro;
        auto decidegree = vanetza::units::degree * boost::units::si::deci;
        auto degree_per_second = vanetza::units::degree / vanetza::units::si::second;
        auto centimeter_per_second = vanetza::units::si::meter_per_second * boost::units::si::centi;

        static const simsignal_t scSignalCpmSent = cComponent::registerSignal("CpmSent");
    }

    bool BaseCpmService::staticInitializationComplete = false;
    std::shared_ptr<const traci::API> BaseCpmService::mTraciAPI;
    traci::Boundary BaseCpmService::mSimulationBoundary;
    std::chrono::milliseconds BaseCpmService::scLowFrequencyContainerInterval;

    template<typename T, typename U>
    long BaseCpmService::round(const boost::units::quantity<T> &q, const U &u) {
        boost::units::quantity<U> v{q};
        return std::round(v.value());
    }

    SpeedValue_t BaseCpmService::buildSpeedValue(const vanetza::units::Velocity &v) {
        static const vanetza::units::Velocity lower{0.0 * boost::units::si::meter_per_second};
        static const vanetza::units::Velocity upper{163.82 * boost::units::si::meter_per_second};

        SpeedValue_t speed = SpeedValue_unavailable;
        if (v >= upper) {
            speed = 16382; // see CDD A.74 (TS 102 894 v1.2.1)
        } else if (v >= lower) {
            speed = round(v, centimeter_per_second) * SpeedValue_oneCentimeterPerSec;
        }
        return speed;
    }

    BaseCpmService::BaseCpmService() : mGenCpmMin{100, SIMTIME_MS},
                                     mGenCpmMax{1000, SIMTIME_MS},
                                     mGenCpm(mGenCpmMax),
                                     mGenCpmLowDynamicsCounter(0),
                                     mGenCpmLowDynamicsLimit(3) {
    }

    void BaseCpmService::initialize() {
        ItsG5BaseService::initialize();

        if (!staticInitializationComplete) {
            staticInitializationComplete = true;
            mTraciAPI = getFacilities().get_const<traci::VehicleController>().getTraCI();
            mSimulationBoundary = traci::Boundary{mTraciAPI->simulation.getNetBoundary()};
            scLowFrequencyContainerInterval = std::chrono::milliseconds(500);
        }

        mNetworkInterfaceTable = &getFacilities().get_const<NetworkInterfaceTable>();
        mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
        mTimer = &getFacilities().get_const<Timer>();
        mLocalDynamicMap = &getFacilities().get_mutable<artery::LocalDynamicMap>();
        mVehicleController = &getFacilities().get_const<traci::VehicleController>();

        mVehicleLength = (long) (mVehicleController->getLength().value() * 10);
        mVehicleWidth = (long) (mVehicleController->getWidth().value() * 10);
        mStationId = mVehicleDataProvider->station_id();
        WATCH(mStationId);

        // avoid unreasonable high elapsed time values for newly inserted vehicles
        mLastCpmTimestamp = simTime();

        // first generated CPM shall include the low frequency container
        mLastLowCpmTimestamp = mLastCpmTimestamp - artery::simtime_cast(scLowFrequencyContainerInterval);

        // generation rate boundaries
        mGenCpmMin = par("minInterval");
        mGenCpmMax = par("maxInterval");
        mGenCpm = mGenCpmMax;

        // vehicle dynamics thresholds
        mHeadingDelta = vanetza::units::Angle{par("headingDelta").doubleValue() * vanetza::units::degree};
        mPositionDelta = par("positionDelta").doubleValue() * vanetza::units::si::meter;
        mSpeedDelta = par("speedDelta").doubleValue() * vanetza::units::si::meter_per_second;

        mDccRestriction = par("withDccRestriction");
        mFixedRate = par("fixedRate");

        // look up primary channel for CA
        mPrimaryChannel = getFacilities().get_const<MultiChannelPolicy>().primaryChannel(vanetza::aid::CA);
    }

    bool BaseCpmService::checkTriggeringConditions(const SimTime &T_now) {
        // provide variables named like in EN 302 637-2 V1.3.2 (section 6.1.3)
        SimTime &T_GenCpm = mGenCpm;
        const SimTime &T_GenCpmMin = mGenCpmMin;
        const SimTime &T_GenCpmMax = mGenCpmMax;
        const SimTime T_GenCpmDcc = mDccRestriction ? genCpmDcc() : T_GenCpmMin;
        const SimTime T_elapsed = T_now - mLastCpmTimestamp;

        bool trigger = false;
        if (T_elapsed >= T_GenCpmDcc) {
            if (mFixedRate) {
                trigger = true;
            } else if (checkHeadingDelta() || checkPositionDelta() || checkSpeedDelta()) {
                trigger = true;
                T_GenCpm = std::min(T_elapsed, T_GenCpmMax); /*< if middleware update interval is too long */
                mGenCpmLowDynamicsCounter = 0;
            } else if (T_elapsed >= T_GenCpm) {
                trigger = true;
                if (++mGenCpmLowDynamicsCounter >= mGenCpmLowDynamicsLimit) {
                    T_GenCpm = T_GenCpmMax;
                }
            }
        }
        return trigger;
    }

    bool BaseCpmService::checkHeadingDelta() const {
        return !vanetza::facilities::similar_heading(mLastCpmHeading, mVehicleDataProvider->heading(), mHeadingDelta);
    }

    bool BaseCpmService::checkPositionDelta() const {
        return (distance(mLastCpmPosition, mVehicleDataProvider->position()) > mPositionDelta);
    }

    bool BaseCpmService::checkSpeedDelta() const {
        return abs(mLastCpmSpeed - mVehicleDataProvider->speed()) > mSpeedDelta;
    }

    SimTime BaseCpmService::genCpmDcc() {
        // network interface may not be ready yet during initialization, so look it up at this later point
        auto networkInterfaceChannel = mNetworkInterfaceTable->select(mPrimaryChannel);
        vanetza::dcc::TransmitRateThrottle *trc = networkInterfaceChannel
                                                  ? networkInterfaceChannel->getDccEntity().getTransmitRateThrottle()
                                                  : nullptr;
        if (!trc) {
            throw cRuntimeError("No DCC TRC found for CA's primary channel %i", mPrimaryChannel);
        }

        static const vanetza::dcc::TransmissionLite ca_tx(vanetza::dcc::Profile::DP2, 0);
        vanetza::Clock::duration interval = trc->interval(ca_tx);
        SimTime dcc{std::chrono::duration_cast<std::chrono::milliseconds>(interval).count(), SIMTIME_MS};
        return std::min(mGenCpmMax, std::max(mGenCpmMin, dcc));
    }

    void BaseCpmService::finalizeAndSendCpm(vanetza::asn1::Cpm cpm, const SimTime &T_now) {
        mLastCpmPosition = mVehicleDataProvider->position();
        mLastCpmSpeed = mVehicleDataProvider->speed();
        mLastCpmHeading = mVehicleDataProvider->heading();
        mLastCpmTimestamp = T_now;
        if (T_now - mLastLowCpmTimestamp >= artery::simtime_cast(scLowFrequencyContainerInterval)) {
            mLastLowCpmTimestamp = T_now;
        }

        using namespace vanetza;
        btp::DataRequestB request;
        request.destination_port = btp::ports::CPM;
        request.gn.its_aid = aid::CA;
        request.gn.transport_type = geonet::TransportType::SHB;
        request.gn.maximum_lifetime = geonet::Lifetime{geonet::Lifetime::Base::One_Second, 1};
        request.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
        request.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

        CpmObject obj(std::move(cpm));
        emit(scSignalCpmSent, &obj);

        using CpmByteBuffer = convertible::byte_buffer_impl<asn1::Cpm>;
        std::unique_ptr<geonet::DownPacket> payload{new geonet::DownPacket()};
        std::unique_ptr<convertible::byte_buffer> buffer{new CpmByteBuffer(obj.shared_ptr())};
        payload->layer(OsiLayer::Application) = std::move(buffer);
        this->request(request, std::move(payload));
    }

    vanetza::asn1::Cpm
    BaseCpmService::createCollectivePerceptionMessage(uint16_t genDeltaTime) {
        vanetza::asn1::Cpm cpm;

        auto object = vanetza::asn1::allocate<PerceivedObject_t>(); //this fill the struct PerceivedObject with basic info
//        auto perception = vanetza::asn1::allocate<CollectivePerceptionMessage>();

        PerceivedObject *arrayobjeto[2];

//        cpm->cpm.cpmParameters.perceptionData = vanetza::asn1::allocate<CpmParameters::CpmParameters__perceptionData>();

        cpm->header.protocolVersion=2;
        cpm->header.messageID=ItsPduHeader__messageID_cpm;
        cpm->header.stationID=1;
        cpm->cpm.cpmParameters.managementContainer.stationType=2;

        for(int i=0; i<2;i++) {
            arrayobjeto[i]=vanetza::asn1::allocate<PerceivedObject>();
            arrayobjeto[i]->objectID= i+1;
            arrayobjeto[i]->timeOfMeasurement = TimeOfMeasurement_oneMilliSecond;
            arrayobjeto[i]->xDistance.value = DistanceValue_oneMeter;
            arrayobjeto[i]->xDistance.confidence = DistanceConfidence_oneMeter;
            arrayobjeto[i]->yDistance.value = DistanceValue_oneMeter;
            arrayobjeto[i]->yDistance.confidence = DistanceConfidence_oneMeter;
            arrayobjeto[i]->xSpeed.value = SpeedValueExtended_oneCentimeterPerSec;
            arrayobjeto[i]->xSpeed.confidence = SpeedConfidence_equalOrWithinOneMeterPerSec;
            arrayobjeto[i]->ySpeed.value = SpeedValueExtended_oneCentimeterPerSec;
            arrayobjeto[i]->ySpeed.confidence = SpeedConfidence_equalOrWithinOneMeterPerSec;
        }
//        perception->containerId=1;
//        perception->containerData.present=PerceptionData__containerData_PR_PerceivedObjectContainer;
//        perception->containerData.choice.PerceivedObjectContainer.numberOfPerceivedObjects=1;

        for(int i=0; i<2;i++) {
           ASN_SEQUENCE_ADD(&cpm->cpm.cpmParameters.perceivedObjectContainer,arrayobjeto[i]);
        }
//        EXPECT_EQ(0, ASN_SEQUENCE_ADD(&cpm->cpm.cpmParameters.perceptionData->list, perception));
//        EXPECT_EQ(1, cpm->cpm.cpmParameters.perceptionData->list.count);
//        EXPECT_EQ(2, cpm->cpm.cpmParameters.perceptionData->list.array[0]->containerData.choice.PerceivedObjectContainer.perceivedObjects.list.count);
//        EXPECT_EQ(perception, cpm->cpm.cpmParameters.perceptionData->list.array[0]);
//        EXPECT_EQ(arrayobjeto[0], cpm->cpm.cpmParameters.perceptionData->list.array[0]->containerData.choice.PerceivedObjectContainer.perceivedObjects.list.array[0]);


//        EXPECT_TRUE(cpm.validate());
//        EXPECT_FALSE(cpm.encode().empty());
        vanetza::ByteBuffer buffer = cpm.encode();
        std::cout << "tamaño: " << buffer.size() << "\n";
        for (const auto byte:buffer){
            printf("%02x ",byte);
        }

        auto a = cpm.decode(buffer);
        std::cout << cpm.size();





//        vanetza::asn1::Cpm message;
//
//        ItsPduHeader_t &header = (*message).header;
//        header.protocolVersion = 2;
//        header.messageID = ItsPduHeader__messageID_cpm;
//        header.stationID = mVehicleDataProvider->station_id();
//
//        CollectivePerceptionMessage_t &cpm = (*message).cpm;
//        cpm.generationDeltaTime = genDeltaTime * GenerationDeltaTime_oneMilliSec;
//        PerceivedObjectContainer_t *po = cpm.cpmParameters.perceivedObjectContainer;
//        po->list = malloc()
//        basic.stationType = StationType_passengerCar;
//        basic.referencePosition = mVehicleDataProvider->approximateReferencePosition();
//
//        HighFrequencyContainer_t &hfc = cpm.cpmParameters.highFrequencyContainer;
//        hfc.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;
//        BasicVehicleContainerHighFrequency &bvc = hfc.choice.basicVehicleContainerHighFrequency;
//        bvc.heading = mVehicleDataProvider->approximateHeading();
//        bvc.speed = mVehicleDataProvider->approximateSpeed();
//        bvc.driveDirection = mVehicleDataProvider->speed().value() >= 0.0 ?
//                             DriveDirection_forward : DriveDirection_backward;
//        bvc.longitudinalAcceleration = mVehicleDataProvider->approximateAcceleration();
//        bvc.curvature.curvatureValue =
//                abs(mVehicleDataProvider->curvature() / vanetza::units::reciprocal_metre) * 10000.0;
//        if (bvc.curvature.curvatureValue >= 1023) {
//            bvc.curvature.curvatureValue = 1023;
//        }
//        bvc.curvature.curvatureConfidence = CurvatureConfidence_unavailable;
//        bvc.curvatureCalculationMode = CurvatureCalculationMode_yawRateUsed;
//        bvc.yawRate.yawRateValue =
//                round(mVehicleDataProvider->yaw_rate(), degree_per_second) * YawRateValue_degSec_000_01ToLeft * 100.0;
//        if (bvc.yawRate.yawRateValue < -32766 || bvc.yawRate.yawRateValue > 32766) {
//            bvc.yawRate.yawRateValue = YawRateValue_unavailable;
//        }
//        bvc.vehicleLength.vehicleLengthValue = mVehicleLength;
//        bvc.vehicleLength.vehicleLengthConfidenceIndication =
//                VehicleLengthConfidenceIndication_noTrailerPresent;
//        bvc.vehicleWidth = mVehicleWidth;

        std::string error;
        if (!cpm.validate(error)) {
            throw cRuntimeError("Invalid High Frequency Cpm: %s", error.c_str());
        }

//        const vanetza::units::Duration delta{
//                (simTime()).inUnit(SIMTIME_MS) * boost::units::si::milli * boost::units::si::seconds
//        };
//
//        if (delta >= 1721.974 * boost::units::si::seconds) {
//            std::cout << "hi" << std::endl;
//        }

        return cpm;
    }

} // namespace artery