//
// Created by bastian on 06.07.21.
//

#ifndef ARTERY_BASECPMSERVICE_H
#define ARTERY_BASECPMSERVICE_H

#include "artery/application/ItsG5BaseService.h"
#include "artery/application/VehicleDataProvider.h"
#include <artery/traci/VehicleController.h>
#include <vanetza/asn1/cpm.hpp>
#include <vanetza/asn1/its/StationDataContainer.h>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include <omnetpp.h>


namespace artery {

    class NetworkInterfaceTable;

    class Timer;

    class VehicleDataProvider;

    class BaseCpmService : public ItsG5BaseService {

    public:
        BaseCpmService();

        void initialize() override;

    protected:

        template<typename T, typename U>
        static long round(const boost::units::quantity<T> &q, const U &u);

        static SpeedValue_t buildSpeedValue(const vanetza::units::Velocity &v);

        bool checkTriggeringConditions(const omnetpp::SimTime &);

        bool checkHeadingDelta() const;

        bool checkPositionDelta() const;

        bool checkSpeedDelta() const;

        omnetpp::SimTime genCpmDcc();

        void finalizeAndSendCpm(vanetza::asn1::Cpm cpm,const omnetpp::SimTime &T_now);

        vanetza::asn1::Cpm
        createCollectivePerceptionMessage(uint16_t genDeltaTime);

        ChannelNumber mPrimaryChannel = channel::CCH;
        const NetworkInterfaceTable *mNetworkInterfaceTable = nullptr;
        const VehicleDataProvider *mVehicleDataProvider = nullptr;
        const Timer *mTimer = nullptr;
        LocalDynamicMap *mLocalDynamicMap = nullptr;
        const traci::VehicleController *mVehicleController = nullptr;


        omnetpp::SimTime mGenCpmMin;
        omnetpp::SimTime mGenCpmMax;
        omnetpp::SimTime mGenCpm;
        unsigned mGenCpmLowDynamicsCounter;
        unsigned mGenCpmLowDynamicsLimit;
        Position mLastCpmPosition;
        vanetza::units::Velocity mLastCpmSpeed;
        vanetza::units::Angle mLastCpmHeading;
        omnetpp::SimTime mLastCpmTimestamp;
        omnetpp::SimTime mLastLowCpmTimestamp;
        vanetza::units::Angle mHeadingDelta;
        vanetza::units::Length mPositionDelta;
        vanetza::units::Velocity mSpeedDelta;
        VehicleWidth_t mVehicleWidth;
        VehicleLengthValue_t mVehicleLength;

        bool mDccRestriction;
        bool mFixedRate;
        long mStationId;

        static bool staticInitializationComplete;
        static std::shared_ptr<const traci::API> mTraciAPI;
        static traci::Boundary mSimulationBoundary;
        static std::chrono::milliseconds scLowFrequencyContainerInterval;
    };


} // namespace artery
#endif //ARTERY_BASECPMSERVICE_H
