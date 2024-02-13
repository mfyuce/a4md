/*
* Artery V2X Simulation Framework
* Copyright 2014-2019 Raphael Riebl et al.
* Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
*/

#ifndef ARTERY_MISBEHAVIORCPMSERVICE_H_
#define ARTERY_MISBEHAVIORCPMSERVICE_H_

#include "artery/application/BaseCpmService.h"
#include "artery/application/misbehavior/util/F2MDParameters.h"
#include "artery/application/misbehavior/util/MisbehaviorTypes.h"
#include "artery/application/misbehavior/util/AttackTypes.h"
#include "artery/envmod/LocalEnvironmentModel.h"
#include "artery/envmod/GlobalEnvironmentModel.h"
#include "artery/utility/Channel.h"
#include "artery/utility/Geometry.h"
#include <vanetza/btp/data_interface.hpp>
#include <map>

namespace artery {

    class NetworkInterfaceTable;

    class Timer;

    class VehicleDataProvider;

    class MisbehaviorCpmService : public BaseCpmService {
    public:
        template<typename T, typename U>
        static long round(const boost::units::quantity<T> &q, const U &u);

        MisbehaviorCpmService() = default;

        ~MisbehaviorCpmService() override;

        void initialize() override;

        void indicate(const vanetza::btp::DataIndication &, std::unique_ptr<vanetza::UpPacket>) override;

        void trigger() override;

        StationID_t getStationId() { return mVehicleDataProvider->getStationId(); };

        misbehaviorTypes::MisbehaviorTypes getMisbehaviorType() { return mMisbehaviorType; };

        attackTypes::AttackTypes getAttackType() { return mAttackType; };

    private:
        void sendCpm(const omnetpp::SimTime &);

        vanetza::asn1::Cpm createAttackCPM(uint16_t genDeltaTime);

        void createFakeReport();

        void initializeStaticParameters();

        void visualizeCpmPosition(vanetza::asn1::Cpm cpm);

        vanetza::asn1::Cpm getNextReplayCpm();

        static bool staticInitializationComplete;
        static GlobalEnvironmentModel *mGlobalEnvironmentModel;

        const LocalEnvironmentModel *mLocalEnvironmentModel = nullptr;

        misbehaviorTypes::MisbehaviorTypes mMisbehaviorType;
        attackTypes::AttackTypes mAttackType;
        std::vector<uint32_t> mPseudonyms;
        int mPseudonymIndex = 0;
        std::list<vanetza::asn1::Cpm> disruptiveMessageQueue;
        std::queue<vanetza::asn1::Cpm> staleMessageQueue;
        std::map<uint32_t, std::deque<vanetza::asn1::Cpm>> receivedMessages;
        std::list<std::string> activePoIs;

        long AttackConstantPositionLatitudeMicrodegrees;
        long AttackConstantPositionLongitudeMicrodegrees;
        long AttackConstantPositionOffsetLatitudeMicrodegrees;
        long AttackConstantPositionOffsetLongitudeMicrodegrees;
        long AttackConstantSpeedValue;
        vanetza::units::Velocity attackConstantSpeedOffsetValue;
        bool attackEventualStopHasStopped;
        ReferencePosition_t attackEventualStopPosition;
        int64_t attackDataReplayCurrentStationId;
        int attackGridSybilVehicleCount;
        double attackGridSybilActualDistanceX;
        double attackGridSybilActualDistanceY;
        int attackGridSybilCurrentVehicleIndex;
        double attackGridSybilLastHeadingAngle;
        uint64_t attackFakeReportLastReportTime;

        std::string lastPoiId;


    };
} // namespace artery

#endif /* ARTERY_MISBEHAVIORCPMSERVICE_H_ */
