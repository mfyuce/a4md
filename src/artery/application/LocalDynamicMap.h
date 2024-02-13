#ifndef ARTERY_LOCALDYNAMICMAP_H_AL7SS9KT
#define ARTERY_LOCALDYNAMICMAP_H_AL7SS9KT

#include "artery/application/CaObject.h"
#include "CpmObject.h"
#include <omnetpp/simtime.h>
#include <vanetza/asn1/cam.hpp>
#include <cstdint>
#include <functional>
#include <map>

namespace artery
{

class Timer;

class LocalDynamicMap
{
public:
    using StationID = uint32_t;
    using Cam = vanetza::asn1::Cam;
    using Cpm = vanetza::asn1::Cpm;
    using CamPredicate = std::function<bool(const Cam&)>;
    using CpmPredicate = std::function<bool(const Cpm&)>;

    LocalDynamicMap(const Timer&);
    void updateAwareness(const CaObject&);
    void updateAwareness(const CpmObject&);
    void dropExpired();
    unsigned count(const CamPredicate&) const;
    unsigned countCpm(const CpmPredicate&) const;

private:
    struct AwarenessEntry
    {
        AwarenessEntry(const CaObject&, omnetpp::SimTime);
        AwarenessEntry(AwarenessEntry&&) = default;
        AwarenessEntry& operator=(AwarenessEntry&&) = default;

        omnetpp::SimTime expiry;
        CaObject object;
    };

    struct AwarenessCpmEntry
    {
        AwarenessCpmEntry(const CpmObject&, omnetpp::SimTime);
        AwarenessCpmEntry(AwarenessCpmEntry&&) = default;
        AwarenessCpmEntry& operator=(AwarenessCpmEntry&&) = default;

        omnetpp::SimTime expiry;
        CpmObject object;
    };

    const Timer& mTimer;
    std::map<StationID, AwarenessEntry> mCaMessages;
    std::map<StationID, AwarenessCpmEntry> mCpmMessages;
};

} // namespace artery

#endif /* ARTERY_LOCALDYNAMICMAP_H_AL7SS9KT */

