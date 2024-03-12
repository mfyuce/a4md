#ifndef ARTERY_LOCALDYNAMICMAP_H_AL7SS9KT
#define ARTERY_LOCALDYNAMICMAP_H_AL7SS9KT

#include "artery/application/CaObject.h"
#include "CpmObject.h"
#include <omnetpp/simtime.h>
#include <vanetza/asn1/cam.hpp>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>

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

    class AwarenessEntry
    {
    public:
        AwarenessEntry(const CaObject&, omnetpp::SimTime);
        AwarenessEntry(AwarenessEntry&&) = default;
        AwarenessEntry& operator=(AwarenessEntry&&) = default;

        omnetpp::SimTime expiry() const { return mExpiry; }
        const Cam& cam() const { return mObject.asn1(); }
        std::shared_ptr<const Cam> camPtr() const { return mObject.shared_ptr(); }

    private:
        omnetpp::SimTime mExpiry;
        CaObject mObject;
    };

    class AwarenessCpmEntry
    {
    public:
        AwarenessCpmEntry(const CaObject&, omnetpp::SimTime);
        AwarenessCpmEntry(AwarenessCpmEntry&&) = default;
        AwarenessCpmEntry& operator=(AwarenessCpmEntry&&) = default;

        omnetpp::SimTime expiry() const { return mExpiry; }
        const Cpm& cpm() const { return mObject.asn1(); }
        std::shared_ptr<const Cpm> cpmPtr() const { return mObject.shared_ptr(); }

    private:
        omnetpp::SimTime mExpiry;
        CpmObject mObject;
    };

    using AwarenessEntries = std::map<StationID, AwarenessEntry>;
    using AwarenessCpmEntries = std::map<StationID, AwarenessCpmEntry>;

    LocalDynamicMap(const Timer&);
    void updateAwareness(const CaObject&);
    void updateAwareness(const CpmObject&);
    void dropExpired();
    unsigned count(const CamPredicate&) const;
    std::shared_ptr<const Cam> getCam(StationID) const;
    const AwarenessEntries& allEntries() const { return mCaMessages; }
    unsigned countCpm(const CpmPredicate&) const;

private:
    const Timer& mTimer;
    AwarenessEntries mCaMessages;
    AwarenessCpmEntries mCpmMessages;
};

} // namespace artery

#endif /* ARTERY_LOCALDYNAMICMAP_H_AL7SS9KT */

