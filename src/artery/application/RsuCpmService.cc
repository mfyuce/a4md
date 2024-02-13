/*
* Artery V2X Simulation Framework
* Copyright 2014-2020 Raphael Riebl et al.
* Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
*/

#include "artery/application/CpmObject.h"
#include "artery/application/LocalDynamicMap.h"
#include "artery/application/RsuCpmService.h"
#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/MultiChannelPolicy.h"
#include "artery/utility/Geometry.h"
#include "artery/utility/Identity.h"
#include <boost/lexical_cast.hpp>
#include <omnetpp/cexception.h>
#include <omnetpp/cxmlelement.h>
#include <vanetza/btp/ports.hpp>
#include <cmath>

namespace artery
{

using namespace omnetpp;

static const simsignal_t scSignalCpmReceived = cComponent::registerSignal("CpmReceived");
static const simsignal_t scSignalCpmSent = cComponent::registerSignal("CpmSent");

Define_Module(RsuCpmService)

void RsuCpmService::initialize()
{
    ItsG5BaseService::initialize();
    mIdentity = &getFacilities().get_const<Identity>();
    mGeoPosition = &getFacilities().get_const<GeoPosition>();
    mNetworkInterfaceTable = &getFacilities().get_const<NetworkInterfaceTable>();
    mTimer = &getFacilities().get_const<Timer>();
    mLocalDynamicMap = &getFacilities().get_mutable<LocalDynamicMap>();

    mGenerationInterval = par("generationInterval");
    mLastCpmTimestamp = -mGenerationInterval;
    mProtectedCommunicationZones = parseProtectedCommunicationZones(par("protectedCommunicationZones").xmlValue());
    if (mProtectedCommunicationZones.size() > 16) {
        throw cRuntimeError("CAMs can include at most 16 protected communication zones");
    } else {
        EV_INFO << "announcing " << mProtectedCommunicationZones.size() << " protected communication zones\n";
    }

    // look up primary channel for CA
    mPrimaryChannel = getFacilities().get_const<MultiChannelPolicy>().primaryChannel(vanetza::aid::CA);
}

auto RsuCpmService::parseProtectedCommunicationZones(cXMLElement* zones_cfg) -> std::list<ProtectedCommunicationZone>
{
    std::list<ProtectedCommunicationZone> zones;

    for (cXMLElement* zone_cfg : zones_cfg->getChildrenByTagName("zone")) {
        ProtectedCommunicationZone zone;
        zone.latitude_deg = boost::lexical_cast<double>(zone_cfg->getAttribute("latitude"));
        zone.longitude_deg = boost::lexical_cast<double>(zone_cfg->getAttribute("longitude"));
        const char* radius_attr = zone_cfg->getAttribute("radius");
        if (radius_attr) {
            zone.radius_m = boost::lexical_cast<unsigned>(radius_attr);
        }
        const char* id_attr = zone_cfg->getAttribute("id");
        if (id_attr) {
            zone.id = boost::lexical_cast<unsigned>(id_attr);
        }
        const char* type_attr = zone_cfg->getAttribute("type");
        if (type_attr) {
            if (std::strcmp(type_attr, "permanent") == 0) {
                zone.type = ProtectedZoneType_permanentCenDsrcTolling;
            } else if (std::strcmp(type_attr, "temporary") == 0) {
                zone.type = ProtectedZoneType_temporaryCenDsrcTolling;
            } else {
                zone.type = boost::lexical_cast<ProtectedZoneType_t>(type_attr);
            }
        }
        zones.push_back(zone);
    }

    return zones;
}

void RsuCpmService::trigger()
{
    Enter_Method("trigger");
    if (simTime() - mLastCpmTimestamp >= mGenerationInterval) {
        sendCpm();
    }
}

void RsuCpmService::indicate(const vanetza::btp::DataIndication& ind, std::unique_ptr<vanetza::UpPacket> packet)
{
    Enter_Method("indicate");

    Asn1PacketVisitor<vanetza::asn1::Cpm> visitor;
    const vanetza::asn1::Cpm* cpm = boost::apply_visitor(visitor, *packet);
    if (cpm && cpm->validate()) {
        CpmObject obj = visitor.shared_wrapper;
        emit(scSignalCpmReceived, &obj);
        mLocalDynamicMap->updateAwareness(obj);
    }
}

void RsuCpmService::sendCpm()
{
    using namespace vanetza;
    btp::DataRequestB request;
    request.destination_port = btp::ports::CPM;
    request.gn.its_aid = aid::CA;
    request.gn.transport_type = geonet::TransportType::SHB;
    request.gn.maximum_lifetime = geonet::Lifetime { geonet::Lifetime::Base::One_Second, 1 };
    request.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
    request.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

    CpmObject obj(createMessage());
    emit(scSignalCpmSent, &obj);

    using CpmByteBuffer = convertible::byte_buffer_impl<asn1::Cpm>;
    std::unique_ptr<geonet::DownPacket> payload { new geonet::DownPacket() };
    std::unique_ptr<convertible::byte_buffer> buffer { new CpmByteBuffer(obj.shared_ptr()) };
    payload->layer(OsiLayer::Application) = std::move(buffer);
    this->request(request, std::move(payload));
}

vanetza::asn1::Cpm RsuCpmService::createMessage() const
{
    vanetza::asn1::Cpm message;
    ItsPduHeader_t& header = (*message).header;
    header.protocolVersion = 2;
    header.messageID = ItsPduHeader__messageID_cpm;
    header.stationID = mIdentity->application;

    CollectivePerceptionMessage_t& cpm = (*message).cpm;
    const uint16_t genDeltaTime = countTaiMilliseconds(mTimer->getCurrentTime());
    cpm.generationDeltaTime = genDeltaTime * GenerationDeltaTime_oneMilliSec;
    PerceivedObjectContainer* basic = cpm.cpmParameters.perceivedObjectContainer;
//    HighFrequencyContainer_t& hfc = cpm.cpmParameters.managementContainer.highFrequencyContainer;
//    for (int i=0;i<basic->list.count; i++) {
//        const PerceivedObject *po = basic->list.array[i];
//        po.stationType = StationType_roadSideUnit;
//        po.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
//        po.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
//        const double longitude = mGeoPosition->longitude / vanetza::units::degree;
//        po.referencePosition.longitude = std::round(longitude * 1e6 * Longitude_oneMicrodegreeEast);
//        const double latitude = mGeoPosition->latitude / vanetza::units::degree;
//        po.referencePosition.latitude = std::round(latitude * 1e6 * Latitude_oneMicrodegreeNorth);
//        po.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
//        po.referencePosition.positionConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
//        po.referencePosition.positionConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;

//        hfc.present = HighFrequencyContainer_PR_rsuContainerHighFrequency;
//        RSUContainerHighFrequency &rchf = hfc.choice.rsuContainerHighFrequency;
//        if (!mProtectedCommunicationZones.empty()) {
//            rchf.protectedCommunicationZonesRSU = vanetza::asn1::allocate<ProtectedCommunicationZonesRSU_t>();
//            for (const ProtectedCommunicationZone &zone: mProtectedCommunicationZones) {
//                auto asn1 = vanetza::asn1::allocate<ProtectedCommunicationZone_t>();
//                asn1->protectedZoneType = zone.type;
//                asn1->protectedZoneLatitude = std::round(zone.latitude_deg * 1e6 * Latitude_oneMicrodegreeNorth);
//                asn1->protectedZoneLongitude = std::round(zone.longitude_deg * 1e6 * Longitude_oneMicrodegreeEast);
//                if (zone.radius_m > 0) {
//                    asn1->protectedZoneRadius = vanetza::asn1::allocate<ProtectedZoneRadius_t>();
//                    *asn1->protectedZoneRadius = zone.radius_m;
//                }
//                if (zone.id) {
//                    asn1->protectedZoneID = vanetza::asn1::allocate<ProtectedZoneID_t>();
//                    *asn1->protectedZoneID = *zone.id;
//                }
//                ASN_SEQUENCE_ADD(rchf.protectedCommunicationZonesRSU, asn1);
//            }
//        }
//    }

    std::string error;
    if (!message.validate(error)) {
            throw cRuntimeError("Invalid RSU CPM: %s", error.c_str());
    }

    return message;
}

} // namespace artery
