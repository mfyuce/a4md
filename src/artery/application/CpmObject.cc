#include <artery/application/CpmObject.h>
#include <omnetpp.h>
#include <cassert>

namespace artery
{

using namespace vanetza::asn1;

Register_Abstract_Class(CpmObject)

CpmObject::CpmObject(Cpm&& cam) :
    m_cam_wrapper(std::make_shared<Cpm>(std::move(cam)))
{
}

CpmObject& CpmObject::operator=(Cpm&& cam)
{
    m_cam_wrapper = std::make_shared<Cpm>(std::move(cam));
    return *this;
}

CpmObject::CpmObject(const Cpm& cam) :
    m_cam_wrapper(std::make_shared<Cpm>(cam))
{
}

CpmObject& CpmObject::operator=(const Cpm& cam)
{
    m_cam_wrapper = std::make_shared<Cpm>(cam);
    return *this;
}

CpmObject::CpmObject(const std::shared_ptr<const Cpm>& ptr) :
    m_cam_wrapper(ptr)
{
    assert(m_cam_wrapper);
}

CpmObject& CpmObject::operator=(const std::shared_ptr<const Cpm>& ptr)
{
    m_cam_wrapper = ptr;
    assert(m_cam_wrapper);
    return *this;
}

std::shared_ptr<const Cpm> CpmObject::shared_ptr() const
{
    assert(m_cam_wrapper);
    return m_cam_wrapper;
}

const vanetza::asn1::Cpm& CpmObject::asn1() const
{
    return *m_cam_wrapper;
}

omnetpp::cObject* CpmObject::dup() const
{
    return new CpmObject { *this };
}

using namespace omnetpp;

class CamStationIdResultFilter : public cObjectResultFilter
{
protected:
    void receiveSignal(cResultFilter* prev, simtime_t_cref t, cObject* object, cObject* details) override
    {
        if (auto cam = dynamic_cast<CpmObject*>(object)) {
            const auto id = cam->asn1()->header.stationID;
            fire(this, t, id, details);
        }
    }
};

Register_ResultFilter("camStationId", CamStationIdResultFilter)


class CamGenerationDeltaTimeResultFilter : public cObjectResultFilter
{
protected:
    void receiveSignal(cResultFilter* prev, simtime_t_cref t, cObject* object, cObject* details) override
    {
        if (auto cam = dynamic_cast<CpmObject*>(object)) {
            const auto genDeltaTime = cam->asn1()->cam.generationDeltaTime;
            fire(this, t, genDeltaTime, details);
        }
    }
};

Register_ResultFilter("camGenerationDeltaTime", CamGenerationDeltaTimeResultFilter)

} // namespace artery
