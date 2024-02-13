#ifndef ARTERY_CPMOBJECT_H_
#define ARTERY_CPMOBJECT_H_

#include <omnetpp/cobject.h>
#include <vanetza/asn1/cpm.hpp>
#include <memory>

namespace artery
{

class CpmObject : public omnetpp::cObject
{
public:
    CpmObject(const CpmObject&) = default;
    CpmObject& operator=(const CpmObject&) = default;

    CpmObject(vanetza::asn1::Cpm&&);
    CpmObject& operator=(vanetza::asn1::Cpm&&);

    CpmObject(const vanetza::asn1::Cpm&);
    CpmObject& operator=(const vanetza::asn1::Cpm&);

    CpmObject(const std::shared_ptr<const vanetza::asn1::Cpm>&);
    CpmObject& operator=(const std::shared_ptr<const vanetza::asn1::Cpm>&);

    const vanetza::asn1::Cpm& asn1() const;

    std::shared_ptr<const vanetza::asn1::Cpm> shared_ptr() const;

    omnetpp::cObject* dup() const override;

private:
    std::shared_ptr<const vanetza::asn1::Cpm> m_cam_wrapper;
};

} // namespace artery

#endif /* ARTERY_CPMOBJECT_H_ */
