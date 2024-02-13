/*
* Artery V2X Simulation Framework
* Copyright 2014-2019 Raphael Riebl et al.
* Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
*/

#ifndef ARTERY_CPMSERVICE_H_
#define ARTERY_CPMSERVICE_H_

#include "artery/application/BaseCpmService.h"
#include "artery/utility/Channel.h"
#include "artery/utility/Geometry.h"
#include <vanetza/btp/data_interface.hpp>

namespace artery {

    class NetworkInterfaceTable;

    class Timer;

    class VehicleDataProvider;

    class CpmService : public BaseCpmService {
    public:
        CpmService() = default;

        void initialize() override;

        void indicate(const vanetza::btp::DataIndication &, std::unique_ptr<vanetza::UpPacket>) override;

        void trigger() override;

        template<typename T, typename U>
        static long round(const boost::units::quantity<T> &q, const U &u);

    private:
        void sendCpm(const omnetpp::SimTime &);
    };

} // namespace artery

#endif /* ARTERY_CPMSERVICE_H_ */
