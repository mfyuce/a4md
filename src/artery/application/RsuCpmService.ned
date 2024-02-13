//
// Artery V2X Simulation Framework
// Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
//

package artery.application;

simple RsuCpmService like ItsG5Service
{
    parameters:
        @signal[CpmReceived](type=CaObject);
        @signal[CpmSent](type=CaObject);

        @statistic[reception](source=CpmReceived;record=vector(cpmStationId)?,vector(cpmGenerationDeltaTime)?);
        @statistic[transmission](source=CpmSent;record=vector(cpmStationId)?,vector(cpmGenerationDeltaTime)?);

        // CPM generation interval
        double generationInterval @unit(s) = default(1.0s);

        // announce protected communication zones (where vehicles need to reduce transmission power)
        xml protectedCommunicationZones = default(xml("<zones/>"));
}
