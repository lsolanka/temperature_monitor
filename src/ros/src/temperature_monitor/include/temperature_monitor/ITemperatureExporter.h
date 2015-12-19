#ifndef ITEMPERATUREEXPORTER_H
#define ITEMPERATUREEXPORTER_H

#include <temperature_monitor/MonitoredTemperature.h>

class ITemperatureExporter
{
public:
    /**
     * Export new temperature item.
     */
    virtual void exportTemperature(const MonitoredTemperature& temperature) = 0;
};

#endif // ITEMPERATUREEXPORTER_H
