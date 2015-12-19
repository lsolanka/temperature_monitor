#ifndef TEMPERATUREFILEEXPORTER_H
#define TEMPERATUREFILEEXPORTER_H

#include <string>
#include <fstream>

#include "ITemperatureExporter.h"
#include <temperature_monitor/MonitoredTemperature.h>

class TemperatureFileExporter : public ITemperatureExporter
{
public:
    /** Open file in `append` mode. **/
    TemperatureFileExporter(const std::string& fileName);

    /**
     * Export new temperature item into file.
     */
    virtual void exportTemperature(const MonitoredTemperature& temperature);

private:
    std::ofstream file_;

    static const char* delim_;
};

#endif // TEMPERATUREFILEEXPORTER_H
