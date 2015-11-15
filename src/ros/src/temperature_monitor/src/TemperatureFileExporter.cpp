#include "boost/date_time/posix_time/posix_time.hpp"

#include <temperature_monitor/TemperatureFileExporter.h>

using namespace boost::posix_time;

const char* TemperatureFileExporter::delim_ = " ";

/* ------------------------------------------------------------------------ */

TemperatureFileExporter::TemperatureFileExporter(const std::string& fileName)
: file_(fileName.c_str(), std::ios_base::app)
{
}

/* ------------------------------------------------------------------------ */

void TemperatureFileExporter::exportTemperature(
        const MonitoredTemperature& temperature)
{
    file_ << to_iso_extended_string(temperature.get_time_stamp()) << delim_
          << temperature.get_temperature()
          << std::endl;
}
