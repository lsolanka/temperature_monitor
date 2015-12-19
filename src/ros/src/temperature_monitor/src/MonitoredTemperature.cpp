#include <temperature_monitor/MonitoredTemperature.h>

using namespace boost::posix_time;

MonitoredTemperature::MonitoredTemperature(
        boost::posix_time::ptime time_stamp, double temperature)
: time_stamp_(time_stamp)
, temperature_(temperature)
{
}

/* ------------------------------------------------------------------------ */

MonitoredTemperature::MonitoredTemperature(double temperature)
: time_stamp_(microsec_clock::universal_time())
, temperature_(temperature)
{
}
