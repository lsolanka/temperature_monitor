#ifndef MONITOREDTEMPERATURE_H
#define MONITOREDTEMPERATURE_H

#include "boost/date_time/posix_time/posix_time_types.hpp"

/** A class that holds temperature data at a certain time **/
class MonitoredTemperature
{
public:
    MonitoredTemperature(boost::posix_time::ptime time_stamp,
                         double temperature);
    
    /** Create temperature and take current time as a time stamp. **/
    MonitoredTemperature(double temperature);

    boost::posix_time::ptime get_time_stamp() const { return time_stamp_; }
    double get_temperature() const { return temperature_; }

private:
    boost::posix_time::ptime time_stamp_;
    double temperature_;
};

#endif // MONITOREDTEMPERATURE_H
