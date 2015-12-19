#include <string>
#include <exception>

#include <boost/program_options.hpp>

#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include <temperature_monitor/MonitoredTemperature.h>
#include <temperature_monitor/ITemperatureExporter.h>
#include <temperature_monitor/TemperatureFileExporter.h>

using namespace std;
namespace po = boost::program_options;

const std::string TEMPERATURE_TOPIC_NAME = "sensor_temp";

bool parse_options(int argc, char* argv[], po::variables_map& vm)
{
    try
    {
        po::options_description desc("Allowed options");
        desc.add_options()
            ("help,h",       "Print help message")
            ("topic_name",   po::value<std::string>()->default_value(TEMPERATURE_TOPIC_NAME)->required(), "Topic name to use for subscribing to temperature")
            ("output_fname", po::value<std::string>()->required(), "Output file name");

        po::positional_options_description pos_desc;
        pos_desc.add("output_fname", 1);

        po::store(po::command_line_parser(argc, argv).options(desc).
                                                      positional(pos_desc).run(),
                  vm);
        po::notify(vm);

        if (vm.count("help"))
        {
            std::cout << desc << std::endl;
            return 0;
        }

        std::cout << "Will subscribe to topic: "
                  << vm["topic_name"].as<std::string>() << std::endl;

        std::cout << "Output file name: " << vm["output_fname"].as<std::string>() << std::endl;
    }
    catch (exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return false;
    }
    catch (...)
    {
        std::cerr << "Unknown error occurred" << std::endl;
        return false;
    }

    return true;
}

class TemperatureSubscriber
{
public:
    TemperatureSubscriber(ITemperatureExporter& exporter,
                          ros::NodeHandle& nh,
                          const std::string& topicName,
                          const std::string& logPrefix = "")
    : exporter_(exporter)
    , subscriber_()
    , topicName_(topicName)
    , logPrefix_(logPrefix)
    {
        subscriber_ = nh.subscribe(topicName_, 1000,
                                   &TemperatureSubscriber::callback, this);
        ROS_INFO_STREAM(logPrefix_ << "Started subscriber on topic: "
                                   << topicName_);
    }

private:
    ITemperatureExporter& exporter_;

    ros::Subscriber subscriber_;
    void callback(const std_msgs::Float32ConstPtr msg);

    const std::string topicName_;
    const std::string logPrefix_;
};

void TemperatureSubscriber::callback(const std_msgs::Float32ConstPtr msg)
{
    exporter_.exportTemperature(MonitoredTemperature(msg->data));
    ROS_INFO_STREAM(logPrefix_ << "Exported temperature: " << msg->data);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "temperature_file_exporter");

    po::variables_map vm;
    if (!parse_options(argc, argv, vm))
    {
        return 1;
    }

    ros::NodeHandle nh;
    TemperatureFileExporter exporter(vm["output_fname"].as<string>());
    TemperatureSubscriber subscriber(exporter, nh,
                                     vm["topic_name"].as<string>(),
                                     "TemperatureSubscriber - ");
    ros::spin();

    return 0;
}
