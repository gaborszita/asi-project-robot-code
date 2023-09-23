#include "logging/LoggingController.hpp"
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/sinks.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/attributes/timer.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/log/utility/setup/console.hpp>

namespace logging = boost::log;
namespace keywords = boost::log::keywords;
namespace sinks = boost::log::sinks;
namespace expr = boost::log::expressions;
namespace attrs = boost::log::attributes;

namespace RobotCode::Logging {

void LoggingController::init() {
  boost::shared_ptr<logging::core> core = logging::core::get();

  // Add common attributes like timestamp and log level
  logging::add_common_attributes();

  // Log non-sensor data to console
  boost::shared_ptr<sinks::text_ostream_backend> console_log_backend =
      boost::make_shared<sinks::text_ostream_backend>(
          keywords::filter = expr::attr<std::string>("Channel") != "device"
      );

  typedef sinks::synchronous_sink<sinks::text_ostream_backend> sync_console_log;
  boost::shared_ptr<sync_console_log> console_sink(new sync_console_log(console_log_backend));
  core->add_sink(console_sink);

  // Log sensor data to file
  boost::shared_ptr<sinks::text_file_backend> sensor_log_backend =
      boost::make_shared<sinks::text_file_backend>(

          keywords::target = "/home/pi",
          keywords::file_name = "/home/pi/%y_%m_%d.log",
          keywords::filter = expr::attr<std::string>("Channel") == "device"
      );

  typedef sinks::asynchronous_sink<sinks::text_file_backend> async_file_log;
  boost::shared_ptr<async_file_log> file_sink(new async_file_log(sensor_log_backend));
  file_sink->set_formatter(expr::format(R"("%1%","%2%","%3%")")
                               % expr::attr<long long>("DataTimeStamp")
                               % expr::attr<std::string>("Device")
                               % expr::smessage);
  core->add_sink(file_sink);
}

}