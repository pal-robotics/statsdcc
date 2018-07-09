
/**
 * Source file for class ROSServer
 * Please see ros_server.h and server.h for documentaion
 */

#include "statsdcc/net/servers/socket/ros_server.h"

#include <sstream>
#include <functional>

#include <boost/bind.hpp>
#include <vector>

#include "statsdcc/logger.h"
#include "statsdcc/net/wrapper.h"
#include "statsdcc/os.h"

namespace statsdcc { namespace net { namespace servers { namespace socket {

ROSServer::ROSServer(std::string node_name, std::shared_ptr<statsdcc::consumers::Consumer> consumer)
  : Server(1, consumer), node_name(node_name), node_handle() {

  /// @note dont think this listener() thread is even needed. Multiple
  /// subscribes with their callbacks should be enough

  sub = node_handle.subscribe("/motors_statistics", 1000, &ROSServer::statisticsCallback, this);

  try {
    this->servers.emplace_back(&ROSServer::listen, this);
  } catch(...) {
    ::logger->error("Thread Creation Error: unable to start ros server");
    this->done = true;
    throw;
  }
}

void ROSServer::listen() {
  this->add_tid(os::get_tid());

  while (!this->done) {
    sleep(1);
    // this->consumer->consume(metric);
  }
}

void ROSServer::statisticsCallback(const pal_statistics_msgs::Statistics &statistics)
{
  ros::Time before = ros::Time::now();

  for (auto it = statistics.statistics.begin(); it != statistics.statistics.end(); ++it)
  {
    const std::string metric = it->name + ":" + std::to_string(it->value) + "|ms";
    this->consumer->consume(metric);
  }

  ros::Time after = ros::Time::now();
  // ::logger->info("Consume took: " + std::to_string((after - before).toSec()));
}

}  // namespace socket
}  // namespace servers
}  // namespace net
}  // namespace statsdcc
