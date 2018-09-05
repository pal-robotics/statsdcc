
#ifndef INCLUDE_STATSDCC_NET_SERVERS_SOCKET_ROS_SERVER_H_
#define INCLUDE_STATSDCC_NET_SERVERS_SOCKET_ROS_SERVER_H_

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <utility>

#include <boost/ptr_container/ptr_vector.hpp>

#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/timer.h>

#include <pal_statistics_msgs/Statistics.h>

#include "statsdcc/net/servers/socket/server.h"
#include "statsdcc/net/wrapper.h"

namespace statsdcc {

// forward declarations
class BackendContainer;
class Ledger;

namespace net { namespace servers { namespace socket {

class ROSServer : public Server {
 public:
  typedef std::vector<std::string> MetricTypes;
  typedef std::pair<std::string, MetricTypes> Rule;
  typedef std::vector<Rule> Rules;

  typedef std::map<std::string, MetricTypes> StatMap;

 public:
  /**
   * A constructor
   *
   * @param consumer a refernce to Consumer object that has implementation for
   *                 consume(std::string& metric) method
   */
  ROSServer(std::string node_name, std::shared_ptr<statsdcc::consumers::Consumer> consumer,
            const std::shared_ptr<BackendContainer>& backend_container);

  ROSServer(const ROSServer&) = delete;
  ROSServer& operator=(const ROSServer&) = delete;

  ROSServer(ROSServer&&) = delete;
  ROSServer& operator=(ROSServer&&) = delete;

  ~ROSServer() = default;

  /**
   * Starts the treads to process in comming data by calling consumer object
   */
  void start();

 private:
  void createStatsSubs();

  void statisticsCallback(const pal_statistics_msgs::Statistics::ConstPtr& statistics,
                          const std::string& topic_name, int rules_index);

private:
  std::string node_name;

  std::shared_ptr<BackendContainer> backend_container;

  ros::NodeHandle node_handle;
  std::vector<ros::Subscriber> subs;
  std::vector<Rules> topics_rules;
  StatMap stat_map;

  std::unique_ptr<Ledger> ledger;
  bool flush_ledger;
  ros::Timer ledger_timer;
  std::unique_ptr<ThreadGuard> flusher_guard;
};

}  // namespace socket
}  // namespace servers
}  // namespace net
}  // namespace statsdcc

#endif  // INCLUDE_STATSDCC_NET_SERVERS_SOCKET_ROS_SERVER_H_
