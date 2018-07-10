
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

#include <pal_statistics_msgs/Statistics.h>

#include "statsdcc/net/servers/socket/server.h"
#include "statsdcc/net/wrapper.h"

namespace statsdcc { namespace net { namespace servers { namespace socket {

class ROSServer : public Server {
 public:
  typedef std::vector<std::string> MetricTypes;
  typedef std::pair<std::string, MetricTypes> Rule;
  typedef std::vector<Rule> Rules;

 public:
  /**
   * A constructor
   *
   * @param consumer a refernce to Consumer object that has implementation for
   *                 consume(std::string& metric) method
   */
  ROSServer(std::string node_name, std::shared_ptr<statsdcc::consumers::Consumer> consumer);

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
                          int rules_index);

private:
  std::string node_name;

  ros::NodeHandle node_handle;
  std::vector<ros::Subscriber> subs;
  std::vector<Rules> topics_rules;
};

}  // namespace socket
}  // namespace servers
}  // namespace net
}  // namespace statsdcc

#endif  // INCLUDE_STATSDCC_NET_SERVERS_SOCKET_ROS_SERVER_H_
