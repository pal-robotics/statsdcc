
#ifndef INCLUDE_STATSDCC_NET_SERVERS_SOCKET_ROS_SERVER_H_
#define INCLUDE_STATSDCC_NET_SERVERS_SOCKET_ROS_SERVER_H_

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <utility>

#include <boost/ptr_container/ptr_vector.hpp>

#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/timer.h>

#include <pal_statistics_msgs/StatisticsNames.h>
#include <pal_statistics_msgs/StatisticsValues.h>

#include "statsdcc/net/servers/socket/server.h"
#include "statsdcc/net/wrapper.h"

namespace statsdcc
{
// forward declarations
class BackendContainer;
class Ledger;
class Metric;
namespace net
{
namespace servers
{
namespace socket
{
class ROSServer : public Server
{
public:
  typedef std::vector<std::string> MetricTypes;
  typedef std::vector<std::shared_ptr<statsdcc::Metric>> Metrics;
  typedef std::pair<std::string, MetricTypes> Rule;
  typedef std::vector<Rule> Rules;

  typedef std::unordered_map<std::string, Metrics> StatMap;

  typedef std::vector<std::string> StatsNames;
  typedef std::pair<StatsNames, uint32_t> StatsNamesVersion;
  typedef std::unordered_map<std::string, StatsNamesVersion> TopicsStatsNames;
  typedef std::unordered_map<std::string, std::vector<Metrics>> TopicMetrics;

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

  void namesCallback(const pal_statistics_msgs::StatisticsNames::ConstPtr& names,
                     const std::string& topic_name, int rules_index);
  void valuesCallback(const pal_statistics_msgs::StatisticsValues::ConstPtr& values,
                      const std::string& topic_name, int rules_index);

private:
  std::string node_name_;

  std::shared_ptr<BackendContainer> backend_container_;

  ros::NodeHandle node_handle_;
  std::vector<ros::Subscriber> subs_;
  std::vector<Rules> topics_rules_;
  TopicsStatsNames topics_stats_names_;
  TopicMetrics topic_metrics_;

  std::unique_ptr<Ledger> ledger_;
  bool flush_ledger_;
  ros::Timer ledger_timer_;
  std::unique_ptr<ThreadGuard> flusher_guard_;
};

}  // namespace socket
}  // namespace servers
}  // namespace net
}  // namespace statsdcc

#endif  // INCLUDE_STATSDCC_NET_SERVERS_SOCKET_ROS_SERVER_H_
