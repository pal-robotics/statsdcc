
#ifndef INCLUDE_STATSDCC_NET_SERVERS_SOCKET_ROS_SERVER_H_
#define INCLUDE_STATSDCC_NET_SERVERS_SOCKET_ROS_SERVER_H_

#include <map>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <boost/ptr_container/ptr_vector.hpp>

#include "pal_statistics_msgs/msg/statistics_names.hpp"
#include "pal_statistics_msgs/msg/statistics_values.hpp"

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"

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
class ROSServer : public Server, public rclcpp::Node
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
  typedef std::unordered_map<std::string, std::shared_ptr<statsdcc::Metric>> TopicProcessingMetrics;

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

  ~ROSServer();

  /**
   * Starts the treads to process in comming data by calling consumer object
   */
  void start();

private:
  ROSServer::Rules to_rules(const std::string & topic_name);

  void createStatsSubs();

  void namesCallback(const pal_statistics_msgs::msg::StatisticsNames::SharedPtr msg,
                     const std::string& topic_name, unsigned int rules_index);
  void valuesCallback(const pal_statistics_msgs::msg::StatisticsValues::SharedPtr msg,
                      const std::string& topic_name, unsigned int rules_index);

private:
  std::shared_ptr<BackendContainer> backend_container_;

  std::vector<rclcpp::Subscription<pal_statistics_msgs::msg::StatisticsNames>::SharedPtr> names_subs_;
  std::vector<rclcpp::Subscription<pal_statistics_msgs::msg::StatisticsValues>::SharedPtr> values_subs_;

  std::vector<Rules> topics_rules_;
  TopicsStatsNames topics_stats_names_;
  TopicMetrics topic_metrics_;
  TopicProcessingMetrics topic_processing_metrics_;

  std::unique_ptr<Ledger> ledger_;
  bool flush_ledger_;
  rclcpp::TimerBase::SharedPtr ledger_timer_;
  std::unique_ptr<ThreadGuard> flusher_guard_;

  std::thread spinner_thread_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

}  // namespace socket
}  // namespace servers
}  // namespace net
}  // namespace statsdcc

#endif  // INCLUDE_STATSDCC_NET_SERVERS_SOCKET_ROS_SERVER_H_
