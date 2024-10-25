
/**
 * Source file for class ROSServer
 * Please see ros_server.h and server.h for documentaion
 */

#include "statsdcc/net/servers/socket/ros_server.h"

#include <chrono>
#include <regex>
#include <set>
#include <sstream>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/logging.hpp"

#include "statsdcc/backend_container.h"
#include "statsdcc/ledger.h"
#include "statsdcc/logger.h"
#include "statsdcc/net/wrapper.h"
#include "statsdcc/os.h"

using std::placeholders::_1;

namespace statsdcc
{
namespace net
{
namespace servers
{
namespace socket
{

const static auto node_options = rclcpp::NodeOptions()
  .allow_undeclared_parameters(true)
  .automatically_declare_parameters_from_overrides(true);

ROSServer::ROSServer(std::string node_name, std::shared_ptr<consumers::Consumer> consumer,
                     const std::shared_ptr<BackendContainer> &backend_container)
  : Server(1, consumer)
  , rclcpp::Node(node_name, node_options)

  , backend_container_(backend_container)

  , names_subs_()
  , values_subs_()
  , topics_rules_()
  , topics_stats_names_()
  , topic_metrics_()
  , topic_processing_metrics_()

  , ledger_(new Ledger())
  , flush_ledger_(false)
  , ledger_timer_()
  , flusher_guard_()

  , spinner_thread_()
  , executor_()
{
  createStatsSubs();

  auto ledge_flusher = [&]() { flush_ledger_ = true; };

  ledger_timer_ = this->create_wall_timer(std::chrono::seconds(::config->frequency), ledge_flusher);

  spinner_thread_ = std::thread([&] {
    executor_.add_node(this->get_node_base_interface());
    executor_.spin();
  });
}

ROSServer::~ROSServer()
{
  executor_.cancel();
  spinner_thread_.join();
}

ROSServer::Rules ROSServer::to_rules(const std::string &topic_name)
{
  ROSServer::Rules rules;

  const auto stat_names = "topics." + topic_name + ".stat_names";
  const auto stat_types = "topics." + topic_name + ".stat_types";

  if(!this->has_parameter(stat_names) || !this->has_parameter(stat_types))
  {
    const auto message = "No stats defined for " + topic_name;
    RCLCPP_ERROR(this->get_logger(), message.c_str());
    throw std::runtime_error(message);
  }

  const auto stat_names_list = this->get_parameter(stat_names).as_string_array();
  const auto stat_types_list = this->get_parameter(stat_types).as_string_array();

  if(stat_names_list.size() != stat_types_list.size())
  {
    const auto message = "'stat_names' and 'stat_types' list have different sizes for topic " + topic_name;
    RCLCPP_ERROR(this->get_logger(), message.c_str());
    throw std::runtime_error(message);
  }

  for (size_t i = 0; i < stat_names_list.size(); ++i)
  {
    const std::string name = stat_names_list[i];
    const std::string type = stat_types_list[i];

    // check type is valid
    if (type != "c" && type != "g" && type != "t" && type != "s")
    {
      const auto message = "Invalid metric type '" + type + "' for stat '" + name + "'";
      RCLCPP_ERROR(this->get_logger(), message.c_str());
      throw std::runtime_error(message);
    }

    // t -> ms conversion
    if (type == "t")
    {
      rules.push_back(std::make_pair(name, MetricTypes{"ms"}));
    }
    else
    {
      rules.push_back(std::make_pair(name, MetricTypes{type}));
    }
  }

  return rules;
}

void ROSServer::createStatsSubs()
{
  // List parameters with the prefix "topics" with any depth
  constexpr auto kAnyDepth = 0u;
  const auto params = this->list_parameters({"topics"}, kAnyDepth);

  const int init_position = std::string("topics.").size();

  std::set<std::string> topics_names;
  for (const auto & param : params.names) {
    // Find the topic name after 'topics.' and before the next '.'
    auto topic_name = param.substr(
        init_position,
        param.find_first_of('.', init_position) - init_position);

    topics_names.insert(topic_name);
  }

  auto i = 0u;
  for (const auto & topic_name : topics_names)
  {
    topics_rules_.push_back(to_rules(topic_name));

    RCLCPP_INFO(this->get_logger(), "Creating subscribers for %s", topic_name.c_str());

    auto names_qos = rclcpp::QoS(rclcpp::KeepLast(1000)).transient_local();

    auto names_callback =
        [topic_name, i, this](const pal_statistics_msgs::msg::StatisticsNames::SharedPtr msg) {
          namesCallback(msg, topic_name, i);
        };

    auto values_callback =
        [topic_name, i, this](const pal_statistics_msgs::msg::StatisticsValues::SharedPtr msg) {
          valuesCallback(msg, topic_name, i);
        };

    auto name_subscription =  this->create_subscription<pal_statistics_msgs::msg::StatisticsNames>(
        topic_name + "/names", names_qos, names_callback);

    auto value_subscription =  this->create_subscription<pal_statistics_msgs::msg::StatisticsValues>(
        topic_name + "/values", 1000, values_callback);

    names_subs_.push_back(name_subscription);
    values_subs_.push_back(value_subscription);

    ++i;
  }
}

void ROSServer::namesCallback(const pal_statistics_msgs::msg::StatisticsNames::SharedPtr msg,
                              const std::string &topic_name, unsigned int /*rules_index*/)
{
  RCLCPP_INFO(this->get_logger(), "Statistics names from %s received", topic_name.c_str());
  topics_stats_names_[topic_name] = std::make_pair(msg->names, msg->names_version);
  topic_metrics_[topic_name].clear();
}

void ROSServer::valuesCallback(const pal_statistics_msgs::msg::StatisticsValues::SharedPtr msg,
                               const std::string &topic_name, unsigned int rules_index)
{
  const auto &topic_stats_name = topics_stats_names_[topic_name];
  // discard if no names for this topic were received or versions differ
  if (topic_stats_name.first.empty())
  {
    RCLCPP_WARN(this->get_logger(),
      "Discarding values from %s, no names received yet", topic_name.c_str());
    return;
  }

  if (topic_stats_name.second != msg->names_version)
  {
    RCLCPP_WARN(this->get_logger(),
      "Discarding values from %s, names and values version differ", topic_name.c_str());
    return;
  }

  auto before = std::chrono::system_clock::now();

  const Rules &rules = topics_rules_[rules_index];
  std::smatch result;

  /// @todo concurrency is not an issue just because callbacks are serialized
  /// just in case -> one ledger per topic
  /// @todo avoid copying the ledger, just switch pointers!
  auto &metrics_vector = topic_metrics_[topic_name];
  bool has_computed_metrics = !metrics_vector.empty();
  if (has_computed_metrics)
  {
    for (size_t i = 0; i < msg->values.size(); ++i)
    {
      const double &stat_value = msg->values[i];

    for (auto metric = metrics_vector[i].begin(); metric != metrics_vector[i].end(); ++metric)
    {
      ledger_->buffer(*metric, stat_value);
    }
    }
  }
  else
  {
    metrics_vector.resize(msg->values.size());
    for (size_t i = 0; i < msg->values.size(); ++i)
    {
      const std::string &stat_name = topic_stats_name.first[i];
      const double &stat_value = msg->values[i];

      bool rule_found = false;
      for (auto rule = rules.begin(); rule != rules.end(); ++rule)
      {
        if (std::regex_match(stat_name, result, std::regex(rule->first)))
        {
          if (rule->second.empty())
          {
            RCLCPP_INFO(this->get_logger(),
              "%s has no metric types defined. Stats won't be logged", stat_name.c_str());
          }

          for (auto metric_type = rule->second.begin(); metric_type != rule->second.end();
               ++metric_type)
          {
            auto metric = ledger_->buffer(stat_name, stat_value, *metric_type);
            metrics_vector[i].push_back(metric);
          }
          // skip rest of rules after a valid match
          rule_found = true;
          break;
        }
      }

      // if no valid rule was found, guarantee we are not looking for a valid regex each
      // time
      if (!rule_found)
      {
        RCLCPP_INFO(this->get_logger(),
          "%s is not matched by any rule. Stat won't be logged", stat_name.c_str());
      }
    }
  }

  if (flush_ledger_)
  {
    // process and flush ledger in separate thread
    flusher_guard_.reset(new ThreadGuard(std::thread(
        &BackendContainer::processAndFlush, backend_container_, std::move(this->ledger_), 0)));

    // delete previous ledger and create new one
    ledger_.reset(new Ledger());
    topic_metrics_.clear(); // Stats where pointing to deleted pointers

    flush_ledger_ = false;
  }

  auto after = std::chrono::system_clock::now();

  const std::string stat_name = "statsdcc." + topic_name + ".callback_processing_time";
  const auto stat_value = std::chrono::duration_cast<std::chrono::milliseconds>(after - before).count();
  auto it = topic_processing_metrics_.find(topic_name);
  if (it == topic_processing_metrics_.end())
  {
    topic_processing_metrics_[topic_name] = ledger_->buffer(stat_name, stat_value, "ms");
  }
  else
  {
    ledger_->buffer(it->second, stat_value);
  }
}

}  // namespace socket
}  // namespace servers
}  // namespace net
}  // namespace statsdcc
