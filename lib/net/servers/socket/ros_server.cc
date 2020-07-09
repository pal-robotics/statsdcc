
/**
 * Source file for class ROSServer
 * Please see ros_server.h and server.h for documentaion
 */

#include "statsdcc/net/servers/socket/ros_server.h"

#include <sstream>
#include <regex>

#include <boost/bind.hpp>

#include <ros/assert.h>

#include "statsdcc/backend_container.h"
#include "statsdcc/ledger.h"
#include "statsdcc/logger.h"
#include "statsdcc/net/wrapper.h"
#include "statsdcc/os.h"

namespace statsdcc
{
namespace net
{
namespace servers
{
namespace socket
{
namespace
{
ROSServer::Rules to_rules(const XmlRpc::XmlRpcValue &stats)
{
  ROSServer::Rules rules;
  ROS_ASSERT(stats.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int i = 0; i < stats.size(); ++i)
  {
    XmlRpc::XmlRpcValue stat = stats[i];
    ROS_ASSERT(stat.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    if (stat.hasMember("name") && stat.hasMember("type"))
    {
      ROS_ASSERT(stat["name"].getType() == XmlRpc::XmlRpcValue::TypeString);
      const std::string name = static_cast<std::string>(stat["name"]);

      ROS_ASSERT(stat["type"].getType() == XmlRpc::XmlRpcValue::TypeArray);
      ROSServer::MetricTypes metric_types;
      for (int j = 0; j < stat["type"].size(); ++j)
      {
        ROS_ASSERT(stat["type"][j].getType() == XmlRpc::XmlRpcValue::TypeString);

        const std::string metric_type = static_cast<std::string>(stat["type"][j]);
        // converto to "c", "ms", "g" or "s"
        if (metric_type == "c" || metric_type == "counter")
        {
          metric_types.push_back("c");
        }
        else if (metric_type == "g" || metric_type == "gauge")
        {
          metric_types.push_back("g");
        }
        else if (metric_type == "t" || metric_type == "timer")
        {
          metric_types.push_back("ms");
        }
        else if (metric_type == "s" || metric_type == "set")
        {
          metric_types.push_back("s");
        }
      }

      rules.push_back(std::make_pair(name, metric_types));
    }
    else
    {
      ::logger->error("Stats list element invalid format");
    }
  }
  return rules;
}
}

ROSServer::ROSServer(std::string node_name, std::shared_ptr<consumers::Consumer> consumer,
                     const std::shared_ptr<BackendContainer> &backend_container)
  : Server(1, consumer)
  , node_name_(node_name)
  , backend_container_(backend_container)
  , node_handle_("~")
  , topics_rules_()
  , topics_stats_names_()
  , topic_metrics_()
  , ledger_(new Ledger())
  , flush_ledger_(false)
  , ledger_timer_()
  , flusher_guard_()
{
  createStatsSubs();

  auto ledge_flusher = [&](const ros::TimerEvent & /*event*/) { flush_ledger_ = true; };

  ledger_timer_ = node_handle_.createTimer(ros::Duration(::config->frequency), ledge_flusher);
}

void ROSServer::createStatsSubs()
{
  // retrieve statistics rules
  if (node_handle_.hasParam("topics"))
  {
    ::logger->info("Retrieving topics list");

    XmlRpc::XmlRpcValue topics, topic;
    std::string topic_name;

    node_handle_.getParam("topics", topics);
    ROS_ASSERT(topics.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int i = 0; i < topics.size(); ++i)
    {
      topic = topics[i];
      ROS_ASSERT(topic.getType() == XmlRpc::XmlRpcValue::TypeStruct);

      if (topic.hasMember("name") && topic.hasMember("stats"))
      {
        ROS_ASSERT(topic["name"].getType() == XmlRpc::XmlRpcValue::TypeString);
        ROS_ASSERT(topic["stats"].getType() == XmlRpc::XmlRpcValue::TypeArray);

        topic_name = static_cast<std::string>(topic["name"]);

        // convert XmlRpc stats in an more iterate friendly type
        topics_rules_.push_back(to_rules(topic["stats"]));

        ::logger->info("Creating subscribers for " + topic_name);

        // skip '/' character from the topic names

        /// @todo delete this
        //        subs_.push_back(node_handle_.subscribe<pal_statistics_msgs::Statistics>(
        //            topic_name, 1000,
        //            boost::bind(&ROSServer::statisticsCallback, this, _1,
        //            topic_name.substr(1), i)));



        subs_.push_back(node_handle_.subscribe<pal_statistics_msgs::StatisticsNames>(
            topic_name + "/names", 1000,
            boost::bind(&ROSServer::namesCallback, this, _1, topic_name.substr(1), i)));

        subs_.push_back(node_handle_.subscribe<pal_statistics_msgs::StatisticsValues>(
            topic_name + "/values", 1000,
            boost::bind(&ROSServer::valuesCallback, this, _1, topic_name.substr(1), i)));
      }
      else
      {
        ::logger->error("Topics list element invalid format");
      }
    }
  }
}

void ROSServer::namesCallback(const pal_statistics_msgs::StatisticsNames::ConstPtr &names,
                              const std::string &topic_name, int /*rules_index*/)
{
  ::logger->info("Statistics names from " + topic_name + " received");
  topics_stats_names_[topic_name] = std::make_pair(names->names, names->names_version);
  topic_metrics_[topic_name].clear();
}

void ROSServer::valuesCallback(const pal_statistics_msgs::StatisticsValues::ConstPtr &values,
                               const std::string &topic_name, int rules_index)
{
  const auto &topic_stats_name = topics_stats_names_[topic_name];
  // discard if no names for this topic were received or versions differ
  if (topic_stats_name.first.empty() ||
      topic_stats_name.second != values->names_version)
  {
    ::logger->warn("Discarding values from " + topic_name + ", names and values version "
                                                            "differ");
    return;
  }

  ros::Time before = ros::Time::now();

  const Rules &rules = topics_rules_[rules_index];
  std::smatch result;

  /// @todo concurrency is not an issue just because callbacks are serialized
  /// just in case -> one ledger per topic
  /// @todo avoid copying the ledger, just switch pointers!
  auto &metrics_vector = topic_metrics_[topic_name];
  bool has_computed_metrics = !metrics_vector.empty();
  if (has_computed_metrics)
  {
    for (size_t i = 0; i < values->values.size(); ++i)
    {
      const double &stat_value = values->values[i];

    for (auto metric = metrics_vector[i].begin(); metric != metrics_vector[i].end(); ++metric)
    {
      ledger_->buffer(*metric, stat_value);
    }
    }
  }
  else
  {
    metrics_vector.resize(values->values.size());
    for (size_t i = 0; i < values->values.size(); ++i)
    {
      const std::string &stat_name = topic_stats_name.first[i];
      const double &stat_value = values->values[i];

      bool rule_found = false;
      for (auto rule = rules.begin(); rule != rules.end(); ++rule)
      {
        if (std::regex_match(stat_name, result, std::regex(rule->first)))
        {
          if (rule->second.empty())
          {
            ::logger->warn(stat_name + " has no metric types defined. Stats won't be "
                                       "logged");
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
        ::logger->warn(stat_name + " is not matched by any rule. Stat won't be logged");
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
    topic_metrics_[topic_name].clear(); // Stats where pointing to deleted pointers

    flush_ledger_ = false;
  }

  ros::Time after = ros::Time::now();

  const std::string stat_name = "statsdcc." + topic_name + ".callback_processing_time";
  const double stat_value = (after - before).toSec();
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
