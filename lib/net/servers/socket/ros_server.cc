
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

namespace statsdcc { namespace net { namespace servers { namespace socket {

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
        if(metric_type == "c" || metric_type == "counter")
        {
          metric_types.push_back("c");
        }
        else if(metric_type == "g" || metric_type == "gauge")
        {
          metric_types.push_back("g");
        }
        else if(metric_type == "t" || metric_type == "timer")
        {
          metric_types.push_back("ms");
        }
        else if(metric_type == "s" || metric_type == "set")
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
  , node_name(node_name)
  , backend_container(backend_container)
  , node_handle("~")
  , topics_rules()
  , stat_map()
  , ledger(new Ledger())
  , flush_ledger(false)
  , ledger_timer()
  , flusher_guard()
{
  createStatsSubs();

  auto ledge_flusher = [&](const ros::TimerEvent &/*event*/)
  {
    flush_ledger = true;
  };

  ledger_timer = node_handle.createTimer(ros::Duration(::config->frequency), ledge_flusher);
}

void ROSServer::createStatsSubs()
{
  // retrieve statistics rules
  if (node_handle.hasParam("topics"))
  {
    ::logger->info("Retrieving topics list");

    XmlRpc::XmlRpcValue topics, topic;
    std::string topic_name;

    node_handle.getParam("topics", topics);
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
        topics_rules.push_back(to_rules(topic["stats"]));

        ::logger->info("Creating subscriber for " + topic_name);

        // skip '/' character from the topic name
        subs.push_back(node_handle.subscribe<pal_statistics_msgs::Statistics>(
            topic_name, 1000,
            boost::bind(&ROSServer::statisticsCallback, this, _1, topic_name.substr(1), i)));
      }
      else
      {
        ::logger->error("Topics list element invalid format");
      }
    }
  }
}

void ROSServer::statisticsCallback(const pal_statistics_msgs::Statistics::ConstPtr &statistics,
                                   const std::string &topic_name, int rules_index)
{
  ros::Time before = ros::Time::now();

  const Rules rules = topics_rules[rules_index];
  std::smatch result;

  /// @todo concurrency is not an issue just because callbacks are serialized
  /// just in case -> one ledger per topic
  /// @todo avoid copying the ledger, just switch pointers!

  for (auto stat = statistics->statistics.begin(); stat != statistics->statistics.end(); ++stat)
  {
    /// @note using map to cache results of regex matches
    auto entry = stat_map.find(stat->name);
    if (entry != stat_map.end())
    {
      for (auto metric_type = entry->second.begin(); metric_type != entry->second.end(); ++metric_type)
      {
        ledger->buffer(stat->name, stat->value, *metric_type);
      }
    }
    else
    {
      for (auto rule = rules.begin(); rule != rules.end(); ++rule)
      {
        if (std::regex_match(stat->name, result, std::regex(rule->first)))
        {
          stat_map[stat->name] = MetricTypes();
          if(rule->second.empty())
          {
            ::logger->warn(stat->name + " has no metric types defined. Stats won't be logged");
          }

          for (auto metric_type = rule->second.begin(); metric_type != rule->second.end();
               ++metric_type)
          {
            stat_map[stat->name].push_back(*metric_type);

            ledger->buffer(stat->name, stat->value, *metric_type);
          }
          // skip rest of rules after a valid match
          continue;
        }
      }

      // if no valid rule was found, guarantee we are not looking for a valid regex each time
      if(stat_map.find(stat->name) == stat_map.end())
      {
        stat_map[stat->name] = MetricTypes();
        ::logger->warn(stat->name + " is not matched by any rule. Stat won't be logged");
      }
    }
  }

  if (flush_ledger)
  {
    // process and flush ledger in separate thread
    flusher_guard.reset(new ThreadGuard(std::thread(
        &BackendContainer::processAndFlush, backend_container, Ledger(*this->ledger), 0)));

    // delete previous ledger and create new one
    ledger.reset(new Ledger());

    flush_ledger = false;
  }

  ros::Time after = ros::Time::now();

  const std::string stat_name = "statsdcc." + topic_name + ".callback_processing_time";
  const double stat_value = (after - before).toSec();
  ledger->buffer(stat_name, stat_value, "ms");
}

}  // namespace socket
}  // namespace servers
}  // namespace net
}  // namespace statsdcc
