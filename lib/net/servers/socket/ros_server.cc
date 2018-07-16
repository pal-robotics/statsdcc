
/**
 * Source file for class ROSServer
 * Please see ros_server.h and server.h for documentaion
 */

#include "statsdcc/net/servers/socket/ros_server.h"

#include <sstream>
#include <regex>

#include <boost/bind.hpp>

#include <ros/assert.h>

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

ROSServer::ROSServer(std::string node_name, std::shared_ptr<statsdcc::consumers::Consumer> consumer)
  : Server(1, consumer), node_name(node_name), node_handle("~"), topics_rules(), stat_map()
{
  createStatsSubs();
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

        subs.push_back(node_handle.subscribe<pal_statistics_msgs::Statistics>(
            topic_name, 1000, boost::bind(&ROSServer::statisticsCallback, this, _1, i)));
      }
      else
      {
        ::logger->error("Topics list element invalid format");
      }
    }
  }
}

void ROSServer::statisticsCallback(const pal_statistics_msgs::Statistics::ConstPtr &statistics,
                                   int rules_index)
{
  ros::Time before = ros::Time::now();

  const Rules rules = topics_rules[rules_index];
  std::smatch result;

  /// @todo consumer->consume(metric) call seems thread safe, i.e., data ends up
  /// in a boost::lockfree::queue. Maybe we can send the metrics in parallel? See OpenMP
  for (auto stat = statistics->statistics.begin(); stat != statistics->statistics.end(); ++stat)
  {
    /// @note using map to cache results of regex matches
    auto entry = stat_map.find(stat->name);
    if (entry != stat_map.end())
    {
      for (auto metric_type = entry->second.begin(); metric_type != entry->second.end(); ++metric_type)
      {
        const std::string metric =
            stat->name + ":" + std::to_string(stat->value) + "|" + *metric_type;
        this->consumer->consume(metric);
      }
    }
    else
    {
      for (auto rule = rules.begin(); rule != rules.end(); ++rule)
      {
        if (std::regex_match(stat->name, result, std::regex(rule->first)))
        {
          for (auto metric_type = rule->second.begin(); metric_type != rule->second.end();
               ++metric_type)
          {
            stat_map[stat->name].push_back(*metric_type);

            const std::string metric =
                stat->name + ":" + std::to_string(stat->value) + "|" + *metric_type;
            this->consumer->consume(metric);
          }
          // skip rest of rules after a valid match
          continue;
        }
      }
    }
  }

  ros::Time after = ros::Time::now();
  // ::logger->info("callback took: " + std::to_string((after - before).toSec()));
}

}  // namespace socket
}  // namespace servers
}  // namespace net
}  // namespace statsdcc
