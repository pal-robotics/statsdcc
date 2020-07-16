
/**
 * Source file for class carbon
 * Please see carbon.h and backend.h for documentaion
 */

#include "statsdcc/backends/carbon.h"

#include <netdb.h>

#include <unordered_map>

#include "statsdcc/chrono.h"
#include "statsdcc/configs/aggregator_config.h"
#include "statsdcc/hashring/hashring.h"
#include "statsdcc/net/connection.h"
#include "statsdcc/net/lib.h"
#include "statsdcc/net/wrapper.h"
#include "statsdcc/os.h"

namespace statsdcc { namespace backends {

Carbon::Carbon() : fqdn("") {
  try {
    this->hashring = std::unique_ptr<hashring::Hashring>(
                      new hashring::Hashring(::config->backends.carbon));
  } catch(std::string msg) {
    ::logger->error("Unable to create hashring: " + msg);
    throw;
  }

  // fill fqdn
  char hostname[255];
  struct hostent* host_info;
  hostname[254] = '\0';
  gethostname(hostname, 255);
  host_info = gethostbyname(hostname);
  fqdn = host_info->h_name;
  fqdn.append(".");
}

void Carbon::flush_stats(const Ledger& ledger, int flusher_id) {
  std::uint64_t time_stamp = chrono::unixtime_ms() / 1000;
  std::string ts_suffix = " " +
    std::to_string(static_cast<long long unsigned int>(time_stamp)) +
    "\n";
  std::uint64_t start_time = chrono::unixtime_ms();
  std::unordered_map<Hostport, std::string> stat_strings;
  std::uint64_t num_stats = 0;
  std::string timer_data_key;

  // prefix for aggregator stats
  this->prefix_stats =
    ::config->name + ".thread_" +
      std::to_string(static_cast<long long int>(flusher_id));

  // prefix for all stats
  this->prefix =
    ::config->prefix;

  // use metric type prefix
  this->use_metric_type_prefix =
      ::config->use_metric_type_prefix;

  // add fqdn prefix
  this->add_fqdn_prefix =
      ::config->add_fqdn_prefix;

  std::string fqdn_prefix = "";
  if(this->add_fqdn_prefix)
  {
    fqdn_prefix = fqdn;
  }

  for (const auto &m : ledger.metrics) {
    std::string key = m.first;
    Metric* metric = m.second.get();
    Counter* counter = dynamic_cast<Counter*>(metric);
    Timer* timer = dynamic_cast<Timer*>(metric);
    Gauge* gauge = dynamic_cast<Gauge*>(metric);

    if (counter)
    {
      std::string value =
        std::to_string(static_cast<long double>(counter->counter_));

      std::string value_per_second =
        std::to_string(static_cast<long double>(counter->counter_rate_));
      // get the destination carbon hostport
      Hostport n = this->hashring->get(key);

      std::string metric_name = this->prefix + fqdn_prefix;
      if (this->use_metric_type_prefix) {
        metric_name = metric_name + "counters.";
      }
      metric_name = metric_name + this->process_name(key);

      stat_strings[n] +=
        metric_name + ".rate"
                    + " "
                    + value_per_second
                    + ts_suffix;

      stat_strings[n] +=
        metric_name + ".count"
                    + " "
                    + value
                    + ts_suffix;

      ++num_stats;
    }
    else if (timer)
    {
      std::string metric_name = this->prefix + fqdn_prefix;
      if (this->use_metric_type_prefix) {
        metric_name = metric_name + "timers.";
      }
      metric_name = metric_name + this->process_name(key);

      for (auto timer_data_itr = timer->timer_data_.cbegin();
          timer_data_itr != timer->timer_data_.cend();
          ++timer_data_itr) {
        std::string timer_data_key = timer_data_itr->first;

        std::string value = std::to_string(
          static_cast<long double>(timer_data_itr->second));

        stat_strings[this->hashring->get(key)] +=
          metric_name + '.'
                      + timer_data_key
                      + " "
                      + value
                      + ts_suffix;
      }
      ++num_stats;
    }
    else if (gauge)
    {
      std::string metric_name = this->prefix + fqdn_prefix;
      if (this->use_metric_type_prefix) {
        metric_name = metric_name + "gauges.";
      }
      metric_name = metric_name + this->process_name(key);

      std::string value = std::to_string(
        static_cast<long double>(gauge->gauge_));

      stat_strings[this->hashring->get(key)] +=
        metric_name + " "
                    + value
                    + ts_suffix;

      ++num_stats;
    }
  }

  // sets
  for (auto set_itr = ledger.sets.cbegin();
      set_itr != ledger.sets.cend();
      ++set_itr) {
    std::string key = set_itr->first;
    auto value = set_itr->second;
    std::string metric_name = this->prefix + fqdn_prefix;
    if (this->use_metric_type_prefix) {
      metric_name = metric_name + "sets.";
    }
    metric_name = metric_name + this->process_name(key);

    stat_strings[this->hashring->get(key)] +=
      metric_name + ".count"
                  + " "
                  + std::to_string(static_cast<long long int>(value.size()))
                  + ts_suffix;
     ++num_stats;
  }

  // stats
  std::string num_stats_str =
    std::to_string(static_cast<long long int>(num_stats));

  std::string total_time =
    std::to_string(
      static_cast<long long unsigned int>(chrono::unixtime_ms() - start_time));

  std::string key = fqdn_prefix + this->prefix_stats + ".num_stats";
  stat_strings[this->hashring->get(key)] +=
    key + " "
        + num_stats_str
        + ts_suffix;

  key = fqdn_prefix + this->prefix_stats + ".carbon_stats.calculation_time";
  stat_strings[this->hashring->get(key)] +=
    key + " "
        + total_time
        + ts_suffix;

  for (auto statsd_metric_itr = ledger.statsd_metrics.cbegin();
      statsd_metric_itr != ledger.statsd_metrics.cend();
      ++statsd_metric_itr) {
    std::string key = fqdn_prefix + this->prefix_stats + '.' + statsd_metric_itr->first;

    std::string value = std::to_string(
      static_cast<long long int>(statsd_metric_itr->second));

    stat_strings[this->hashring->get(key)] +=
      key + " "
          + value
          + ts_suffix;
  }

  for (auto stat_string_pair_itr = stat_strings.cbegin();
      stat_string_pair_itr != stat_strings.cend();
      ++stat_string_pair_itr) {
    auto& dest = stat_string_pair_itr->first;
    auto& stat_string = stat_string_pair_itr->second;

    auto itr = this->connections.find(dest);
    if (itr == this->connections.end()) {
      this->connections[dest] = std::move(net::Connection(dest));
      (this->connections[dest]).write(stat_string);
    } else {
      itr->second.write(stat_string);
    }
  }
}

}  // namespace backends
}  // namespace statsdcc
