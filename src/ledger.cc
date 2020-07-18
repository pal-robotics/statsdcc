
/**
 * Source file for class Ledger
 * Please see Ledger.h for documentation
 */

#include "statsdcc/ledger.h"

#include <boost/lexical_cast.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>

#include "statsdcc/chrono.h"
#include "statsdcc/configs/aggregator_config.h"
#include "statsdcc/logger.h"

namespace statsdcc {

// buffers the metric
void Ledger::buffer(const std::string& metric) {
  const char *metric_csty = metric.c_str();

  char name_buffer[MAX_TOKEN_BUFFER_SIZE] = {0};
  char type_buffer[3] = {0};
  char metric_value_buffer[MAX_TOKEN_BUFFER_SIZE] = {0};
  double metric_value = -std::numeric_limits<double>::max();
  double sample_rate = 1;
  MetricType type = MetricType::counter;

  // record number of calls to buffer method
  ++this->statsd_metrics["metrics_processed"];

  sscanf(metric_csty,
         "%" MAX_TOKEN_STRING_LENGTH "[^:]:" \
         "%" MAX_TOKEN_STRING_LENGTH "[^|]|" \
         "%2[^|]|"                           \
         "@%lf",
         name_buffer, metric_value_buffer, type_buffer, &sample_rate);

  // trim leading spaces if any
  char *tmp_type = type_buffer;
  while (*tmp_type == ' ' && *tmp_type != '\0')
    ++tmp_type;
  std::string metric_name(std::move(name_buffer));
  std::string metric_type_str(tmp_type);

  // track bad lines
  // using find() intead of == so that we can ignore trailing spaces
  bool bad_line = false;
  if ("" == metric_type_str) {
    bad_line = true;
  } else if (metric_type_str.find("ms") == 0) {
    type = MetricType::timer;
    try {
      metric_value = boost::lexical_cast<double>(metric_value_buffer);
    } catch(...) {
      bad_line = true;
    }
  } else if (metric_type_str.find("c") == 0) {
    type = MetricType::counter;
    try {
      metric_value = boost::lexical_cast<double>(metric_value_buffer);
    } catch(...) {
      bad_line = true;
    }
  } else if (metric_type_str.find("g") == 0) {
    type = MetricType::gauge;
    try {
      metric_value = boost::lexical_cast<double>(metric_value_buffer);
    } catch(...) {
      bad_line = true;
    }
  } else if (metric_type_str.find("s") == 0) {
    type = MetricType::set;
    bad_line = false;
  } else {
    try {
      metric_value = boost::lexical_cast<double>(metric_value_buffer);
    } catch(...) {
      bad_line = true;
    }
  }

  if (bad_line) {
    ::logger->info("Bad line: " + metric);
    // setup the names for the stats stored in counters
    std::string bad_lines_seen = ::config->name + ".bad_lines_seen";
    addBadLine(bad_lines_seen);
    return;
  }

  // ++this->frequency[metric_name];
  auto metric_it = metrics.find(metric_name);
  if (metric_it != metrics.end())
  {
    if (type == MetricType::gauge)
    {
      Gauge *gauge = dynamic_cast<Gauge *>(metric_it->second.get());
      // check if +/- is specified
      char char_after_colon = metric_csty[metric.find_first_of(":") + 1];
      if (('+' == char_after_colon) || ('-' == char_after_colon))
      {
        gauge->incrementalUpdate(metric_value, sample_rate);
      }
      else
      {
        gauge->update(metric_value, sample_rate);
      }
    }
    else
    {
      buffer(metric_it->second, metric_value, sample_rate);
    }
  }
  else
  {
    switch (type)
    {
      case MetricType::timer:
      {
        if (::config->percentiles.empty())
        {
          // This timer cannot be used to compute percecntiles, but has a much smaller memory footprint
          auto ret = metrics.emplace(metric_name, std::shared_ptr<Metric>(new LeanTimer));
          ret.first->second->update(metric_value, sample_rate);
        }
        else {
          auto ret = metrics.emplace(metric_name, std::shared_ptr<Metric>(new FullTimer));
          ret.first->second->update(metric_value, sample_rate);
        }
        break;
      }
      case MetricType::gauge:
      {
        auto ret = metrics.emplace(metric_name, std::shared_ptr<Metric>(new Gauge));
        ret.first->second->update(metric_value, sample_rate);
        break;
      }
      case MetricType::set:
        this->sets[metric_name].insert(metric_value_buffer);
        break;

      default:
        auto ret = metrics.emplace(metric_name, std::shared_ptr<Metric>(new Counter));
        ret.first->second->update(metric_value, sample_rate);
        break;
    }
  }
}

void Ledger::buffer(const std::shared_ptr<Metric> &metric, double metric_value, double sample_rate)
{
  metric->update(metric_value, sample_rate);
}

std::shared_ptr<Metric> Ledger::buffer(const std::string &metric_name, double metric_value,
                    const std::string &metric_type)
{
  MetricType type = MetricType::counter;
  double sample_rate = 1;

  // record number of calls to buffer method
  // ++this->statsd_metrics["metrics_processed"];

  // track bad lines
  bool bad_line = false;
  if (metric_type == "ms")
  {
    type = MetricType::timer;
  }
  else if (metric_type == "c")
  {
    type = MetricType::counter;
  }
  else if (metric_type == "g")
  {
    type = MetricType::gauge;
  }
  else if (metric_type == "s")
  {
    type = MetricType::set;
  }
  else
  {
    bad_line = true;
  }

  if (bad_line) {
    ::logger->info("Bad line: " + metric_name + "|" + metric_type);
    // setup the names for the stats stored in counters
    std::string bad_lines_seen = ::config->name + ".bad_lines_seen";
    addBadLine(bad_lines_seen);
    return nullptr;
  }

  // ++this->frequency[metric_name];

  auto metric_it = metrics.find(metric_name);
  if (metric_it != metrics.end())
  {
    buffer(metric_it->second, metric_value);
    return metric_it->second;
  }
  else
  {
  switch (type) {
    case MetricType::timer:
    {
      std::shared_ptr<Metric> ptr;
      if (::config->percentiles.empty())
      {
        // This timer cannot be used to compute percecntiles, but has a much smaller memory footprint
        ptr = std::make_shared<LeanTimer>();
      }
      else
      {
        ptr = std::make_shared<FullTimer>();
      }
      metrics[metric_name] = ptr;
      ptr->update(metric_value, sample_rate);
      return ptr;
    }
    case MetricType::gauge:
      {
        /// @todo +/- is specified
        auto ptr = std::make_shared<Gauge>();
        metrics[metric_name] = ptr;
        ptr->update(metric_value, sample_rate);
        return ptr;
      }
    case MetricType::set:
      /// @todo sets
      return  nullptr;

    default:
      auto ptr = std::make_shared<Counter>();
      metrics[metric_name] = ptr;
      ptr->update(metric_value, sample_rate);
      return ptr;
  }
  }
}

// Aggregates buffered metrics
void Ledger::process() {
  std::uint64_t start_time = chrono::unixtime_ms();

  // process counters
  // calculate "per second" rate

  for (auto &m : this->metrics) {
    const std::string &key = m.first;
    Metric* metric = m.second.get();
    Counter* counter = dynamic_cast<Counter*>(metric);
    FullTimer* timer = dynamic_cast<FullTimer*>(metric);
    LeanTimer* lean_timer = dynamic_cast<LeanTimer*>(metric);
    if (counter)
    {
      counter->counter_rate_ = counter->counter_ / ::config->frequency;
    }
    else if (timer)
    {
    // process timers
    std::unordered_map<std::string, double> &current_timer_data = timer->timer_data_;

    if (key.length() <= 0) {
      current_timer_data["count"] = current_timer_data["count_ps"] = 0;
    } else {
      // get sorted values
      std::vector<double> values(timer->timers_);
//      std::sort(values.begin(), values.end());

      // get count, sum, mean, min, and max
      int count = values.size();
      double sum = 0;
      auto minmax = std::minmax_element(values.begin(), values.end());
      double min = *minmax.first;
      double max = *minmax.second;
      for (auto value_itr = values.cbegin();
          value_itr != values.cend();
          ++value_itr) {
        sum += *value_itr;
      }
      double mean = sum / count;

      // initialize sum, mean, and threshold boundary
      double threshold_boundary = max;

      for (auto threshold_itr = ::config->percentiles.cbegin();
          threshold_itr != ::config->percentiles.cend();
          ++threshold_itr) {
        double pct =
          (*threshold_itr < 0) ? -(*threshold_itr) : (*threshold_itr);
         // Int is potentially too short since we could have more than 65K metrics in the flush window
         unsigned long num_in_threshold = 0;

        if (count > 1) {
          num_in_threshold = round(pct / 100 * count);
          if (0 == num_in_threshold) {
            continue;
          }
          threshold_boundary = values[num_in_threshold - 1];
        }

        double pct_sum = 0;
        for (unsigned long i = 0; i < num_in_threshold; ++i) {
            pct_sum += values[i];
        }
        double pct_mean = pct_sum / num_in_threshold;

        // generate pct name
        char clean_pct[17] = {0};
        snprintf(clean_pct, sizeof(clean_pct), "%g", pct);
        for (unsigned int i = 0; i < strlen(clean_pct); ++i) {
          clean_pct[i] = ('.' == clean_pct[i]) ? '_' : clean_pct[i];
        }

        current_timer_data[std::string("upper_") + std::string(clean_pct)] = threshold_boundary;
        current_timer_data[std::string("count_") + std::string(clean_pct)] = num_in_threshold;
        current_timer_data[std::string("sum_") + std::string(clean_pct)] = pct_sum;
        current_timer_data[std::string("mean_") + std::string(clean_pct)] = pct_mean;

      }  // foreach percentile

      current_timer_data["upper"] = max;
      current_timer_data["lower"] = min;
      current_timer_data["count"] = timer->counter_;
      current_timer_data["mean"] = mean;
      timer->timer_data_ = current_timer_data;
    }
    }
    else if (lean_timer)
    {
      std::unordered_map<std::string, double> &current_timer_data = lean_timer->timer_data_;

      if (key.length() <= 0)
      {
        current_timer_data["count"] = current_timer_data["count_ps"] = 0;
      }
      else
      {
        current_timer_data["upper"] = (!std::isinf(lean_timer->max_) ?
                                           lean_timer->max_ :
                                           std::numeric_limits<double>::quiet_NaN());
        current_timer_data["lower"] = (!std::isinf(lean_timer->min_) ?
                                         lean_timer->min_ :
                                         std::numeric_limits<double>::quiet_NaN());
        current_timer_data["count"] = lean_timer->counter_;
        current_timer_data["mean"] = lean_timer->sum_ / double(lean_timer->count_);
      }
      lean_timer->timer_data_ = current_timer_data;
    }
  }  // foreach metric
  this->statsd_metrics["processing_time"] =
    chrono::unixtime_ms() - start_time;
}

void Ledger::setProcTime(std::int64_t value)
{
  this->statsd_metrics["metrics_processing_time"] =
      this->statsd_metrics["metrics_processing_time"] + value;
}

void Ledger::addBadLine(const std::string &name)
{
  // Only emplaces first time
  auto metric_it = metrics.find(name);
  if (metric_it != metrics.end())
  {
    metric_it->second->update(1, 1);
  }
  else
  {
    auto ret = metrics.emplace(name, std::shared_ptr<Metric>(new Counter));
    ret.first->second->update(1, 1);
  }
}

}  // namespace statsdcc
