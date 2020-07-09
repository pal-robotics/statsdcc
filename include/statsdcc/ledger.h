
/**
 * Definition of class Ledger
 */

#ifndef INCLUDE_STATSDCC_LEDGER_H_
#define INCLUDE_STATSDCC_LEDGER_H_

#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "statsdcc/configs/aggregator_config.h"

// <name>:<val>|<type> 1496 1  1  1  1  = 1500
#define MAX_TOKEN_BUFFER_SIZE 1497
#define MAX_TOKEN_STRING_LENGTH "1496"

namespace statsdcc {

namespace backends {
class Carbon;  // forward declare to avoid cyclic include
class Stdout;
class Repeater;
}

class Metric
{
public:
  virtual void update(double value, double sample_rate) = 0;
};

class Counter : public Metric
{
public:
  Counter() : counter_(0.0), counter_rate_(0.0)
  {
  }
  void update(double metric_value, double sample_rate) override
  {
    counter_ += metric_value * (1 / sample_rate);
  }
  double counter_;
  double counter_rate_;  // was unused
};

class Set : public Metric
{
public:
  void update(double metric_value, double sample_rate) override
  {
  }
};

class Gauge : public Metric
{
public:
  void update(double metric_value, double /*sample_rate*/) override
  {
    gauge_ = metric_value;
  }
  double gauge_;
};

class IncrementalGauge : public Metric
{
public:
  void update(double metric_value, double /*sample_rate*/) override
  {
    gauge_ += metric_value;
  }
  double gauge_;
};

class Timer : public Metric
{
public:
  Timer()
  {
    timer_data_.reserve(1000); // resizing this online consumes a significant amount of time
  }
  void update(double metric_value, double sample_rate) override
  {
    counter_ += metric_value * (1 / sample_rate);
    timers_.push_back(metric_value);
  }

  double counter_;
  std::vector<double> timers_;
  std::unordered_map<std::string, double> timer_data_;
};

/**
 * Parses the metric submitted in one of the following format,
 * into std collections for easy processing
 *   METRIC:METRIC_VALUE|c (counter)
 *   METRIC:METRIC_VALUE|c@SAMPLING_RATE (counter with sampling)
 *   METRIC:METRIC_VALUE|ms (timer)
 *   METRIC:METRIC_VALUE|ms@SAMPLING_RATE (timer with sampling)
 *   METRIC:METRIC_VALUE|g (gauge)
 *   METRIC:METRIC_VALUE|s (set)
 *
 * Also performs aggregation similar to https://github.com/etsy/statsd
 */
class Ledger {
  friend class backends::Carbon;
  friend class backends::Stdout;
  friend class backends::Repeater;
  friend class LedgerTest;  // unit tests

 private:
  enum class MetricType { counter, timer, gauge, set };

 public:
  inline Ledger() {}
  inline Ledger(const Ledger& ledger) = delete;
  inline Ledger& operator=(const Ledger& ledger) = delete;
  ~Ledger() = default;

  /**
   * Buffers the metric.
   *
   * @param metric The metric value to be buffered for processing later
   */
  void buffer(const std::string& metric);
  std::shared_ptr<Metric> buffer(const std::string& metric_name, double metric_value, const std::string& metric_type);
  void buffer(const std::shared_ptr<Metric> &metric, double metric_value);

  /**
   * Aggregates the metric values buffered by buffer method.
   */
  void process();

  inline int bad_lines_seen() {
    auto bad_lines_key = ::config->name + ".bad_lines_seen";
    return (this->metrics.find(bad_lines_key) != this->metrics.end())
      ? static_cast<int>(dynamic_cast<Counter*>(this->metrics[bad_lines_key].get())->counter_)
      : 0;
  }

  void setProcTime(std::int64_t value);

  std::unordered_map<std::string, long long int> frequency;

 private:
  struct TimerData
  {
    std::vector<double> timers;
    double timer_counter;
  };
  void addBadLine(const std::string &name);


  std::unordered_map<std::string, std::shared_ptr<Metric>> metrics;
  std::unordered_map<std::string, std::unordered_set<std::string> > sets;
  std::unordered_map<std::string, std::int64_t> statsd_metrics;
};

}  // namespace statsdcc

#endif  // INCLUDE_STATSDCC_LEDGER_H_
