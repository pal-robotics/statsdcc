#include "gtest/gtest.h"
#include "statsdcc/configs/aggregator_config.h"
#include "statsdcc/ledger.h"
#include "statsdcc/logger.h"

std::shared_ptr<statsdcc::Logger> logger;
std::unique_ptr<statsdcc::configs::AggregatorConfig> config;

namespace statsdcc {

class LedgerTest: public ::testing::Test {
 protected:
  virtual void SetUp() {
    logger = statsdcc::Logger::get_logger();

    ::config.reset(new configs::AggregatorConfig());
    ::config->name = "test";
    ::config->percentiles.push_back(90);
    ::config->frequency = 10;

    // buffer counter, without sampling
    ledger.buffer("counter:1|c");

    // buffer counter, with sampling
    ledger.buffer("counter_with_sampling:1|c|@0.1");

    // increment counter, without sampling
    ledger.buffer("increment_counter:1|c");
    ledger.buffer("increment_counter:4|c");

    // increment counter, with sampling
    ledger.buffer("increment_counter_with_sampling:1|c|@0.1");
    ledger.buffer("increment_counter_with_sampling:4|c");

    // buffer simple timer
    ledger.buffer("timer:320|ms");
    ledger.buffer("timer:320.00|ms");
    ledger.buffer("timer:36.56|ms");

    // buffer timer with sampling
    ledger.buffer("timer_with_sampling:32.5|ms|@0.1");
    ledger.buffer("timer_with_sampling:32.5|ms|@0.25");

    // buffer simple gauge
    ledger.buffer("gauge:333|g");

    // add and subtract gauge
    ledger.buffer("math:500|g");
    ledger.buffer("math:-100|g");
    ledger.buffer("math:-0.5|g");

    // sets
    ledger.buffer("uniques:765|s");
    ledger.buffer("uniques:765|s");
    ledger.buffer("uniques:900|s");
    ledger.buffer("uniques:setval|s");
    ledger.buffer("setval:44.07|s");

    // bad lines
    ledger.buffer("counter:1|");
    ledger.buffer("1|c");
    ledger.buffer("timer_with_sampling:bad|ms|@0.1");
    ledger.buffer("gauge:33_3|g");
    ledger.buffer("gauge:33 3|g");
    ledger.buffer("90");

    // initialize for processing
    ledger.buffer("counter_rate:10000|c");

    ledger.buffer("timer_data:1|ms");
    ledger.buffer("timer_data:2|ms");
    ledger.buffer("timer_data:3|ms");
    ledger.buffer("timer_data:4|ms");
    ledger.buffer("timer_data:5|ms");
    ledger.buffer("timer_data:6|ms");
    ledger.buffer("timer_data:7|ms");
    ledger.buffer("timer_data:8|ms");
    ledger.buffer("timer_data:9|ms");
    ledger.buffer("timer_data:10|ms");

    ledger.process();
  }

  virtual void TearDown() {
    ::config.reset();
  }

  double getCounter(const std::string &name)
  {
    auto &metric = ledger.metrics[name];
    Counter* counter = dynamic_cast<Counter*>(metric.get());
    return counter->counter_;
  }
  double getCounterRate(const std::string &name)
  {
    auto &metric = ledger.metrics[name];
    Counter* counter = dynamic_cast<Counter*>(metric.get());
    return counter->counter_rate_;
  }
  std::vector<double> getTimer(const std::string &name)
  {
    auto &metric = ledger.metrics[name];
    FullTimer* timer = dynamic_cast<FullTimer*>(metric.get());
    return timer->timers_;
  }
  std::unordered_map<std::string, double> getTimerData(const std::string &name)
  {
    auto &metric = ledger.metrics[name];
    FullTimer* timer = dynamic_cast<FullTimer*>(metric.get());
    return timer->timer_data_;
  }
  double getTimerCounter(const std::string &name)
  {
    auto &metric = ledger.metrics[name];
    FullTimer* timer = dynamic_cast<FullTimer*>(metric.get());
    return timer->counter_;
  }
  double getGauge(const std::string &name)
  {
    auto &metric = ledger.metrics[name];
    Gauge* gauge = dynamic_cast<Gauge*>(metric.get());
    return gauge->gauge_;
  }


  Ledger ledger;

  // copies of Ledger instance variables
  std::unordered_map<std::string, double> counters;
  std::unordered_map<std::string, std::vector<double> > timers;
  std::unordered_map<std::string, double> timer_counters;
  std::unordered_map<std::string, double> gauges;
  std::unordered_map<std::string, std::unordered_set<std::string> > sets;

  std::unordered_map<std::string, double> counter_rates;

  std::unordered_map<
    std::string, std::unordered_map<std::string, double>
  > timer_data;
};

TEST_F(LedgerTest, simple_counter) {
  EXPECT_EQ(1, getCounter("counter"));
}

TEST_F(LedgerTest, counter_with_sampling) {
  EXPECT_EQ(10, getCounter("counter_with_sampling"));
}

TEST_F(LedgerTest, increment_counter) {
  EXPECT_EQ(5, getCounter("increment_counter"));
}

TEST_F(LedgerTest, increment_counter_with_sampling) {
  EXPECT_EQ(14, getCounter("increment_counter_with_sampling"));
}

TEST_F(LedgerTest, simple_timer) {
  EXPECT_EQ(320, getTimer("timer")[0]);
  EXPECT_EQ(320, getTimer("timer")[1]);
  EXPECT_EQ(36.56, getTimer("timer")[2]);
}

TEST_F(LedgerTest, timer_with_sampling) {
  EXPECT_EQ(32.5, getTimer("timer_with_sampling")[0]);
  EXPECT_EQ(14, getTimerCounter("timer_with_sampling"));
}

TEST_F(LedgerTest, simple_gauge) {
  EXPECT_EQ(333, getGauge("gauge"));
}

TEST_F(LedgerTest, math_gauge) {
  EXPECT_EQ(399.5, getGauge("math"));
}

TEST_F(LedgerTest, sets) {
//  EXPECT_EQ(1, sets["uniques"].count("765"));
//  EXPECT_EQ(1, sets["uniques"].count("900"));
//  EXPECT_EQ(1, sets["uniques"].count("setval"));
//  EXPECT_EQ(1, sets["setval"].count("44.07"));
}

TEST_F(LedgerTest, bad_lines) {
  EXPECT_EQ(6, getCounter(config->name + ".bad_lines_seen"));
}

TEST_F(LedgerTest, counter_rate) {
  EXPECT_EQ(1000, getCounterRate("counter_rate"));
}

TEST_F(LedgerTest, timer_data) {
  EXPECT_EQ(10, getTimerData("timer_data")["upper"]);
  EXPECT_EQ(1, getTimerData("timer_data")["lower"]);
  EXPECT_EQ(10, getTimerData("timer_data")["count"]);
  EXPECT_EQ(5.5, getTimerData("timer_data")["mean"]);
  EXPECT_EQ(9, getTimerData("timer_data")["upper_90"]);
  EXPECT_EQ(9, getTimerData("timer_data")["count_90"]);
  EXPECT_EQ(45, getTimerData("timer_data")["sum_90"]);
  EXPECT_EQ(5, getTimerData("timer_data")["mean_90"]);
}

}  // namespace statsdcc
