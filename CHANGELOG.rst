^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package statsdcc
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.9 (2020-07-16)
------------------
* Merge branch 'perf-improve' into 'erbium-devel'
  Use only one map for timer metric to reduce hash find operations
  See merge request third-party/statsdcc!10
* Optimize Timer to avoid storing all recorded values
  In our use case, reduces memory footprint to 2% of the original
* Further improvements, but with memory leak
* Performance overhaul, use pal_statistics ordering to avoid map lookups
* Add time
* Use only one map for timer metric to reduce hash find operations
* Contributors: Victor Lopez, victor

0.0.8 (2019-11-07)
------------------
* Merge branch 'rename_net' into 'erbium-devel'
  Rename library with generic name 'net'
  See merge request third-party/statsdcc!9
* Renamed library with generic name 'net'
* Contributors: Jordan Palacios

0.0.7 (2019-08-28)
------------------
* Disable PAL flags
* Contributors: Victor Lopez

0.0.6 (2019-04-17)
------------------
* Merge branch 'optimized-msg' into 'erbium-devel'
  Optimized statistics msg
  See merge request third-party/statsdcc!8
* Changes to increase performance
* Adapted ros_server to new Statistics message format
* Renamed private member variables
* Cosmetics, formatting
* Contributors: Jordan Palacios, Victor Lopez

0.0.5 (2019-01-25)
------------------
* Merge branch 'clang-fixes' into 'erbium-devel'
  Clang fixes
  See merge request third-party/statsdcc!7
* Clang fixes
* Contributors: Victor Lopez

0.0.4 (2018-09-10)
------------------
* Merge branch 'increase_performance' into 'erbium-devel'
  Increase performance
  See merge request qa/statsdcc!6
* Replaced continue with break
* Added some @todo's
* Now processes the ledger in the separate thread too
* Increased workers sleep to 100 ns
* ros_server now has its own ledger and flushes directly to the backend
* Added introspection. Now logs the time for processing metrics
* Added introspection. Now logs the time for processing a callback
* Added unixtime_ns()
* Contributors: Jordan Palacios

0.0.3 (2018-08-08)
------------------
* Merge branch 'carbon_backend_stats' into 'erbium-devel'
  Cleanup carbon backend stats
  See merge request qa/statsdcc!5
* Add FQDN prefix to carbon backend stats
* Fixed names of some statistics
* Contributors: Jordan Palacios

0.0.2 (2018-07-31)
------------------
* Merge branch 'improve_performance' into 'erbium-devel'
  Improve performance
  See merge request qa/statsdcc!4
* New consume() function to avoid a find_first_of call for each metric
* Merge branch 'more_options' into 'erbium-devel'
  Add more configuration options
  See merge request qa/statsdcc!3
* Stats with no regex won't be looked up in each callback
* Disabled custom compile options: -g -03
* Add FQDN prefix option. Disabled by default
* Metric type prefix is now optional. Enabled by default
* Now allows aggregator configurations with no percentiles
* Contributors: Jordan Palacios

0.0.1 (2018-07-25)
------------------
* Merge branch 'ros_server' into 'erbium-devel'
  Add ros server
  See merge request qa/statsdcc!2
* Increased performance by caching regex matches results
* Added rule parsing, poorly optimized
* Added ros server
* Merge branch 'catkinize' into 'erbium-devel'
  Catkinize package
  See merge request qa/statsdcc!1
* Catkinized tests
  - Ledger test enabled
  - Hashring test disabled (original fails)
* Catkinized package
* Fixed initialization error: now initializes all members
* Fixed json.h includes
* increased buffer size to support 64k udp packets - fix missing metrics from a batch
* Merge pull request #3 from HBOCodeLabs/master
  Statsdcc <-> Statsdcc Compatibility Enhancements
* Merge pull request #1 from HBOCodeLabs/rrusso1982/timer_percentiles
  Statsdcc <-> statsd compatibility improvements
* Adjust loop to be long
* Adjusting the timer parecent threshold to a long
* Move period to right side
* metrics prefixes
* typo
* prefix in carbon header
* Adding prefix to config.h
* Update readme
* Adding prefix for emitted metrics
* Signed int comparion fix
* GitIgnore Updates
* Adding timer percentile calculations
* option for proxy to blacklist specific metrics
* removed patch that tries to match v8 sort
* statsdcc release
  Signed-off-by: Santosh Domalapalli <sdomalapalli@wayfair.com>
* Contributors: Jordan Palacios, Rob Russo, Russo, Robert (HBO), Santosh Domalapalli, sdomalap
