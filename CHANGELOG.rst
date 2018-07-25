^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package statsdcc
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
