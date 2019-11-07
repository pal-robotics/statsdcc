add_library (config configs/config.cc)
add_library (backend_config configs/aggregator_config.cc)
add_library (proxy_config configs/proxy_config.cc)

add_library (aggregator_worker workers/aggregator/worker.cc)
add_library (proxy_worker workers/proxy/worker.cc)
add_library (ledger ledger.cc)

add_executable (statsdcc aggregator.cc)

# link libraries
target_link_libraries (statsdcc
  tcp_server
  udp_server
  ros_server
  aggregator_http_server
  status
  aggregator_consumer
  aggregator_worker
  ledger
  carbon
  stdout
  repeater
  hashring
  statsdccnet
  backend_config
  config
  logger
  ${CRYPTO}
  ${JSONCPP}
  ${BOOST_REGEX}
  ${TCMALLOC_MINIMAL}
  ${MICROHTTPD}
  ${catkin_LIBRARIES}
  pthread
)

add_executable (proxy proxy.cc)
target_link_libraries (proxy
  proxy_http_server
  tcp_server
  udp_server
  ros_server
  status
  statsdccnet
  proxy_worker
  proxy_config
  statsdccnet
  config
  logger
  ${JSONCPP}
  ${BOOST_REGEX}
  ${TCMALLOC_MINIMAL}
  ${MICROHTTPD}
  pthread
)


#############
## Install ##
#############

install(
    TARGETS
        config backend_config proxy_config aggregator_worker
        proxy_worker ledger statsdcc proxy
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
