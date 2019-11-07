add_library (statsdccnet net/lib.cc)

add_library (tcp_server net/servers/socket/tcp_server.cc)

add_library (udp_server net/servers/socket/udp_server.cc)

add_library (ros_server net/servers/socket/ros_server.cc)
target_link_libraries(ros_server ${catkin_LIBRARIES} ledger)
add_dependencies(ros_server ledger)

add_library (aggregator_http_server net/servers/http/aggregator/http_server.cc)
add_library (proxy_http_server net/servers/http/proxy/http_server.cc)

add_library (carbon backends/carbon.cc)
add_library (stdout backends/stdout.cc)
add_library (repeater backends/repeater.cc)

add_library (aggregator_consumer consumers/aggregator_consumer.cc)

add_library (hashring hashring/hashring.cc)

add_library (logger logger.cc)

add_library (status status.cc)


#############
## Install ##
#############

install(
    TARGETS
        statsdccnet tcp_server udp_server ros_server aggregator_http_server proxy_http_server
        carbon stdout repeater aggregator_consumer hashring logger status
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

