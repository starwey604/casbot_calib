target_sources(app PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/bridge_app.c
  ${CMAKE_CURRENT_LIST_DIR}/qas_client.c
  ${CMAKE_CURRENT_LIST_DIR}/qas_protocol.c
  ${CMAKE_CURRENT_LIST_DIR}/rate_core.c
  ${CMAKE_CURRENT_LIST_DIR}/usb_transport.c
)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_LIST_DIR})
