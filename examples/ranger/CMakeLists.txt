add_executable(ranger_mini_v3_state ranger_mini_v3_state.cpp)
target_link_libraries(ranger_mini_v3_state PRIVATE agilex_ugv_sdk::agilex_ugv_sdk)
set_target_properties(ranger_mini_v3_state PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
)
