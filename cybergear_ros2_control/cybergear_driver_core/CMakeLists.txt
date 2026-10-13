cmake_minimum_required(VERSION 3.15)

set(CYBERGEAR_DRIVER_CORE_MAJOR_VERSION 0)
set(CYBERGEAR_DRIVER_CORE_MINOR_VERSION 0)
set(CYBERGEAR_DRIVER_CORE_PATCH_VERSION 0)
set(CYBERGEAR_DRIVER_CORE_VERSION
  ${CYBERGEAR_DRIVER_CORE_MAJOR_VERSION}.${CYBERGEAR_DRIVER_CORE_MINOR_VERSION}.${CYBERGEAR_DRIVER_CORE_PATCH_VERSION})

project(cybergear_driver_core
  VERSION ${CYBERGEAR_DRIVER_CORE_VERSION}
  LANGUAGES
    CXX
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR_IN_INTERFACE ON)

set(PUBLIC_HEADERS
  include/cybergear_driver_core/cybergear_packet_param.hpp
  include/cybergear_driver_core/cybergear_frame_id.hpp
  include/cybergear_driver_core/cybergear_driver_core.hpp
  include/cybergear_driver_core/protocol_constant.hpp
  include/cybergear_driver_core/bounded_float_byte_converter.hpp
  include/cybergear_driver_core/cybergear_packet.hpp
  include/cybergear_driver_core/scaled_float_byte_converter.hpp
)
add_library(${PROJECT_NAME} INTERFACE)
set_target_properties(${PROJECT_NAME}
  PROPERTIES
    PUBLIC_HEADER "${PUBLIC_HEADERS}"
)
target_include_directories(${PROJECT_NAME}
  INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME}
  INTERFACE
    cxx_std_20
)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

export(
  TARGETS ${PROJECT_NAME}
  FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake
)
install(
  TARGETS
    ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)
install(
  EXPORT ${PROJECT_NAME}Targets
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/cmake/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}ConfigVersion.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/cmake/${PROJECT_NAME}ConfigVersion.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
)
install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/cmake/${PROJECT_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/cmake/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
)

set(CPACK_PACKAGE_NAME ${PROJECT_NAME})
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_RESOURCE_FILE_LICENSE ${CMAKE_CURRENT_SOURCE_DIR}/LICENSE)

include(CPack)
