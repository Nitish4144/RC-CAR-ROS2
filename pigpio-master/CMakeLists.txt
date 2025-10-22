cmake_minimum_required(VERSION 3.10)
project(pigpio LANGUAGES C VERSION 0.71)

# Enable threading
find_package(Threads REQUIRED)

# librt is part of glibc on modern Linux — no FindRT needed
# Simply link using "rt"
set(RT_LIBRARIES rt)

option(BUILD_SHARED_LIBS "Build shared libraries" ON)
add_compile_options(-Wall)

# ------------------------------------------------------------------------------
# Libraries
# ------------------------------------------------------------------------------

add_library(pigpio pigpio.c command.c custom.cext)
add_library(pigpiod_if pigpiod_if.c command.c)
add_library(pigpiod_if2 pigpiod_if2.c command.c)

# ------------------------------------------------------------------------------
# Executables
# ------------------------------------------------------------------------------

add_executable(x_pigpio x_pigpio.c)
target_link_libraries(x_pigpio pigpio ${RT_LIBRARIES} Threads::Threads)

add_executable(x_pigpiod_if x_pigpiod_if.c)
target_link_libraries(x_pigpiod_if pigpiod_if ${RT_LIBRARIES} Threads::Threads)

add_executable(x_pigpiod_if2 x_pigpiod_if2.c)
target_link_libraries(x_pigpiod_if2 pigpiod_if2 ${RT_LIBRARIES} Threads::Threads)

add_executable(pigpiod pigpiod.c)
target_link_libraries(pigpiod pigpio ${RT_LIBRARIES} Threads::Threads)

add_executable(pigs pigs.c command.c)
target_link_libraries(pigs Threads::Threads)

add_executable(pig2vcd pig2vcd.c command.c)
target_link_libraries(pig2vcd Threads::Threads)

# ------------------------------------------------------------------------------
# Installation
# ------------------------------------------------------------------------------

include(GNUInstallDirs)

install(TARGETS
  pigpio pigpiod_if pigpiod_if2 pigpiod pigs pig2vcd
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES pigpio.h pigpiod_if.h pigpiod_if2.h
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ
)

# ------------------------------------------------------------------------------
# Manual pages
# ------------------------------------------------------------------------------

file(GLOB man_1_SRC "*.1")
install(FILES ${man_1_SRC}
  DESTINATION share/man/man1
  PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ
)

file(GLOB man_3_SRC "*.3")
install(FILES ${man_3_SRC}
  DESTINATION share/man/man3
  PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ
)

# ------------------------------------------------------------------------------
# Python module (safe install for workspace)
# ------------------------------------------------------------------------------

find_package(Python3 COMPONENTS Interpreter QUIET)

if(Python3_FOUND)
  configure_file(${CMAKE_CURRENT_LIST_DIR}/cmake/setup.py.in
                 ${CMAKE_CURRENT_BINARY_DIR}/setup.py COPYONLY)
  # Install into workspace, not system
  install(CODE "
    execute_process(
      COMMAND ${Python3_EXECUTABLE} -m pip install --no-warn-script-location --prefix=\${CMAKE_INSTALL_PREFIX} .
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    )
  ")
endif()

# ------------------------------------------------------------------------------
# Package export (optional for external CMake use)
# ------------------------------------------------------------------------------

include(CMakePackageConfigHelpers)
include(GenerateExportHeader)

generate_export_header(${PROJECT_NAME})
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

include(CPack)
