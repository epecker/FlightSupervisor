include(FetchContent)

FetchContent_Declare(
  destimes
  URL https://github.com/SimulationEverywhere/DESTimes/archive/refs/heads/master.zip
)

FetchContent_GetProperties(destimes)
if(NOT destimes_POPULATED)
  FetchContent_Populate(destimes)
endif()

set(DESTIMES_INCLUDE_DIR "${FETCHCONTENT_BASE_DIR}/destimes-src/include" CACHE STRING "DESTimes Include File Location")