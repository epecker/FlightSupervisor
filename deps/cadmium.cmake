include(FetchContent)

FetchContent_Declare(
  cadmium
  URL https://codeload.github.com/SimulationEverywhere/cadmium/zip/refs/heads/master
)

FetchContent_GetProperties(cadmium)
if(NOT cadmium_POPULATED)
  FetchContent_Populate(cadmium)
endif()

set(CADMIUM_INCLUDE_DIR ${FETCHCONTENT_BASE_DIR}/cadmium-src/include CACHE STRING "Cadmium Include File Location")