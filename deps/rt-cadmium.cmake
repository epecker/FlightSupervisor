include(FetchContent)

FetchContent_Declare(
  rt-cadmium
  URL https://codeload.github.com/KyleBjornson/cadmium/zip/refs/heads/master
)

FetchContent_GetProperties(rt-cadmium)
if(NOT rt-cadmium_POPULATED)
  FetchContent_Populate(rt-cadmium)
endif()

set(RT_CADMIUM_INCLUDE_DIR ${FETCHCONTENT_BASE_DIR}/cadmium-src/include CACHE STRING "RT-Cadmium Include File Location")