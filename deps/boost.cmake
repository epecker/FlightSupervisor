include(FetchContent)

FetchContent_Declare(
  boost
  URL https://boostorg.jfrog.io/artifactory/main/release/1.77.0/source/boost_1_77_0.zip
  URL_HASH SHA256=d2886ceff60c35fc6dc9120e8faa960c1e9535f2d7ce447469eae9836110ea77
)

FetchContent_GetProperties(boost)
if(NOT boost_POPULATED)
  FetchContent_Populate(boost)
endif()

set(BOOST_INCLUDE_DIR ${FETCHCONTENT_BASE_DIR}/boost-src CACHE STRING "Boost Include File Location")