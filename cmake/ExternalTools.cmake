include(FetchContent)


# Add third-party headers to include path. Note this is done with SYSTEM
# to disable potential compiler warnings

###############
# CPM Packages
###############
# CPMAddPackage("gh:/SFML/SFML@2.5.1")


### Dependency declaration ###
FetchContent_Declare(
  sfml
  GIT_REPOSITORY "https://github.com/SFML/SFML.git"
  GIT_TAG        2.5.1
  )
### Dependency population ###
# sfml
set(BUILD_SHARED_LIBS OFF)
set(SFML_BUILD_EXAMPLES OFF)
set(SFML_BUILD_DOC OFF)
set(SFML_BUILD_NETWORK OFF)
set(SFML_BUILD_AUDIO OFF)
set(SFML_BUILD_GRAPHICS ON)
set(SFML_BUILD_WINDOW ON)

FetchContent_GetProperties(sfml)
if(NOT sfml_POPULATED)
  FetchContent_Populate(sfml)
  add_subdirectory(${sfml_SOURCE_DIR})
endif()

include_directories(${SFML_INCLUDE_DIRS})
include_directories(SYSTEM ${sfml_SOURCE_DIR}/include)

find_package(OpenMP REQUIRED)

# FetchContent_GetProperties(OpenMP)
if(OpenMP_CXX_FOUND)
  message("OpenMP found and will use it!")
endif()

# FetchContent_GetProperties(OpenMP)
