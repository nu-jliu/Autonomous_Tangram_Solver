# Lines that begin with a # are comments
# set the minimum required version of cmake, usually the first line
cmake_minimum_required(VERSION 3.22)

# project_name sets the name of the project and causes cmake to
# find the c and c++ compilers
project(tangram_utils)

# Find your dependencies.
# Many libraries ship with files that allow CMake to find them
# Then general behavior is to call "find_package" but the options
# provided are package specific.  Usually there is then a CMAKE variable
# That is defined to reference the library
# here: we find the eigen library as per the instruction
# https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html
# find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV 4.5 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

# Create a library.  Can specify if it is shared or static but usually
# you don't need or want to.
# name is the name of the library without the extension or lib prefix
# name creates a cmake "target"
file(GLOB SRC_FILES src/*.cpp)

add_library(${PROJECT_NAME} 
    ${SRC_FILES}
)

# Create an executable from the following source code files
# The Name of the executable creates a cmake "target"
# add_executable(frame_main src/frame_main.cpp)

# specify additional compilation flags for the library
# Public causes the flags to propagate to anything
# that links against this library
target_compile_options(${PROJECT_NAME} PUBLIC -O3 -Wall -Wextra -pedantic)
# target_compile_options(frame_main PUBLIC -Wall -Wextra -pedantic)

# Enable c++17 support.
# Public causes the features to propagate to anything
# that links against this library
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# Use target_include_directories so that #include"mylibrary/header.hpp" works
# The use of the <BUILD_INTERFACE> and <INSTALL_INTERFACE> is because when
# Using the library from the build directory or after installation
# During build, the headers are read from the source code directory
# When used from the installed location, headers are in the
# system include/ directory
target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Use target_link_libraries to add dependencies to a "target"
# (e.g., a library or executable)
# This will automatically add all required library files
# that need to be linked
# and paths to th locations of header files
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
# target_link_libraries(frame_main ${PROJECT_NAME})

# install the include files by copying the whole include directory
install(DIRECTORY 
    include/${PROJECT_NAME} 
    DESTINATION include
)

# Create a CMake Exported Target containing the lib and exe.
# Also create CMake Export called projet_name-targets
# The CMake Export contains files that allow other CMake projects
# to find this project. It must be installed separately.
install(TARGETS 
    ${PROJECT_NAME} 
    EXPORT ${PROJECT_NAME}-targets
)

# The project_name-targets created by install(TARGETS) needs to be installed.
# install(EXPORT ...) will generate a file called project_name-config.cmake
# that contains the exported targets.
# After installation this file will then be found when calling
# find_package(project_name) from another cmake project
# A user can then target_link_libraries(target project_name::library)
# to use the libraries installed here
install(
    EXPORT ${PROJECT_NAME}-targets
    FILE ${PROJECT_NAME}-config.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION lib/cmake/${PROJECT_NAME}
)

if(NOT CMAKE_CROSSCOMPILING)
    include(CTest)

    # CTest sets BUILD_TESTING to on. To disable tests add -DBUILD_TESTING=OFF when invoking cmake
    if(BUILD_TESTING)
        # Find the Unit testing framework. In this example, Catch2
        find_package(Catch2 3 REQUIRED)

        file(GLOB TEST_SRC_FILES
            CMAKE_CONFIGURE_DEPENDS
            "tests/test_*.cpp"
        )

        foreach(TEST_SRC_FILE ${TEST_SRC_FILES})
            get_filename_component(TEST_NAME ${TEST_SRC_FILE} NAME_WE)
            message(TEST_SRC_FILE=${TEST_SRC_FILE})

            # A test is just an executable that is linked against the unit testing library
            add_executable(${TEST_NAME} ${TEST_SRC_FILE})
            
            target_link_libraries(${TEST_NAME} 
                ${PROJECT_NAME} 
                Catch2::Catch2WithMain
            )

            # register the test with CTest, telling it what executable to run
            add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
        endforeach()
    endif()

    # Building documentation should be optional.
    # To build documentation pass -DBUILD_DOCS=ON when generating the build system
    option(BUILD_DOCS "Build the documentation" OFF)

    find_package(Doxygen)
    # build just because Doxygen is missing
    if(${DOXYGEN_FOUND} AND ${BUILD_DOCS})

        # Turn the README.md into the homepage of the doxygen docs
        set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)

        # Tell Doxygen where to find the documentation
        doxygen_add_docs(doxygen include/ src/ README.md ALL)

        # The documentation will be in the build/html directory
        # The main page is build/html/index.html
    endif()
endif()