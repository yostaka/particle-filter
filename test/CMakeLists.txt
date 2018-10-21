project(unitTests)

add_executable(unitTests
        ../src/particle_filter.cpp
        Tests.cpp)

target_link_libraries(unitTests
        gtest
        gtest_main
        pthread)
