add_library(libimages STATIC
        libimages/algorithms/grayscale.cpp
        libimages/algorithms/morphology.cpp
        libimages/algorithms/threshold_masking.cpp
        libimages/debug_io.cpp
        libimages/image.cpp
        libimages/image_io.cpp
)

target_include_directories(libimages PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(libimages PUBLIC libbase PRIVATE third_party_stb)

if (BUILD_TESTING)
    add_executable(libimages_tests
            libimages/algorithms/grayscale_tests.cpp
            libimages/algorithms/morphology_tests.cpp
            libimages/algorithms/threshold_masking_tests.cpp
            libimages/debug_io_tests.cpp
            libimages/tests_utils.cpp
    )
    target_link_libraries(libimages_tests PRIVATE libimages GTest::gtest_main)
    add_test(NAME libimages_tests COMMAND libimages_tests)
endif ()
