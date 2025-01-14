
enable_testing()

file(GLOB test_files ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
add_executable(mutest ${test_files})
target_link_libraries(mutest mucore)
add_test(NAME run_tes COMMAND mutest)

set_target_properties(mutest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

