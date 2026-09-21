
file(GLOB test_files ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
add_executable(mutest ${test_files})
target_link_libraries(mutest mucore)
# mutestは相対パス(../assets等)で素材を参照するためbuild/で実行する
add_test(NAME mutest COMMAND mutest WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

set_target_properties(mutest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

