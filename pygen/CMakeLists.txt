set(PYGEN_OUTPUTS
  ${CMAKE_CURRENT_SOURCE_DIR}/../movutl/generated/generated_props.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../movutl/generated/generated_props.cpp
)

add_custom_target(
  pygen
  COMMAND python3.13 main.py
  WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/
  COMMENT "movutlのプロパティのパースやLua bindingコードを生成しています"
)

# add_dependencies(mucore pygen)

