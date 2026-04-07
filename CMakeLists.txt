zephyr_include_directories(include)

if(CONFIG_ZMK_INPUT_PROCESSOR_ROTATE)
  target_sources(app PRIVATE src/input_processor_rotate.c)
endif()

if(CONFIG_ZMK_BEHAVIOR_ROTATE_ADJUST)
  target_sources(app PRIVATE src/behavior_rotate_adjust.c)
endif()
