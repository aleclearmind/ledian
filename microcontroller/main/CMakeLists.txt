idf_component_register(
    SRCS
    #ledc_fade_example_main.c
    main.cpp
    blinker.cpp
    Command.cpp
    Colors.cpp
    renderer.cpp
    led_strip_encoder.c
    command_parser.cpp
    INCLUDE_DIRS ".")
#    command_parser.include(CPackProductBuild)
