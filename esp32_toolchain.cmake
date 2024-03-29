include(CMakeForceCompiler)

set(CMAKE_SYSTEM_NAME Generic)

set(idf_target "esp32")
set(idf_path "/home/jan/esp/v5.2.1/esp-idf")

if("${idf_target}" STREQUAL "esp32c3")
    set(CMAKE_SYSTEM_PROCESSOR riscv)
    set(FLAGS "-ffunction-sections -fdata-sections" CACHE STRING "" FORCE)
else()
    set(CMAKE_SYSTEM_PROCESSOR xtensa)
    set(FLAGS "-mlongcalls -ffunction-sections -fdata-sections" CACHE STRING "" FORCE)
endif()

set(CMAKE_CROSSCOMPILING 1)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(PLATFORM_NAME "LwIP")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

set(CMAKE_C_COMPILER /home/jan/.espressif/tools/xtensa-esp-elf/esp-13.2.0_20230928/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc)
set(CMAKE_CXX_COMPILER /home/jan/.espressif/tools/xtensa-esp-elf/esp-13.2.0_20230928/xtensa-esp-elf/bin/xtensa-esp32-elf-g++)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${FLAGS} ${IDF_INCLUDES}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-exceptions -fno-rtti ${FLAGS} ${IDF_INCLUDES}")

add_compile_definitions(ESP_PLATFORM LWIP_IPV4 LWIP_IPV6 PLATFORM_NAME_FREERTOS)

include_directories(
        "/home/jan/uros_ws/app_ws/src/micro_ros_espidf_component/examples/addtwoints_server/build/config"
        ${idf_path}/components/soc/${idf_target}/include
    )
