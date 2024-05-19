#此文件从模板自动生成! 请勿更改!
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.28)

# specify cross-compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# project settings
project(BananaBot C CXX ASM)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_STANDARD 11)

#Uncomment for hardware floating point
add_compile_definitions(ARM_MATH_CM7;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
add_compile_options(-mfloat-abi=hard -mfpu=fpv5-d16)
add_link_options(-mfloat-abi=hard -mfpu=fpv5-d16)

#Uncomment for software floating point
#add_compile_options(-mfloat-abi=soft)

add_compile_options(-mcpu=cortex-m7 -mthumb -mthumb-interwork)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)

# uncomment to mitigate c++17 absolute addresses warnings
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-register")

# Enable assembler files preprocessing
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g)
endif ()

include_directories(Core/Inc Drivers/STM32H7xx_HAL_Driver/Inc Drivers/STM32H7xx_HAL_Driver/Inc/Legacy Drivers/CMSIS/Device/ST/STM32H7xx/Include Drivers/CMSIS/Include AZURE_RTOS/App Middlewares/ST/usbx/common/core/inc/ Middlewares/ST/usbx/ports/generic/inc/ Middlewares/ST/usbx/common/usbx_device_classes/inc/ Middlewares/ST/threadx/common/inc/ Middlewares/ST/threadx/ports/cortex_m7/gnu/inc/)

# Include arm_math head files
include_directories(DSP/Include DSP/Include/dsp DSP/PrivateInclude)
# Include BSP head files
include_directories(Module/BSP/Inc)
# Include PSAI head files
include_directories(Module/PSAI/Inc)
# Include mavlink head files
include_directories(Module/Mavlink/FishCom)
# Include IMU head files
include_directories(Module/IMU/Inc)
# Include LED head files
include_directories(Module/LED/Inc)
# include user service head files
include_directories(Service/Inc)
# include user task head files
include_directories(Task/Inc)

add_definitions(-DDEBUG -DUSE_HAL_DRIVER -DSTM32H723xx -DTX_INCLUDE_USER_DEFINE_FILE -DUSE_FULL_LL_DRIVER)

file(GLOB_RECURSE SOURCES "AZURE_RTOS/*.*" "ThreadX/*.*" "Core/*.*" "Middlewares/*.*" "Drivers/*.*")

# Add user source files

file(GLOB_RECURSE Module "Module/*.*")
file(GLOB_RECURSE SERVICES "Service/*.*")
file(GLOB_RECURSE TASKS "Task/*.*")

LIST(APPEND SOURCES ${Module})
LIST(APPEND SOURCES ${SERVICES})
LIST(APPEND SOURCES ${TASKS})

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32H723VETX_FLASH.ld)

add_link_options(-specs=nosys.specs)
add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-mcpu=cortex-m7 -mthumb -mthumb-interwork)
add_link_options(-T ${LINKER_SCRIPT})

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

# Add LVGL
# set(LV_CONF_BUILD_DISABLE_EXAMPLES ON)
# set(LV_CONF_BUILD_DISABLE_DEMOS ON)

# add_subdirectory(LVGL lvgl)
# target_include_directories(${PROJECT_NAME}.elf PUBLIC $<TARGET_PROPERTY:lvgl,INCLUDE_DIRECTORIES>)
# target_link_libraries(${PROJECT_NAME}.elf PUBLIC lvgl)

# Add OneMessage
add_subdirectory(Module/OneMessage om)
target_include_directories(${PROJECT_NAME}.elf PUBLIC $<TARGET_PROPERTY:OneMessage,INCLUDE_DIRECTORIES>)
target_link_libraries(${PROJECT_NAME}.elf PUBLIC OneMessage)

# Link arm_math
#add_subdirectory(DSP/Source dsp)
#target_link_libraries(${PROJECT_NAME}.elf PUBLIC CMSISDSP)
target_link_libraries(${PROJECT_NAME}.elf PUBLIC ${PROJECT_SOURCE_DIR}/DSP/libCMSISDSP.a)

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")
