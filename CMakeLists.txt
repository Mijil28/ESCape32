cmake_minimum_required(VERSION 3.15)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_C_FLAGS "-Os -g")
set(CMAKE_C_FLAGS_DEBUG "-Og")
project(ESCape32 C)
if(UNIX AND NOT CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT) # Use 'CMAKE_INSTALL_PREFIX' as alternative system root
	set(CMAKE_PREFIX_PATH ${CMAKE_INSTALL_PREFIX})
	include(GNUInstallDirs)
	include_directories(${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR})
	link_directories(${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR})
endif()
if(LIBOPENCM3_DIR)
	include_directories(${LIBOPENCM3_DIR}/include)
	link_directories(${LIBOPENCM3_DIR}/lib)
endif()
add_compile_options(-ffreestanding -ffunction-sections -fdata-sections -fsingle-precision-constant
	-Wall -Wextra -Wpedantic -Wundef -Wshadow -Wredundant-decls -Wstrict-prototypes -Wmissing-prototypes
	-Wno-variadic-macros -Wno-unused-result -Wno-unused-parameter -Wno-unused-label)
add_link_options(-nostartfiles -Wl,--gc-sections)

function(add_target name mcu)
	include(mcu/${mcu}/config.cmake)
	file(READ src/main.c main_c)
	string(REGEX MATCH "REVISION [0-9]+" rev1 ${main_c})
	string(REGEX MATCH "REVPATCH [0-9]+" rev2 ${main_c})
	string(REGEX REPLACE "[^0-9]+" "-rev" rev1 ${rev1})
	if (rev2)
		string(REGEX REPLACE "[^0-9]+" "." rev2 ${rev2})
	endif()
	set(elf ${name}${rev1}${rev2}.elf)
	set(bin ${name}${rev1}${rev2}.bin)
	set(hex ${name}${rev1}${rev2}.hex)
	file(GLOB srcs src/*.c mcu/${mcu}/*.c)
	add_executable(${elf} ${srcs})
	target_include_directories(${elf} PRIVATE src mcu/${mcu})
	target_compile_options(${elf} PRIVATE ${opts})
	target_compile_definitions(${elf} PRIVATE ${mcu} ${name} ${defs} TARGET_NAME="${name}" ${ARGN})
	target_link_options(${elf} PRIVATE -T${CMAKE_CURRENT_SOURCE_DIR}/mcu/${mcu}/config.ld -T${CMAKE_CURRENT_SOURCE_DIR}/mcu/common.ld ${opts})
	target_link_libraries(${elf} c_nano gcc nosys ${libs})
	add_custom_command(
		OUTPUT ${bin} ${hex}
		COMMAND arm-none-eabi-objcopy -O binary ${elf} ${bin}
		COMMAND arm-none-eabi-objcopy -O ihex ${elf} ${hex}
		DEPENDS ${elf}
	)
	add_custom_target(${name} ALL DEPENDS ${hex})
	add_custom_target(flash-${name} COMMAND st-flash --reset --connect-under-reset --format ihex write ${hex} DEPENDS ${hex})
endfunction()

add_subdirectory(boot)

add_target(HOBBYWING1 STM32F051 DEAD_TIME=40 COMP_MAP=123 TIMING=24 FREQ_MIN=36 FREQ_MAX=72 DUTY_RAMP=50 DUTY_RATE=100 IO_PA2)
add_target(HOBBYWING2 AT32F421 DEAD_TIME=96 COMP_MAP=123 TIMING=24 FREQ_MIN=36 FREQ_MAX=72 DUTY_RAMP=50 DUTY_RATE=100)
add_target(HOBBYWING3 STM32F302 DEAD_TIME=57 COMP_MAP=123 SENS_MAP=0xA3 VOLT_MUL=1290 USE_COMP1 TIMING=24 FREQ_MIN=36 FREQ_MAX=72 DUTY_RAMP=50 DUTY_RATE=100 IO_PB9 USE_HSE=8)

