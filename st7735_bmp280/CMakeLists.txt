# Set minimum required version of CMake
cmake_minimum_required(VERSION 3.12)

# Include build functions from Pico SDK
include(../pico_sdk_import.cmake)

# Set name of project (as PROJECT_NAME) and C/C++ standards
project(st7735_bmp280 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Creates a pico-sdk subdirectory in our project for the libraries
pico_sdk_init()

# Tell CMake where to find the executable source file
add_executable(${PROJECT_NAME} st7735_bmp280.cpp)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME})

add_library(pico_st7735_3 INTERFACE)

target_sources(pico_st7735_3 INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/st7735/ST7735_TFT.cpp
  ${CMAKE_CURRENT_LIST_DIR}/st7735/ST7735_TFT_graphics.cpp
  ${CMAKE_CURRENT_LIST_DIR}/st7735/ST7735_TFT_Print.cpp
  ${CMAKE_CURRENT_LIST_DIR}/st7735/ST7735_TFT_Font.cpp
  ${CMAKE_CURRENT_LIST_DIR}/st7735/ST7735_TFT_Bitmap_Data.cpp
)

target_include_directories(pico_st7735 INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)

# Pull in pico libraries that we need
target_link_libraries(${PROJECT_NAME} pico_stdlib hardware_spi hardware_i2c pico_st7735_3)

# Enable usb output, disable uart output
pico_enable_stdio_usb(my_blink 1)
pico_enable_stdio_uart(my_blink 0)
