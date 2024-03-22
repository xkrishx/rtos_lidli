# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/KPC/esp/v5.2.1/esp-idf/components/bootloader/subproject"
  "D:/KPC/PC_embedded/rtos/mqtt_sense/build/bootloader"
  "D:/KPC/PC_embedded/rtos/mqtt_sense/build/bootloader-prefix"
  "D:/KPC/PC_embedded/rtos/mqtt_sense/build/bootloader-prefix/tmp"
  "D:/KPC/PC_embedded/rtos/mqtt_sense/build/bootloader-prefix/src/bootloader-stamp"
  "D:/KPC/PC_embedded/rtos/mqtt_sense/build/bootloader-prefix/src"
  "D:/KPC/PC_embedded/rtos/mqtt_sense/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/KPC/PC_embedded/rtos/mqtt_sense/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/KPC/PC_embedded/rtos/mqtt_sense/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
