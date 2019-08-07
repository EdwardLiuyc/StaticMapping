// MIT License

// Copyright (c) 2019 Edward Liu

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/*!
 * \file macro_defines.h
 * \todo add the functions to namespace "common"
 */
#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>

#include "common/eigen_pitfalls_avoidence.h"

#define NONE_FORMAT "\e[0m"
#define BLACK "\e[1;30m"
#define L_BLACK "\e[0;30m"
#define RED "\e[1;31m"
#define L_RED "\e[0;31m"
#define GREEN "\e[1;32m"
#define L_GREEN "\e[0;32m"
#define BROWN "\e[1;33m"
#define YELLOW "\e[1;33m"
#define BLUE "\e[1;34m"
#define L_BLUE "\e[0;34m"
#define PURPLE "\e[1;35m"
#define L_PURPLE "\e[0;35m"
#define CYAN "\e[1;36m"
#define L_CYAN "\e[0;36m"
#define GRAY "\e[1;37m"
#define WHITE "\e[1;37m"

#define BOLD "\e[1m"
#define UNDERLINE "\e[4m"
#define BLINK "\e[5m"
#define REVERSE "\e[7m"
#define HIDE "\e[8m"
#define CLEAR "\e[2J"
#define CLRLINE "\r\e[K"  // or "\e[1K\r"

namespace static_map {
/*!
 * \brief just for debug
 *
 * use this function with end_clock/end_clock_min_time
 * the when you call end_clock, you will get the time cost from the start to
 * the end
 */
void start_clock();
void end_clock(const char* filename, const char* func_name, const int line);

void start_clock_std();
void end_clock_std();
void end_clock_min_time_std(double min_time);

/*!
 * @brief return the filename without path
 */
char* splited_file_name(const char*);

}  // namespace static_map

using static_map::splited_file_name;

/*!
 * @def PRINT_ERROR(args)
 * print error message
 *
 * @def PRINT_ERROR_FMT(fmt, ...)
 * print error in a specific format ( like printf )
 *
 * @def PRINT_DEBUG(args)
 * print debug message
 *
 * @def PRINT_DEBUG_FMT(fmt, ...)
 * print debug in a specific format ( like printf )
 *
 * @def PRINT_INFO(args)
 * print infomation
 *
 * @def PRINT_INFO_FMT(fmt, ...)
 * print infomation in a specific format ( like printf )
 *
 * @def PRINT_WARNING(args)
 * print warning
 *
 * @def PRINT_WARNING_FMT(fmt, ...)
 * print warning in a specific format ( like printf )
 *
 * @def PRINT_COLOR(color, args)
 * print a message in specific color
 *
 * @def PRINT_COLOR_FMT(color, fmt, ...)
 * print a message in a specific format and color ( like printf )
 */
#define PRINT_ERROR(args)                                                     \
  (std::cout << "[ " << BOLD << splited_file_name(__FILE__) << NONE_FORMAT    \
             << ": " << __LINE__ << " ]" << RED << " [ERROR] " << NONE_FORMAT \
             << (args) << std::endl)
#define PRINT_ERROR_FMT(fmt, ...)                            \
  (printf("[ %s: %d ]" RED " [ERROR] " NONE_FORMAT fmt "\n", \
          splited_file_name(__FILE__), __LINE__, __VA_ARGS__))

// DEBUG
#define PRINT_DEBUG(args)                                               \
  (std::cout << "[ " << splited_file_name(__FILE__) << ": " << __LINE__ \
             << " ]" << BLUE << " [DEBUG] " << NONE_FORMAT << (args)    \
             << std::endl)
#define PRINT_DEBUG_FMT(fmt, ...)                             \
  (printf("[ %s: %d ]" BLUE " [DEBUG] " NONE_FORMAT fmt "\n", \
          splited_file_name(__FILE__), __LINE__, __VA_ARGS__))

// INFO
#define PRINT_INFO(args)                                                \
  (std::cout << "[ " << splited_file_name(__FILE__) << ": " << __LINE__ \
             << " ]" << GREEN << " [INFO] " << NONE_FORMAT << (args)    \
             << std::endl)
#define PRINT_INFO_FMT(fmt, ...)                              \
  (printf("[ %s: %d ]" GREEN " [INFO] " NONE_FORMAT fmt "\n", \
          splited_file_name(__FILE__), __LINE__, __VA_ARGS__))

// WARN
#define PRINT_WARNING(args)                                              \
  (std::cout << "[ " << splited_file_name(__FILE__) << ": " << __LINE__  \
             << " ]" << YELLOW << " [WARNING] " << NONE_FORMAT << (args) \
             << std::endl)
#define PRINT_WARNING_FMT(fmt, ...)                               \
  (printf("[ %s: %d ]" YELLOW " [WARNING] " NONE_FORMAT fmt "\n", \
          splited_file_name(__FILE__), __LINE__, __VA_ARGS__))

#define PRINT_COLOR(color, args) \
  (std::cout << color << (args) << NONE_FORMAT << std::endl)
#define PRINT_COLOR_FMT(color, fmt, ...) \
  (printf(color fmt NONE_FORMAT "\n", __VA_ARGS__))

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

/*!
 * \def DOUBLE_M_PI
 * M_PI * 2
 */
#ifndef DOUBLE_M_PI
#define DOUBLE_M_PI 6.28318530718
#endif

/*!
 * \def SIMPLE_MEM_BARRIER
 * memory barrier in asm way, it makes sure that
 * the value of variable written to memory instead of register
 */
#ifndef SIMPLE_MEM_BARRIER
#define SIMPLE_MEM_BARRIER \
  { asm volatile("" : : : "memory"); }
#endif

/*!
 * \def DO_PRAGMA(x)
 * just print a message when compiling
 *
 * \def TODO(x)
 * print a TODO message when compiling
 */
#define DO_PRAGMA(x) _Pragma(#x)
#define TODO(x) DO_PRAGMA(message("TODO - " #x))

/*!
 * \def DOUBLE_EQUAL(a, b)
 * Determine if two double numbers a and b are equal
 */
#define DOUBLE_EQUAL(a, b) (fabs((a) - (b)) < 1.e-6)

#define RAD_TO_DEG 57.29577951

#define LOCAL_OMP_THREADS_NUM 10

#define GET_SINGLE_OPTION(XML_NODE, CHILD_NAME, CHILD_ATTR_NAME, TARGET,     \
                          DATA_TYPE, TARGET_TYPE)                            \
  {                                                                          \
    if (!XML_NODE.child(CHILD_NAME).empty() &&                               \
        XML_NODE.child(CHILD_NAME).attribute(CHILD_ATTR_NAME)) {             \
      auto tmp_node = XML_NODE.child(CHILD_NAME);                            \
      TARGET =                                                               \
          (TARGET_TYPE)tmp_node.attribute(CHILD_ATTR_NAME).as_##DATA_TYPE(); \
      std::cout << std::boolalpha << "  " << std::setw(50)                   \
                << std::string(#TARGET) << " -> " << (DATA_TYPE)TARGET       \
                << std::endl;                                                \
    } else {                                                                 \
      std::cout << std::boolalpha << "  " << std::setw(50)                   \
                << std::string(#TARGET) << " -> " << L_GREEN << "[default] " \
                << NONE_FORMAT << (DATA_TYPE)TARGET << std::endl;            \
    }                                                                        \
  }
