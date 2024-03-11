#pragma once

#include <assert.h>
#define ROS_WARN(...) printf(__VA_ARGS__)
#define ROS_INFO(...) printf(__VA_ARGS__)
#define ROS_DEBUG(...)
#define ROS_INFO_STREAM(args) std::cout << args << "\n"
#define ROS_WARN_STREAM(args) std::cout << args << "\n"
#define ROS_DEBUG_STREAM(args) std::cout << args << "\n" 
#define ROS_ASSERT(cond) \
do { \
    if (!(cond)) { \
        printf("ASSERTION FAILED\n\tfile = %s\n\tline = %d\n\tcond = %s\n", __FILE__, __LINE__, #cond); \
        assert(false); \
    } \
} while (0)
#define ROS_BREAK() \
do { \
     printf("BREAKPOINT HIT\n\tfile = %s\n\tline=%d\n", __FILE__, __LINE__); \
     assert(false); \
} while (0)
