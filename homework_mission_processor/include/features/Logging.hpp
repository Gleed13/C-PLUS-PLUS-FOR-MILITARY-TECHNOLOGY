#pragma once

#include <iostream>

#define ENABLE_LOG   1
#define ENABLE_DEBUG 1

#if ENABLE_LOG
  #define LOG(msg) std::cout << "[LOG] " << msg << std::endl
  #define WARNING(msg) std::cout << "[WARNING] " << msg << std::endl
  #define ERROR(msg) std::cerr << "[ERROR] " << msg << std::endl
#else
  #define LOG(msg)
  #define WARNING(msg)
  #define ERROR(msg)
#endif

#if ENABLE_DEBUG
  #define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
#else
  #define DEBUG(msg)
#endif
