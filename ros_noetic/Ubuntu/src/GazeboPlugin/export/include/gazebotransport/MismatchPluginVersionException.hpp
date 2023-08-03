/* Copyright 2021 The MathWorks, Inc. */
#ifndef MISMATCH_PLUGIN_VERSION_EXP_HPP
#define MISMATCH_PLUGIN_VERSION_EXP_HPP

#include <exception>

namespace robotics {
namespace gazebotransport {
// Exception class to handle mismatch plugin version
class MismatchPluginVersionException : public std::exception {
  public:
    const char* what() const throw() {
        return "Plugin Version Mismatch!";
    }
};

} // namespace gazebotransport
} // namespace robotics
#endif
