// /////////////////////////////////////////////////////////////////////////////
/// @file Fmt Helper
/// @brief Helper functionality printing with fmt library
// /////////////////////////////////////////////////////////////////////////////
#pragma once

#include <iostream>
#include <sstream>
#include <unordered_map>
#include <vector>
#include <type_traits>
#include <fmt/core.h>
#include <fmt/color.h>

namespace vdb_utils {

/// @brief Very minimalistic wrapper for fmt print library. Additional functionally includes
///   - print enabled colors only
class FmtHelper {
public:

  /// @brief default self ptr
  typedef std::shared_ptr<FmtHelper> Ptr;

  /// @brief default constructor
  /// @param[in] colors list of colors to enable for fmt print
  FmtHelper(std::vector<std::string> colors) {
    for (auto& colstr : colors) {
      // verify developer has not used an invalid fmt color
      // -- should be an assert instead of exception, but fine for now (since NDEBUG looks to be set).
      if (FmtHelper::color_.find(colstr) == FmtHelper::color_.end()) {
        std::ostringstream ss;
        ss << "Assert failed: Invalid color format given: " << colstr;
        throw std::runtime_error(ss.str().c_str());
      }
      // enable the given color
      enable_color_[color_[colstr]] = colstr;
    }
  };

  /// @brief default shared pointer contructor
  static Ptr MakeShared(std::vector<std::string> colors) { return Ptr(new FmtHelper(colors)); }

  /// @brief default destructor
  ~FmtHelper() {}

  /// @brief output string input as specialized format print (fmt library)
  /// @param[in] args... given string to output
  template <
    fmt::color color, // format color
    typename... Args  // variadic string to format print
  >
  inline void print(const Args&... args) const {
    if (enable_color_.find(color) != enable_color_.end()) {
      fmt::print(fg(color) , args...);
    }
  }

  /// @brief output string input as specialized format print (fmt library), passing additional format options
  /// @param[in] format addition format options
  /// @param[in] args... given string to output
  template <
    fmt::color color,     // format color
    typename fmt_options, // additional format options (color, emphasis, etc.)
    typename = typename std::enable_if< ! std::is_literal_type< fmt_options >::value, fmt_options>::type,
    typename... Args      // variadic string to format print
  >
  inline void print(fmt_options&& format, const Args&... args) const {
    if (enable_color_.find(color) != enable_color_.end()) {
      fmt::print(fg(color) | format, args...);
    }
  }

private:
  /// @brief fmt list of colors string name to fmt enum representation
  static std::unordered_map<std::string, fmt::color> color_;
  /// @brief reverse of 'color_' map, of fmt color to color string name representation
  std::unordered_map<fmt::color, std::string> enable_color_;
};

};  // vdb_utils
