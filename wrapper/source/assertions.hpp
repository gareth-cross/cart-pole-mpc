// Copyright (c) 2024 Gareth Cross
#pragma once
#include <exception>
#include <string>
#include <string_view>

#include <fmt/core.h>

namespace assert {
namespace constants {
inline constexpr std::string_view message_prefix = "Assertion failed: {}\nFile: {}\nLine: {}";
}  // namespace constants

// Generates an exception with a formatted string.
template <typename... Ts>
[[nodiscard]] auto format_assert(const std::string_view condition, const std::string_view file,
                                 const int line, const std::string_view reason_fmt = {},
                                 Ts&&... args) {
  std::string err = fmt::format(constants::message_prefix, condition, file, line);
  if (!reason_fmt.empty()) {
    err.append("\nReason: ");
    fmt::format_to(std::back_inserter(err), reason_fmt, std::forward<Ts>(args)...);
  }
  return err;
}

// Version that prints args `a` & `b` as well, to support binary comparisons.
template <typename A, typename B, typename... Ts>
[[nodiscard]] auto format_assert_binary(const std::string_view condition,
                                        const std::string_view file, const int line,
                                        const std::string_view a_str, A&& a,
                                        const std::string_view b_str, B&& b,
                                        const std::string_view reason_fmt = {}, Ts&&... args) {
  std::string err = fmt::format(constants::message_prefix, condition, file, line);
  // TODO: Check if `a` and `b` are formattable here.
  fmt::format_to(std::back_inserter(err), "\nOperands are: `{}` = {}, `{}` = {}", a_str,
                 std::forward<A>(a), b_str, std::forward<B>(b));
  if (!reason_fmt.empty()) {
    err.append("\nReason: ");
    fmt::format_to(std::back_inserter(err), reason_fmt, std::forward<Ts>(args)...);
  }
  return err;
}

// The exception type that the library throws.
class default_error : public std::exception {
 public:
  explicit default_error(std::string message) noexcept : message_(std::move(message)) {}

  // Return the message string.
  const char* what() const noexcept override { return message_.c_str(); }

 private:
  std::string message_;
};

}  // namespace assert

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wgnu-zero-variadic-macro-arguments"
#endif  // __clang__

// Assertion macros.
#define _F_ASSERT_IMPL(cond, file, line, handler, ...)                        \
  do {                                                                        \
    if (!static_cast<bool>(cond)) {                                           \
      throw assert::default_error(handler(#cond, file, line, ##__VA_ARGS__)); \
    }                                                                         \
  } while (false)

#define _F_ASSERT_ALWAYS_IMPL(file, line, handler, ...)                               \
  do {                                                                                \
    throw assert::default_error(handler("Assert always", file, line, ##__VA_ARGS__)); \
  } while (false)

// Macro to use when defining an assertion.
#define F_ASSERT(cond, ...) \
  _F_ASSERT_IMPL(cond, __FILE__, __LINE__, assert::format_assert, ##__VA_ARGS__)

#define F_ASSERT_ALWAYS(...) \
  _F_ASSERT_ALWAYS_IMPL(__FILE__, __LINE__, assert::format_assert, ##__VA_ARGS__)

#define F_ASSERT_EQ(a, b, ...)                                                               \
  _F_ASSERT_IMPL((a) == (b), __FILE__, __LINE__, assert::format_assert_binary, #a, a, #b, b, \
                 ##__VA_ARGS__)

#define F_ASSERT_NE(a, b, ...)                                                               \
  _F_ASSERT_IMPL((a) != (b), __FILE__, __LINE__, assert::format_assert_binary, #a, a, #b, b, \
                 ##__VA_ARGS__)

#define F_ASSERT_LT(a, b, ...)                                                              \
  _F_ASSERT_IMPL((a) < (b), __FILE__, __LINE__, assert::format_assert_binary, #a, a, #b, b, \
                 ##__VA_ARGS__)

#define F_ASSERT_GT(a, b, ...)                                                              \
  _F_ASSERT_IMPL((a) > (b), __FILE__, __LINE__, assert::format_assert_binary, #a, a, #b, b, \
                 ##__VA_ARGS__)

#define F_ASSERT_LE(a, b, ...)                                                               \
  _F_ASSERT_IMPL((a) <= (b), __FILE__, __LINE__, assert::format_assert_binary, #a, a, #b, b, \
                 ##__VA_ARGS__)

#define F_ASSERT_GE(a, b, ...)                                                               \
  _F_ASSERT_IMPL((a) >= (b), __FILE__, __LINE__, assert::format_assert_binary, #a, a, #b, b, \
                 ##__VA_ARGS__)

#ifdef __clang__
#pragma clang diagnostic pop
#endif  // __clang__
