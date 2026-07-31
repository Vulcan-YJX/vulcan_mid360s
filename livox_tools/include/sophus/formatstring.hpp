// This file is part of Sophus.
//
// Copyright 2013 Hauke Strasdat
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights  to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.

#ifndef SOPHUS__FORMATSTRING_HPP_
#define SOPHUS__FORMATSTRING_HPP_

#include <iostream>
#include <string>
#include <utility>

namespace Sophus
{
namespace details
{

// Following: http://stackoverflow.com/a/22759544
template <class T>
class IsStreamable
{
private:
  template <class TT>
  static auto test(int)  // NOLINT(readability/casting)
    -> decltype(std::declval<std::stringstream &>() << std::declval<TT>(), std::true_type());
  template <class>
  static auto test(...) -> std::false_type;

public:
  static bool const value = decltype(test<T>(0))::value;
};

template <class T>
class ArgToStream
{
public:
  static void impl(std::stringstream & stream, T && arg) { stream << std::forward<T>(arg); }
};

inline void FormatStream(std::stringstream & stream, char const * text)
{
  stream << text;
  return;
}

// Following: http://en.cppreference.com/w/cpp/language/parameter_pack
template <class T, typename... Args>
void FormatStream(std::stringstream & stream, char const * text, T && arg, Args &&... args)
{
  static_assert(IsStreamable<T>::value, "One of the args has no ostream overload!");
  for (; *text != '\0'; ++text) {
    if (*text == '%') {
      ArgToStream<T &&>::impl(stream, std::forward<T>(arg));
      FormatStream(stream, text + 1, std::forward<Args>(args)...);
      return;
    }
    stream << *text;
  }
  stream << "\nFormat-Warning: There are " << sizeof...(Args) + 1 << " args unused.";
  return;
}

template <class... Args>
std::string FormatString(char const * text, Args &&... args)
{
  std::stringstream stream;
  FormatStream(stream, text, std::forward<Args>(args)...);
  return stream.str();
}

inline std::string FormatString() { return std::string(); }

}  // namespace details
}  // namespace Sophus

#endif  // SOPHUS__FORMATSTRING_HPP_
