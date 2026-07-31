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

#ifndef SOPHUS__INTERPOLATE_DETAILS_HPP_
#define SOPHUS__INTERPOLATE_DETAILS_HPP_

#include "rxso2.hpp"
#include "rxso3.hpp"
#include "se2.hpp"
#include "se3.hpp"
#include "sim2.hpp"
#include "sim3.hpp"
#include "so2.hpp"
#include "so3.hpp"

namespace Sophus
{
namespace interp_details
{

template <class Group>
struct Traits;

template <class Scalar>
struct Traits<SO2<Scalar>>
{
  static bool constexpr supported = true;

  static bool hasShortestPathAmbiguity(SO2<Scalar> const & foo_T_bar)
  {
    using std::abs;
    Scalar angle = foo_T_bar.log();
    return abs(abs(angle) - Constants<Scalar>::pi()) < Constants<Scalar>::epsilon();
  }
};

template <class Scalar>
struct Traits<RxSO2<Scalar>>
{
  static bool constexpr supported = true;

  static bool hasShortestPathAmbiguity(RxSO2<Scalar> const & foo_T_bar)
  {
    return Traits<SO2<Scalar>>::hasShortestPathAmbiguity(foo_T_bar.so2());
  }
};

template <class Scalar>
struct Traits<SO3<Scalar>>
{
  static bool constexpr supported = true;

  static bool hasShortestPathAmbiguity(SO3<Scalar> const & foo_T_bar)
  {
    using std::abs;
    Scalar angle = foo_T_bar.logAndTheta().theta;
    return abs(abs(angle) - Constants<Scalar>::pi()) < Constants<Scalar>::epsilon();
  }
};

template <class Scalar>
struct Traits<RxSO3<Scalar>>
{
  static bool constexpr supported = true;

  static bool hasShortestPathAmbiguity(RxSO3<Scalar> const & foo_T_bar)
  {
    return Traits<SO3<Scalar>>::hasShortestPathAmbiguity(foo_T_bar.so3());
  }
};

template <class Scalar>
struct Traits<SE2<Scalar>>
{
  static bool constexpr supported = true;

  static bool hasShortestPathAmbiguity(SE2<Scalar> const & foo_T_bar)
  {
    return Traits<SO2<Scalar>>::hasShortestPathAmbiguity(foo_T_bar.so2());
  }
};

template <class Scalar>
struct Traits<SE3<Scalar>>
{
  static bool constexpr supported = true;

  static bool hasShortestPathAmbiguity(SE3<Scalar> const & foo_T_bar)
  {
    return Traits<SO3<Scalar>>::hasShortestPathAmbiguity(foo_T_bar.so3());
  }
};

template <class Scalar>
struct Traits<Sim2<Scalar>>
{
  static bool constexpr supported = true;

  static bool hasShortestPathAmbiguity(Sim2<Scalar> const & foo_T_bar)
  {
    return Traits<SO2<Scalar>>::hasShortestPathAmbiguity(foo_T_bar.rxso2().so2());
    ;
  }
};

template <class Scalar>
struct Traits<Sim3<Scalar>>
{
  static bool constexpr supported = true;

  static bool hasShortestPathAmbiguity(Sim3<Scalar> const & foo_T_bar)
  {
    return Traits<SO3<Scalar>>::hasShortestPathAmbiguity(foo_T_bar.rxso3().so3());
    ;
  }
};

}  // namespace interp_details
}  // namespace Sophus

#endif  // SOPHUS__INTERPOLATE_DETAILS_HPP_
