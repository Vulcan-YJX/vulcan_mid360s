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

#ifndef SOPHUS__INTERPOLATE_HPP_
#define SOPHUS__INTERPOLATE_HPP_

#include <Eigen/Eigenvalues>

#include "interpolate_details.hpp"

namespace Sophus
{

/// This function interpolates between two Lie group elements ``foo_T_bar``
/// and ``foo_T_baz`` with an interpolation factor of ``alpha`` in [0, 1].
///
/// It returns a pose ``foo_T_quiz`` with ``quiz`` being a frame between ``bar``
/// and ``baz``. If ``alpha=0`` it returns ``foo_T_bar``. If it is 1, it returns
/// ``foo_T_bar``.
///
/// (Since interpolation on Lie groups is inverse-invariant, we can equivalently
/// think of the input arguments as being ``bar_T_foo``, ``baz_T_foo`` and the
/// return value being ``quiz_T_foo``.)
///
/// Precondition: ``p`` must be in [0, 1].
///
template <class G, class Scalar2 = typename G::Scalar>
enable_if_t<interp_details::Traits<G>::supported, G> interpolate(
  G const & foo_T_bar, G const & foo_T_baz, Scalar2 p = Scalar2(0.5f))
{
  using Scalar = typename G::Scalar;
  Scalar inter_p(p);
  SOPHUS_ENSURE(inter_p >= Scalar(0) && inter_p <= Scalar(1), "p (%) must in [0, 1].");
  return foo_T_bar * G::exp(inter_p * (foo_T_bar.inverse() * foo_T_baz).log());
}

}  // namespace Sophus

#endif  // SOPHUS__INTERPOLATE_HPP_
