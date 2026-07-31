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

#ifndef SOPHUS__SIM_DETAILS_HPP_
#define SOPHUS__SIM_DETAILS_HPP_

#include "types.hpp"

namespace Sophus
{
namespace details
{

template <class Scalar, int N>
Matrix<Scalar, N, N> calcW(
  Matrix<Scalar, N, N> const & Omega, Scalar const theta, Scalar const sigma)
{
  using std::abs;
  using std::cos;
  using std::exp;
  using std::sin;
  static Matrix<Scalar, N, N> const I = Matrix<Scalar, N, N>::Identity();
  static Scalar const one(1);
  static Scalar const half(0.5);
  Matrix<Scalar, N, N> const Omega2 = Omega * Omega;
  Scalar const scale = exp(sigma);
  Scalar A, B, C;
  if (abs(sigma) < Constants<Scalar>::epsilon()) {
    C = one;
    if (abs(theta) < Constants<Scalar>::epsilon()) {
      A = half;
      B = Scalar(1. / 6.);
    } else {
      Scalar theta_sq = theta * theta;
      A = (one - cos(theta)) / theta_sq;
      B = (theta - sin(theta)) / (theta_sq * theta);
    }
  } else {
    C = (scale - one) / sigma;
    if (abs(theta) < Constants<Scalar>::epsilon()) {
      Scalar sigma_sq = sigma * sigma;
      A = ((sigma - one) * scale + one) / sigma_sq;
      B = (scale * half * sigma_sq + scale - one - sigma * scale) / (sigma_sq * sigma);
    } else {
      Scalar theta_sq = theta * theta;
      Scalar a = scale * sin(theta);
      Scalar b = scale * cos(theta);
      Scalar c = theta_sq + sigma * sigma;
      A = (a * sigma + (one - b) * theta) / (theta * c);
      B = (C - ((b - one) * sigma + a * theta) / (c)) * one / (theta_sq);
    }
  }
  return A * Omega + B * Omega2 + C * I;
}

template <class Scalar, int N>
Matrix<Scalar, N, N> calcWInv(
  Matrix<Scalar, N, N> const & Omega, Scalar const theta, Scalar const sigma, Scalar const scale)
{
  using std::abs;
  using std::cos;
  using std::sin;
  static Matrix<Scalar, N, N> const I = Matrix<Scalar, N, N>::Identity();
  static Scalar const half(0.5);
  static Scalar const one(1);
  static Scalar const two(2);
  Matrix<Scalar, N, N> const Omega2 = Omega * Omega;
  Scalar const scale_sq = scale * scale;
  Scalar const theta_sq = theta * theta;
  Scalar const sin_theta = sin(theta);
  Scalar const cos_theta = cos(theta);

  Scalar a, b, c;
  if (abs(sigma * sigma) < Constants<Scalar>::epsilon()) {
    c = one - half * sigma;
    a = -half;
    if (abs(theta_sq) < Constants<Scalar>::epsilon()) {
      b = Scalar(1. / 12.);
    } else {
      b = (theta * sin_theta + two * cos_theta - two) / (two * theta_sq * (cos_theta - one));
    }
  } else {
    Scalar const scale_cu = scale_sq * scale;
    c = sigma / (scale - one);
    if (abs(theta_sq) < Constants<Scalar>::epsilon()) {
      a = (-sigma * scale + scale - one) / ((scale - one) * (scale - one));
      b = (scale_sq * sigma - two * scale_sq + scale * sigma + two * scale) /
          (two * scale_cu - Scalar(6) * scale_sq + Scalar(6) * scale - two);
    } else {
      Scalar const s_sin_theta = scale * sin_theta;
      Scalar const s_cos_theta = scale * cos_theta;
      a = (theta * s_cos_theta - theta - sigma * s_sin_theta) /
          (theta * (scale_sq - two * s_cos_theta + one));
      b = -scale *
          (theta * s_sin_theta - theta * sin_theta + sigma * s_cos_theta - scale * sigma +
           sigma * cos_theta - sigma) /
          (theta_sq *
           (scale_cu - two * scale * s_cos_theta - scale_sq + two * s_cos_theta + scale - one));
    }
  }
  return a * Omega + b * Omega2 + c * I;
}

}  // namespace details
}  // namespace Sophus

#endif  // SOPHUS__SIM_DETAILS_HPP_
