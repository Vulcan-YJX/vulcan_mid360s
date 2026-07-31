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

#ifndef SOPHUS__ROTATION_MATRIX_HPP_
#define SOPHUS__ROTATION_MATRIX_HPP_

#include <Eigen/Dense>
#include <Eigen/SVD>

#include "types.hpp"

namespace Sophus
{

/// Takes in arbitrary square matrix and returns true if it is
/// orthogonal.
template <class D>
SOPHUS_FUNC bool isOrthogonal(Eigen::MatrixBase<D> const & R)
{
  using Scalar = typename D::Scalar;
  static int const N = D::RowsAtCompileTime;
  static int const M = D::ColsAtCompileTime;

  static_assert(N == M, "must be a square matrix");
  static_assert(N >= 2, "must have compile time dimension >= 2");

  return (R * R.transpose() - Matrix<Scalar, N, N>::Identity()).norm() <
         Constants<Scalar>::epsilon();
}

/// Takes in arbitrary square matrix and returns true if it is
/// "scaled-orthogonal" with positive determinant.
///
template <class D>
SOPHUS_FUNC bool isScaledOrthogonalAndPositive(Eigen::MatrixBase<D> const & sR)
{
  using Scalar = typename D::Scalar;
  static int const N = D::RowsAtCompileTime;
  static int const M = D::ColsAtCompileTime;
  using std::pow;
  using std::sqrt;

  Scalar det = sR.determinant();

  if (det <= Scalar(0)) {
    return false;
  }

  Scalar scale_sqr = pow(det, Scalar(2. / N));

  static_assert(N == M, "must be a square matrix");
  static_assert(N >= 2, "must have compile time dimension >= 2");

  return (sR * sR.transpose() - scale_sqr * Matrix<Scalar, N, N>::Identity())
           .template lpNorm<Eigen::Infinity>() < sqrt(Constants<Scalar>::epsilon());
}

/// Takes in arbitrary square matrix (2x2 or larger) and returns closest
/// orthogonal matrix with positive determinant.
template <class D>
SOPHUS_FUNC enable_if_t<
  std::is_floating_point<typename D::Scalar>::value,
  Matrix<typename D::Scalar, D::RowsAtCompileTime, D::RowsAtCompileTime>>
makeRotationMatrix(Eigen::MatrixBase<D> const & R)
{
  using Scalar = typename D::Scalar;
  static int const N = D::RowsAtCompileTime;
  static int const M = D::ColsAtCompileTime;

  static_assert(N == M, "must be a square matrix");
  static_assert(N >= 2, "must have compile time dimension >= 2");

  Eigen::JacobiSVD<Matrix<Scalar, N, N>> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);

  // Determine determinant of orthogonal matrix U*V'.
  Scalar d = (svd.matrixU() * svd.matrixV().transpose()).determinant();
  // Starting from the identity matrix D, set the last entry to d (+1 or
  // -1),  so that det(U*D*V') = 1.
  Matrix<Scalar, N, N> Diag = Matrix<Scalar, N, N>::Identity();
  Diag(N - 1, N - 1) = d;
  return svd.matrixU() * Diag * svd.matrixV().transpose();
}

}  // namespace Sophus

#endif  // SOPHUS__ROTATION_MATRIX_HPP_
