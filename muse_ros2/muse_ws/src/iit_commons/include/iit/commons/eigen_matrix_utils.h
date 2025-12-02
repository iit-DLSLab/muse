#ifndef _IIT_COMMONS_EIGEN_MATRIX_UTILS_H_
#define _IIT_COMMONS_EIGEN_MATRIX_UTILS_H_

#include <Eigen/Dense>

/*! \mainpage IIT Commons Library
 *
 * \section intro_sec Introduction
 *
 * This library contains many useful tools for algebra, rotations, kinematics
 * and dynamics of a quadruped robot...
 *
 * \section install_sec Installation
 * this is a CMake project, therefore you can build and install this package as
 * usual:
 * \code{.sh}
 * mkdir build
 * cd build
 * cmake ..
 * make
 * sudo make install
 * \endcode
 * this will build and install the library in <code>/usr/local</code>
 */

namespace iit {
namespace commons {

/**
 * Coefficient-wise, unary operator that evaluates to zero for each element
 * of the matrix expression whose absolute value is below a threshold.
 * Useful for printing matrices, approximating with 0 small coefficients.
 *
 * \tparam Scalar the numerical data type of the scalars (i.e. the elements of
 *         the matrix)
 *
 * Example usage:
 * \code
 *   iit::commons::CwiseAlmostZeroOp<double> approx(1E-5);
 *   // ... definition of matrix M
 *   // ...
 *   std::cout << M.unaryExpr(approx) << std::endl;
 * \endcode
 */
template <typename Scalar>
struct CwiseAlmostZeroOp {
    CwiseAlmostZeroOp(const Scalar& threshold) : thresh(threshold) {}
    const Scalar operator()(const Scalar& x) const {
        return std::abs(x) < thresh ? 0 : x;
    }
    private:
        Scalar thresh;
};


/**
 * Custom binary functor that evaluates the difference of two scalars and rounds
 * it to zero if it is negligible.
 *
 * \tparam Scalar the numerical data type of the elements of the matrices
 *
 * Example:
 * \code
 *    RoundTheDifference<double> rounder(3);
 *    // ... definition of matrices A, B, R
 *    R = A.binaryExpr(B, rounder);  // returns A-B approximated
 * \endcode
 *
 * \see roundedDifference
 */
template<typename Scalar>
struct RoundTheDifference
{
    /**
     * \param perc the percentage used to compute the threshold: the difference
     *   is rounded to zero if it is smaller than this percentage of the first
     *   scalar
     * \param abs_thresh an absolute threshold below which the difference is
     *   rounded to zero, regardless the ratio with the operands; defaults to
     *   1E-6
     */
    RoundTheDifference(unsigned short perc, double abs_thresh = 1E-6) :
        percentage(perc), thresh(abs_thresh) {}

    Scalar operator()(const Scalar& a, const Scalar& b) const
    {
        Scalar diff = a-b;
        //   If 'a' is identically zero, even with a very small 'diff' (like
        // 1E-6), the percentage check will fail; therefore check explicitly
        // whether 'diff' is negligible in "absolute" terms
        bool negligible = (std::abs(diff)<thresh) || (std::abs(diff) < std::abs(a)/100*percentage);
        return negligible ? 0 : diff;
    }
private:
    unsigned short percentage;
    double thresh;
};

/**
 * Computes the difference between matrix A and B, rounding to zero the small
 * elements. It uses the RoundTheDifference functor.
 */
template<typename D1, typename D2>
inline const Eigen::CwiseBinaryOp<RoundTheDifference<typename D1::Scalar>, const D1, const D2>
roundedDifference(
        const Eigen::MatrixBase<D1>& A,
        const Eigen::MatrixBase<D2>& B,
        unsigned short percentage=3)
{
    static RoundTheDifference<typename D1::Scalar> rounder(percentage);
    return A.binaryExpr(B, rounder);
}


}
}


#endif
