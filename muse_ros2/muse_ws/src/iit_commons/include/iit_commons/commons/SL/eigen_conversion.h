#ifndef IIT_COMMONS_SL_UTILS_H_
#define IIT_COMMONS_SL_UTILS_H_

#include <iostream>
#include <Eigen/Dense>

/*!
 * \file
 * \brief This file contains conversion functions for matrices and vectors, between
 * SL and Eigen data types.
 */


namespace iit {
namespace commons {

/**
 * \defgroup SL SL
 * Miscellaneous utilities related to the "SL simulator and motor control"
 * software package, developed by Stefan Schaal.
 */

/**
 * This namespace contains general utilities related to the "SL simulator and
 * motor control" software package, developed by Stefan Schaal.
 * \ingroup SL
 */
namespace SL {


namespace internal {

static const bool SL_USES_ROWMAJOR_STORAGE = true;


template<typename EigenM>
struct same_storage {
    static const bool val =
            (SL_USES_ROWMAJOR_STORAGE ==
             (Eigen::internal::traits<EigenM>::Flags&Eigen::RowMajorBit) );
};


template<typename M, bool RowMajorStorage>
struct EigenConverter;

/*
 * Returns the pointer to the beginning of the memory chunk where all the
 * coefficients of the matrix are stored, consecutively,
 * This function must be consistent with the 'my_matrix' utility of SL
 */
inline const double* matrixToDataPointer(double const* const* m) {
    return &(m[1][1]);
}
inline const double* vectorToDataPointer(const double* v) {
    return &(v[1]);
}
inline double* matrixToDataPointer(double ** m) {
    return &(m[1][1]);
}
inline double* vectorToDataPointer(double* v) {
    return &(v[1]);
}

}// namespace internal



/**
 * \name Eigen matrix copy
 * Copy a SL matrix into an Eigen matrix.
 *
 * The method can be used both with actual matrices or with matrix expressions
 * such as block expressions (see the Eigen documentation).
 *
 * The amount of data to be copied is always determined from the size of the
 * Eigen parameter. If the size of the SL matrix does not match, well, seg fault...
 *
 * The copy is best efficient when \c dest is a fixed-size matrix stored with
 * row-major order (as in SL). Note that Eigen defaults to column-major though.
 */
///@{
/**
 * @param dest the Eigen matrix where to copy the coefficients
 * @param src the SL matrix to read the coefficients from
 */
template<typename M>
inline void copy(Eigen::PlainObjectBase<M>& dest, double const* const* src)
{
    internal::EigenConverter<M, internal::same_storage<M>::val >
    ::copy(dest, internal::matrixToDataPointer(src));
}
template<typename M>
inline void copy(const Eigen::MatrixBase<M>& dest, double const* const* src)
{
    internal::EigenConverter<M, internal::same_storage<M>::val >
    ::copy(const_cast< Eigen::MatrixBase<M>& >(dest), internal::matrixToDataPointer(src));
    // About the 'const_cast', see the Eigen documentation, section about
    //  writing functions taking Eigen expressions as parameters
}
/**
 * @param dest the SL matrix where to copy the coefficients
 * @param src  the Eigen matrix to read the coefficients from
 */
template<typename M>
inline void copy(double** dest, const Eigen::PlainObjectBase<M>& src)
{
    internal::EigenConverter<M, internal::same_storage<M>::val >
    ::copy(internal::matrixToDataPointer(dest), src);
}
template<typename M>
inline void copy(double** dest, const Eigen::MatrixBase<M>& src)
{
    internal::EigenConverter<M, internal::same_storage<M>::val >
    ::copy(internal::matrixToDataPointer(dest), src);
}
///@}

/**
 * \name Eigen vector copy
 * Copy a SL vector into an Eigen matrix.
 *
 * See the documentation of matrix copy
 */
///@{
/**
 * @param dest the Eigen matrix where to copy the coefficients
 * @param src the SL vector to read the coefficients from
 */
template<typename M>
inline void copy(Eigen::PlainObjectBase<M>& dest, const double* src)
{
    internal::EigenConverter<M, internal::same_storage<M>::val >
    ::copy(dest, internal::vectorToDataPointer(src));
}
template<typename M>
inline void copy(const Eigen::MatrixBase<M>& dest, const double* src)
{
    internal::EigenConverter<M, internal::same_storage<M>::val >
    ::copy(const_cast< Eigen::MatrixBase<M>& >(dest), internal::vectorToDataPointer(src));
    // About the 'const_cast', see the Eigen documentation, section about
    //  writing functions taking Eigen expressions as parameters
}
/**
 * @param dest the SL vector where to copy the coefficients
 * @param src  the Eigen matrix to read the coefficients from
 */
template<typename M>
inline void copy(double* dest, const Eigen::PlainObjectBase<M>& src)
{
    internal::EigenConverter<M, internal::same_storage<M>::val >
    ::copy(internal::vectorToDataPointer(dest), src);
}
template<typename M>
inline void copy(double* dest, const Eigen::MatrixBase<M>& src)
{
    internal::EigenConverter<M, internal::same_storage<M>::val >
    ::copy(internal::vectorToDataPointer(dest), src);
}
///@}


namespace internal {

template<typename M>
static inline void my_copy(
        Eigen::MatrixBase<M>& dest,
        const double* src,
        unsigned int rows,
        unsigned int cols)
{
    for(unsigned int r=0; r<rows; r++) {
         for(unsigned int c=0; c<cols; c++) {
             dest(r,c) = src[r*cols + c];
         }
     }
}

template<typename M>
static inline void my_copy(
        double* dest,
        const Eigen::MatrixBase<M>& src,
        unsigned int rows,
        unsigned int cols)
{
    for(unsigned int r=0; r<rows; r++) {
         for(unsigned int c=0; c<cols; c++) {
             dest[r*cols + c] = src(r,c);
         }
     }
}

template<typename M, bool RowMajorStorage>
struct EigenConverter {};



template<typename M>
struct EigenConverter<M, true>
{
    static inline void copy(Eigen::PlainObjectBase<M>& dest, const double* src)
    {
        //std::cout << "fast!" << std::endl;
        std::memcpy(dest.data(), src, dest.size() * sizeof(double));
    }

    static inline void copy(double* dest, const Eigen::PlainObjectBase<M>& src)
    {
        //std::cout << "fast!" << std::endl;
        std::memcpy(dest, src.data(), src.size() * sizeof(double));
    }

//    static inline void copy(Eigen::PlainObjectBase<M>& dest, const double* src, unsigned int size)
//    {
//        eigen_assert(size == dest.size());
//        std::memcpy(dest.data(), src, size*sizeof(double));
//    }
};


template<typename M>
struct EigenConverter<M, false> {
    static inline void copy(Eigen::MatrixBase<M>& dest, const double* src)
    {
        my_copy(dest, src, dest.rows(), dest.cols());
    }
    static inline void copy(double* dest, const Eigen::MatrixBase<M>& src)
    {
        my_copy(dest, src, src.rows(), src.cols());
    }

//    static inline void copy(Eigen::MatrixBase<M>& dest, const double* src, unsigned int rows, unsigned int cols)
//    {
//        eigen_assert(dest.size() == rows*cols);
//        my_copy(dest, src, rows, cols);
//    }

};



}

}
}
}


#endif /* IIT_COMMONS_SL_UTILS_H_ */
