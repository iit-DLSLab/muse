#ifndef IIT_COMMONS_DOG_JSIM_H_
#define IIT_COMMONS_DOG_JSIM_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/StateDependentMatrix.h>
#include <iit/commons/dog/declarations.h>
#include <iit/rbd/InertiaMatrix.h>

namespace iit {
namespace dog {

typedef iit::rbd::InertiaMatrixDense InertiaMatrix;
/**
 * The type of the Joint Space Inertia Matrix (JSIM) of the robot HyQ.
 */
class JSIMBase : public iit::rbd::StateDependentMatrix<iit::dog::JointState, 18, 18, JSIMBase>
{
public:
    typedef Eigen::Matrix<double,18,18> MatrixType;
    /** The type of the F sub-block of the floating-base JSIM */
    typedef const Eigen::Block<const MatrixType,6,12> BlockF_t;
    /** The type of the fixed-base sub-block of the JSIM */
    typedef const Eigen::Block<const MatrixType,12,12> BlockFixedBase_t;

public:
    virtual const JSIMBase& update(const iit::dog::JointState&) = 0;

    /**
         * Computes and saves the matrix L of the L^T L factorization of this JSIM.
         */
    virtual void computeL() = 0;
    /**
         * Computes and saves the inverse of this JSIM.
         * This function assumes that computeL() has been called already, since it
         * uses L to compute the inverse. The algorithm takes advantage of the branch
         * induced sparsity of the robot, if any.
         */
    virtual void computeInverse() = 0;
    /**
         * Returns an unmodifiable reference to the matrix L. See also computeL()
         */
    virtual const MatrixType& getL() const = 0;
    /**
         * Returns an unmodifiable reference to the inverse of this JSIM
         */
    virtual const MatrixType& getInverse() const = 0;

    /**
         * The spatial composite-inertia tensor of the robot base,
         * ie the inertia of the whole robot for the current configuration.
         * According to the convention of this class about the layout of the
         * floating-base JSIM, this tensor is the 6x6 upper left corner of
         * the JSIM itself.
         * \return the 6x6 InertiaMatrix that correspond to the spatial inertia
         *   tensor of the whole robot, according to the last joints configuration
         *   used to update this JSIM
         */
    virtual const InertiaMatrix& getWholeBodyInertia() const = 0;
    /**
         * The matrix that maps accelerations in the actual joints of the robot
         * to the spatial force acting on the floating-base of the robot.
         * This matrix is the F sub-block of the JSIM in Featherstone's notation.
         * \return the 6x12 upper right block of this JSIM
         */
    virtual const BlockF_t getF() const = 0;
    /**
         * The submatrix of this JSIM related only to the actual joints of the
         * robot (as for a fixed-base robot).
         * This matrix is the H sub-block of the JSIM in Featherstone's notation.
         * \return the 12x12 lower right block of this JSIM,
         *   which correspond to the fixed-base JSIM
         */
    virtual const BlockFixedBase_t getFixedBaseBlock() const = 0;
protected:
    /**
         * Computes and saves the inverse of the matrix L. See also computeL()
         */
    virtual void computeLInverse() = 0;
};

} // namespace dog
} // namespace iit
#endif

