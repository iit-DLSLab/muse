/* CPYHDR { */
/*
 * This file is part of the 'iit-rbd' library.
 * Copyright © 2015 2016 2017, Marco Frigerio (marco.frigerio@iit.it)
 *
 * See the LICENSE file for more information.
 */
/* } CPYHDR */

#ifndef IIT_RBD_INTERNALS_
#define IIT_RBD_INTERNALS_

#include "InertiaMatrix.h"

namespace iit {
namespace rbd {
namespace internal {

/*
 * A container of the 9 coefficients of a 3x3 matrix.
 *
 * When multiple, repeated access to the elements of the matrix is necessary,
 * copying them once at the beginning in local variables might slightly improve
 * the performance (coefficient access typically comes at the cost of few
 * arithmetic operations)
 */
struct Mat3x3Coefficients {
    double XX,XY,XZ,YX,YY,YZ,ZX,ZY,ZZ;

    Mat3x3Coefficients() {}
    Mat3x3Coefficients(double xx, double xy, double xz,
                       double yx, double yy, double yz,
                       double zx, double zy, double zz) :
               XX(xx), XY(xy), XZ(xz),
               YX(yx), YY(yy), YZ(yz),
               ZX(zx), ZY(zy), ZZ(zz)
    {}

    template <typename D>
    Mat3x3Coefficients(const MatrixBase<D>& E) :
        XX(E(X,X)), XY(E(X,Y)), XZ(E(X,Z)),
        YX(E(Y,X)), YY(E(Y,Y)), YZ(E(Y,Z)),
        ZX(E(Z,X)), ZY(E(Z,Y)), ZZ(E(Z,Z))
    {}

    template <typename D>
    void read(const MatrixBase<D>& E)
    {
        XX = E(X,X); XY = E(X,Y); XZ = E(X,Z);
        YX = E(Y,X); YY = E(Y,Y); YZ = E(Y,Z);
        ZX = E(Z,X); ZY = E(Z,Y); ZZ = E(Z,Z);
    }
};

/*
 * A container of the 6 distinct coefficients of a symmetrix 3x3 matrix.
 *
 * See comment above.
 */
struct SymmMat3x3Coefficients {
    double XX,XY,XZ,YY,YZ,ZZ;

    SymmMat3x3Coefficients() {}

    SymmMat3x3Coefficients(double xx, double xy, double xz,
                                      double yy, double yz,
                                                 double zz) :
               XX(xx), XY(xy), XZ(xz),
                       YY(yy), YZ(yz),
                               ZZ(zz)
    {}

    template <typename D>
    SymmMat3x3Coefficients(const MatrixBase<D>& E) {
        read(E);
    }

    template <typename D>
    void read(const MatrixBase<D>& E)
    {
        XX = E(X,X); XY = E(X,Y); XZ = E(X,Z);
                     YY = E(Y,Y); YZ = E(Y,Z);
                                  ZZ = E(Z,Z);
    }
    template <typename D>
    void write(const MatrixBase<D>& Econst)
    {
        MatrixBase<D>& E = const_cast<MatrixBase<D>&>(Econst);
        E(X,X) = XX; E(X,Y) = XY; E(X,Z) = XZ;
        E(Y,X) = XY; E(Y,Y) = YY; E(Y,Z) = YZ;
        E(Z,X) = XZ; E(Z,Y) = YZ; E(Z,Z) = ZZ;
    }
};

/*
 * Performs the transformation of a symmetrix 3x3 matrix A, with the rotation
 * matrix E
 *    B = E * A * E^T
 *
 * These calculations are documented in the appendix of Roy's book
 */
inline void rot_symmetric_EAET(
          const Mat3x3Coefficients& E,
          const SymmMat3x3Coefficients& A,
                SymmMat3x3Coefficients& B)
{
    double LXX = A.XX - A.ZZ;
    double LXY = A.XY; // same as LYX
    double LYY = A.YY - A.ZZ;
    double LZX = 2*A.XZ;
    double LZY = 2*A.YZ;

    double yXX = E.YX*LXX + E.YY*LXY + E.YZ*LZX;
    double yXY = E.YX*LXY + E.YY*LYY + E.YZ*LZY;
    double yYX = E.ZX*LXX + E.ZY*LXY + E.ZZ*LZX;
    double yYY = E.ZX*LXY + E.ZY*LYY + E.ZZ*LZY;

    double v1 = -A.YZ;
    double v2 =  A.XZ;
    double EvX = E.XX*v1 + E.XY*v2;
    double EvY = E.YX*v1 + E.YY*v2;
    double EvZ = E.ZX*v1 + E.ZY*v2;

    B.XY = yXX * E.XX + yXY * E.XY + EvZ;
    B.XZ = yYX * E.XX + yYY * E.XY - EvY;
    B.YZ = yYX * E.YX + yYY * E.YY + EvX;

    double zYY = yXX * E.YX + yXY * E.YY;
    double zZZ = yYX * E.ZX + yYY * E.ZY;
    B.XX = LXX + LYY - zYY - zZZ + A.ZZ;
    B.YY = zYY + A.ZZ;
    B.ZZ = zZZ + A.ZZ;
}

/*
 * Performs the transformation of a 3x3 matrix A, with the rotation
 * matrix E
 *    B = E * A * E^T
 */
inline void rot_EAET(
        const Mat3x3Coefficients& E,
        const Mat3x3Coefficients& A,
              Mat3x3Coefficients& B)
{
    double LXX = A.XX - A.ZZ;
    double LXY = A.XY;
    double LYX = A.YX;
    double LYY = A.YY - A.ZZ;
    double LZX = A.ZX + A.XZ;
    double LZY = A.ZY + A.YZ;

    double v1 = -A.YZ;
    double v2 =  A.XZ;
    double EvX = E.XX*v1 + E.XY*v2;
    double EvY = E.YX*v1 + E.YY*v2;
    double EvZ = E.ZX*v1 + E.ZY*v2;

    double yXX = E.XX*LXX + E.XY*LYX + E.XZ*LZX;
    double yXY = E.XX*LXY + E.XY*LYY + E.XZ*LZY;
    double yYX = E.YX*LXX + E.YY*LYX + E.YZ*LZX;
    double yYY = E.YX*LXY + E.YY*LYY + E.YZ*LZY;
    double yZX = E.ZX*LXX + E.ZY*LYX + E.ZZ*LZX;
    double yZY = E.ZX*LXY + E.ZY*LYY + E.ZZ*LZY;

    B.XX = yXX*E.XX + yXY*E.XY + A.ZZ;
    B.YY = yYX*E.YX + yYY*E.YY + A.ZZ;
    B.ZZ = yZX*E.ZX + yZY*E.ZY + A.ZZ;

    B.XY = yXX*E.YX + yXY*E.YY - EvZ;
    B.YX = yYX*E.XX + yYY*E.XY + EvZ;
    B.XZ = yXX*E.ZX + yXY*E.ZY + EvY;
    B.ZX = yZX*E.XX + yZY*E.XY - EvY;
    B.YZ = yYX*E.ZX + yYY*E.ZY - EvX;
    B.ZY = yZX*E.YX + yZY*E.YY + EvX;
}


}
}
}

#endif
