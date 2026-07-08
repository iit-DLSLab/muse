#ifndef IIT_COMMONS_EULERANGLES_TRANSFORMS_H_
#define IIT_COMMONS_EULERANGLES_TRANSFORMS_H_

#include <Eigen/Dense>
#include <iit/rbd/TransformsBase.h>
#include "common.h"

namespace iit {
namespace eulerang {
namespace internal {

// The type of the "vector" with the status of the variables
typedef angles_state state_t;


template<class M>
class RotationMatrix : public iit::rbd::RotationTransformBase<state_t, M> {};


/**
 * The class with the rotation matrices (3x3)
 */
class RotationMatrices {
public:
    class Dummy {};
    typedef RotationMatrix<Dummy>::MatrixType MatrixType;
public:
    class Type_S_X_Exyz : public RotationMatrix<Type_S_X_Exyz>
    {
    public:
        Type_S_X_Exyz();
        const Type_S_X_Exyz& update(const state_t&);
    protected:
    };

    class Type_S_X_Exzy : public RotationMatrix<Type_S_X_Exzy>
    {
    public:
        Type_S_X_Exzy();
        const Type_S_X_Exzy& update(const state_t&);
    protected:
    };

    class Type_S_X_Eyxz : public RotationMatrix<Type_S_X_Eyxz>
    {
    public:
        Type_S_X_Eyxz();
        const Type_S_X_Eyxz& update(const state_t&);
    protected:
    };

    class Type_S_X_Eyzx : public RotationMatrix<Type_S_X_Eyzx>
    {
    public:
        Type_S_X_Eyzx();
        const Type_S_X_Eyzx& update(const state_t&);
    protected:
    };

    class Type_S_X_Ezxy : public RotationMatrix<Type_S_X_Ezxy>
    {
    public:
        Type_S_X_Ezxy();
        const Type_S_X_Ezxy& update(const state_t&);
    protected:
    };

    class Type_S_X_Ezyx : public RotationMatrix<Type_S_X_Ezyx>
    {
    public:
        Type_S_X_Ezyx();
        const Type_S_X_Ezyx& update(const state_t&);
    protected:
    };

    class Type_S_X_Exyx : public RotationMatrix<Type_S_X_Exyx>
    {
    public:
        Type_S_X_Exyx();
        const Type_S_X_Exyx& update(const state_t&);
    protected:
    };

    class Type_S_X_Exzx : public RotationMatrix<Type_S_X_Exzx>
    {
    public:
        Type_S_X_Exzx();
        const Type_S_X_Exzx& update(const state_t&);
    protected:
    };

    class Type_S_X_Eyxy : public RotationMatrix<Type_S_X_Eyxy>
    {
    public:
        Type_S_X_Eyxy();
        const Type_S_X_Eyxy& update(const state_t&);
    protected:
    };

    class Type_S_X_Eyzy : public RotationMatrix<Type_S_X_Eyzy>
    {
    public:
        Type_S_X_Eyzy();
        const Type_S_X_Eyzy& update(const state_t&);
    protected:
    };

    class Type_S_X_Ezxz : public RotationMatrix<Type_S_X_Ezxz>
    {
    public:
        Type_S_X_Ezxz();
        const Type_S_X_Ezxz& update(const state_t&);
    protected:
    };

    class Type_S_X_Ezyz : public RotationMatrix<Type_S_X_Ezyz>
    {
    public:
        Type_S_X_Ezyz();
        const Type_S_X_Ezyz& update(const state_t&);
    protected:
    };

    class Type_Exyz_X_S : public RotationMatrix<Type_Exyz_X_S>
        {
        public:
            Type_Exyz_X_S();
            const Type_Exyz_X_S& update(const state_t&);
        protected:
        };

        class Type_Exzy_X_S : public RotationMatrix<Type_Exzy_X_S>
        {
        public:
            Type_Exzy_X_S();
            const Type_Exzy_X_S& update(const state_t&);
        protected:
        };

        class Type_Eyxz_X_S : public RotationMatrix<Type_Eyxz_X_S>
        {
        public:
            Type_Eyxz_X_S();
            const Type_Eyxz_X_S& update(const state_t&);
        protected:
        };

        class Type_Eyzx_X_S : public RotationMatrix<Type_Eyzx_X_S>
        {
        public:
            Type_Eyzx_X_S();
            const Type_Eyzx_X_S& update(const state_t&);
        protected:
        };

        class Type_Ezxy_X_S : public RotationMatrix<Type_Ezxy_X_S>
        {
        public:
            Type_Ezxy_X_S();
            const Type_Ezxy_X_S& update(const state_t&);
        protected:
        };

        class Type_Ezyx_X_S : public RotationMatrix<Type_Ezyx_X_S>
        {
        public:
            Type_Ezyx_X_S();
            const Type_Ezyx_X_S& update(const state_t&);
        protected:
        };

        class Type_Exyx_X_S : public RotationMatrix<Type_Exyx_X_S>
        {
        public:
            Type_Exyx_X_S();
            const Type_Exyx_X_S& update(const state_t&);
        protected:
        };

        class Type_Exzx_X_S : public RotationMatrix<Type_Exzx_X_S>
        {
        public:
            Type_Exzx_X_S();
            const Type_Exzx_X_S& update(const state_t&);
        protected:
        };

        class Type_Eyxy_X_S : public RotationMatrix<Type_Eyxy_X_S>
        {
        public:
            Type_Eyxy_X_S();
            const Type_Eyxy_X_S& update(const state_t&);
        protected:
        };

        class Type_Eyzy_X_S : public RotationMatrix<Type_Eyzy_X_S>
        {
        public:
            Type_Eyzy_X_S();
            const Type_Eyzy_X_S& update(const state_t&);
        protected:
        };

        class Type_Ezxz_X_S : public RotationMatrix<Type_Ezxz_X_S>
        {
        public:
            Type_Ezxz_X_S();
            const Type_Ezxz_X_S& update(const state_t&);
        protected:
        };

        class Type_Ezyz_X_S : public RotationMatrix<Type_Ezyz_X_S>
        {
        public:
            Type_Ezyz_X_S();
            const Type_Ezyz_X_S& update(const state_t&);
        protected:
        };

    public:
        RotationMatrices();
        void updateParameters();
        Type_S_X_Exyz S_X_Exyz;
        Type_S_X_Exzy S_X_Exzy;
        Type_S_X_Eyxz S_X_Eyxz;
        Type_S_X_Eyzx S_X_Eyzx;
        Type_S_X_Ezxy S_X_Ezxy;
        Type_S_X_Ezyx S_X_Ezyx;
        Type_S_X_Exyx S_X_Exyx;
        Type_S_X_Exzx S_X_Exzx;
        Type_S_X_Eyxy S_X_Eyxy;
        Type_S_X_Eyzy S_X_Eyzy;
        Type_S_X_Ezxz S_X_Ezxz;
        Type_S_X_Ezyz S_X_Ezyz;
        Type_Exyz_X_S Exyz_X_S;
        Type_Exzy_X_S Exzy_X_S;
        Type_Eyxz_X_S Eyxz_X_S;
        Type_Eyzx_X_S Eyzx_X_S;
        Type_Ezxy_X_S Ezxy_X_S;
        Type_Ezyx_X_S Ezyx_X_S;
        Type_Exyx_X_S Exyx_X_S;
        Type_Exzx_X_S Exzx_X_S;
        Type_Eyxy_X_S Eyxy_X_S;
        Type_Eyzy_X_S Eyzy_X_S;
        Type_Ezxz_X_S Ezxz_X_S;
        Type_Ezyz_X_S Ezyz_X_S;


protected:

}; //class 'Rotations'

}
}
}

#endif
