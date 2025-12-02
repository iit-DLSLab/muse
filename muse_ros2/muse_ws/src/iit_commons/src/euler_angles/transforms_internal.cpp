#include <iit/commons/euler_angles/transforms_internal.h>


iit::eulerang::internal::RotationMatrices::RotationMatrices
    ()
     :
    S_X_Exyz(),
    S_X_Exzy(),
    S_X_Eyxz(),
    S_X_Eyzx(),
    S_X_Ezxy(),
    S_X_Ezyx(),
    S_X_Exyx(),
    S_X_Exzx(),
    S_X_Eyxy(),
    S_X_Eyzy(),
    S_X_Ezxz(),
    S_X_Ezyz()
{
    updateParameters();
}
void iit::eulerang::internal::RotationMatrices::updateParameters() {
}

iit::eulerang::internal::RotationMatrices::Type_S_X_Exyz::Type_S_X_Exyz()
{
}
const iit::eulerang::internal::RotationMatrices::Type_S_X_Exyz& iit::eulerang::internal::RotationMatrices::Type_S_X_Exyz::update(const state_t& state) {
    static double sin__a3__;
    static double sin__a2__;
    static double sin__a1__;
    static double cos__a2__;
    static double cos__a3__;
    static double cos__a1__;

    sin__a3__ = std::sin( state.a3);
    sin__a2__ = std::sin( state.a2);
    sin__a1__ = std::sin( state.a1);
    cos__a2__ = std::cos( state.a2);
    cos__a3__ = std::cos( state.a3);
    cos__a1__ = std::cos( state.a1);

    (*this)(0,0) = ( cos__a2__ *  cos__a3__);
    (*this)(0,1) = (- cos__a2__ *  sin__a3__);
    (*this)(0,2) =  sin__a2__;
    (*this)(1,0) = (( cos__a1__ *  sin__a3__) + (( sin__a1__ *  sin__a2__) *  cos__a3__));
    (*this)(1,1) = (( cos__a1__ *  cos__a3__) - (( sin__a1__ *  sin__a2__) *  sin__a3__));
    (*this)(1,2) = (- sin__a1__ *  cos__a2__);
    (*this)(2,0) = (( sin__a1__ *  sin__a3__) - (( cos__a1__ *  sin__a2__) *  cos__a3__));
    (*this)(2,1) = ((( cos__a1__ *  sin__a2__) *  sin__a3__) + ( sin__a1__ *  cos__a3__));
    (*this)(2,2) = ( cos__a1__ *  cos__a2__);
    return *this;
}
iit::eulerang::internal::RotationMatrices::Type_S_X_Exzy::Type_S_X_Exzy()
{
}
const iit::eulerang::internal::RotationMatrices::Type_S_X_Exzy& iit::eulerang::internal::RotationMatrices::Type_S_X_Exzy::update(const state_t& state) {
    static double sin__a2__;
    static double sin__a3__;
    static double sin__a1__;
    static double cos__a2__;
    static double cos__a3__;
    static double cos__a1__;

    sin__a2__ = std::sin( state.a2);
    sin__a3__ = std::sin( state.a3);
    sin__a1__ = std::sin( state.a1);
    cos__a2__ = std::cos( state.a2);
    cos__a3__ = std::cos( state.a3);
    cos__a1__ = std::cos( state.a1);

    (*this)(0,0) = ( cos__a2__ *  cos__a3__);
    (*this)(0,1) = - sin__a2__;
    (*this)(0,2) = ( cos__a2__ *  sin__a3__);
    (*this)(1,0) = (( sin__a1__ *  sin__a3__) + (( cos__a1__ *  sin__a2__) *  cos__a3__));
    (*this)(1,1) = ( cos__a1__ *  cos__a2__);
    (*this)(1,2) = ((( cos__a1__ *  sin__a2__) *  sin__a3__) - ( sin__a1__ *  cos__a3__));
    (*this)(2,0) = ((( sin__a1__ *  sin__a2__) *  cos__a3__) - ( cos__a1__ *  sin__a3__));
    (*this)(2,1) = ( sin__a1__ *  cos__a2__);
    (*this)(2,2) = ((( sin__a1__ *  sin__a2__) *  sin__a3__) + ( cos__a1__ *  cos__a3__));
    return *this;
}
iit::eulerang::internal::RotationMatrices::Type_S_X_Eyxz::Type_S_X_Eyxz()
{
}
const iit::eulerang::internal::RotationMatrices::Type_S_X_Eyxz& iit::eulerang::internal::RotationMatrices::Type_S_X_Eyxz::update(const state_t& state) {
    static double sin__a1__;
    static double sin__a2__;
    static double sin__a3__;
    static double cos__a1__;
    static double cos__a3__;
    static double cos__a2__;

    sin__a1__ = std::sin( state.a1);
    sin__a2__ = std::sin( state.a2);
    sin__a3__ = std::sin( state.a3);
    cos__a1__ = std::cos( state.a1);
    cos__a3__ = std::cos( state.a3);
    cos__a2__ = std::cos( state.a2);

    (*this)(0,0) = ((( sin__a1__ *  sin__a2__) *  sin__a3__) + ( cos__a1__ *  cos__a3__));
    (*this)(0,1) = ((( sin__a1__ *  sin__a2__) *  cos__a3__) - ( cos__a1__ *  sin__a3__));
    (*this)(0,2) = ( sin__a1__ *  cos__a2__);
    (*this)(1,0) = ( cos__a2__ *  sin__a3__);
    (*this)(1,1) = ( cos__a2__ *  cos__a3__);
    (*this)(1,2) = - sin__a2__;
    (*this)(2,0) = ((( cos__a1__ *  sin__a2__) *  sin__a3__) - ( sin__a1__ *  cos__a3__));
    (*this)(2,1) = (( sin__a1__ *  sin__a3__) + (( cos__a1__ *  sin__a2__) *  cos__a3__));
    (*this)(2,2) = ( cos__a1__ *  cos__a2__);
    return *this;
}
iit::eulerang::internal::RotationMatrices::Type_S_X_Eyzx::Type_S_X_Eyzx()
{
}
const iit::eulerang::internal::RotationMatrices::Type_S_X_Eyzx& iit::eulerang::internal::RotationMatrices::Type_S_X_Eyzx::update(const state_t& state) {
    static double sin__a1__;
    static double sin__a3__;
    static double sin__a2__;
    static double cos__a1__;
    static double cos__a2__;
    static double cos__a3__;

    sin__a1__ = std::sin( state.a1);
    sin__a3__ = std::sin( state.a3);
    sin__a2__ = std::sin( state.a2);
    cos__a1__ = std::cos( state.a1);
    cos__a2__ = std::cos( state.a2);
    cos__a3__ = std::cos( state.a3);

    (*this)(0,0) = ( cos__a1__ *  cos__a2__);
    (*this)(0,1) = (( sin__a1__ *  sin__a3__) - (( cos__a1__ *  sin__a2__) *  cos__a3__));
    (*this)(0,2) = ((( cos__a1__ *  sin__a2__) *  sin__a3__) + ( sin__a1__ *  cos__a3__));
    (*this)(1,0) =  sin__a2__;
    (*this)(1,1) = ( cos__a2__ *  cos__a3__);
    (*this)(1,2) = (- cos__a2__ *  sin__a3__);
    (*this)(2,0) = (- sin__a1__ *  cos__a2__);
    (*this)(2,1) = (( cos__a1__ *  sin__a3__) + (( sin__a1__ *  sin__a2__) *  cos__a3__));
    (*this)(2,2) = (( cos__a1__ *  cos__a3__) - (( sin__a1__ *  sin__a2__) *  sin__a3__));
    return *this;
}
iit::eulerang::internal::RotationMatrices::Type_S_X_Ezxy::Type_S_X_Ezxy()
{
}
const iit::eulerang::internal::RotationMatrices::Type_S_X_Ezxy& iit::eulerang::internal::RotationMatrices::Type_S_X_Ezxy::update(const state_t& state) {
    static double sin__a1__;
    static double sin__a2__;
    static double sin__a3__;
    static double cos__a1__;
    static double cos__a3__;
    static double cos__a2__;

    sin__a1__ = std::sin( state.a1);
    sin__a2__ = std::sin( state.a2);
    sin__a3__ = std::sin( state.a3);
    cos__a1__ = std::cos( state.a1);
    cos__a3__ = std::cos( state.a3);
    cos__a2__ = std::cos( state.a2);

    (*this)(0,0) = (( cos__a1__ *  cos__a3__) - (( sin__a1__ *  sin__a2__) *  sin__a3__));
    (*this)(0,1) = (- sin__a1__ *  cos__a2__);
    (*this)(0,2) = (( cos__a1__ *  sin__a3__) + (( sin__a1__ *  sin__a2__) *  cos__a3__));
    (*this)(1,0) = ((( cos__a1__ *  sin__a2__) *  sin__a3__) + ( sin__a1__ *  cos__a3__));
    (*this)(1,1) = ( cos__a1__ *  cos__a2__);
    (*this)(1,2) = (( sin__a1__ *  sin__a3__) - (( cos__a1__ *  sin__a2__) *  cos__a3__));
    (*this)(2,0) = (- cos__a2__ *  sin__a3__);
    (*this)(2,1) =  sin__a2__;
    (*this)(2,2) = ( cos__a2__ *  cos__a3__);
    return *this;
}
iit::eulerang::internal::RotationMatrices::Type_S_X_Ezyx::Type_S_X_Ezyx()
{
}
const iit::eulerang::internal::RotationMatrices::Type_S_X_Ezyx& iit::eulerang::internal::RotationMatrices::Type_S_X_Ezyx::update(const state_t& state) {
    static double sin__a2__;
    static double sin__a3__;
    static double sin__a1__;
    static double cos__a1__;
    static double cos__a2__;
    static double cos__a3__;

    sin__a2__ = std::sin( state.a2);
    sin__a3__ = std::sin( state.a3);
    sin__a1__ = std::sin( state.a1);
    cos__a1__ = std::cos( state.a1);
    cos__a2__ = std::cos( state.a2);
    cos__a3__ = std::cos( state.a3);

    (*this)(0,0) = ( cos__a1__ *  cos__a2__);
    (*this)(0,1) = ((( cos__a1__ *  sin__a2__) *  sin__a3__) - ( sin__a1__ *  cos__a3__));
    (*this)(0,2) = (( sin__a1__ *  sin__a3__) + (( cos__a1__ *  sin__a2__) *  cos__a3__));
    (*this)(1,0) = ( sin__a1__ *  cos__a2__);
    (*this)(1,1) = ((( sin__a1__ *  sin__a2__) *  sin__a3__) + ( cos__a1__ *  cos__a3__));
    (*this)(1,2) = ((( sin__a1__ *  sin__a2__) *  cos__a3__) - ( cos__a1__ *  sin__a3__));
    (*this)(2,0) = - sin__a2__;
    (*this)(2,1) = ( cos__a2__ *  sin__a3__);
    (*this)(2,2) = ( cos__a2__ *  cos__a3__);
    return *this;
}
iit::eulerang::internal::RotationMatrices::Type_S_X_Exyx::Type_S_X_Exyx()
{
}
const iit::eulerang::internal::RotationMatrices::Type_S_X_Exyx& iit::eulerang::internal::RotationMatrices::Type_S_X_Exyx::update(const state_t& state) {
    static double sin__a2__;
    static double sin__a3__;
    static double sin__a1__;
    static double cos__a2__;
    static double cos__a3__;
    static double cos__a1__;

    sin__a2__ = std::sin( state.a2);
    sin__a3__ = std::sin( state.a3);
    sin__a1__ = std::sin( state.a1);
    cos__a2__ = std::cos( state.a2);
    cos__a3__ = std::cos( state.a3);
    cos__a1__ = std::cos( state.a1);

    (*this)(0,0) =  cos__a2__;
    (*this)(0,1) = ( sin__a2__ *  sin__a3__);
    (*this)(0,2) = ( sin__a2__ *  cos__a3__);
    (*this)(1,0) = ( sin__a1__ *  sin__a2__);
    (*this)(1,1) = (( cos__a1__ *  cos__a3__) - (( sin__a1__ *  cos__a2__) *  sin__a3__));
    (*this)(1,2) = ((- cos__a1__ *  sin__a3__) - (( sin__a1__ *  cos__a2__) *  cos__a3__));
    (*this)(2,0) = (- cos__a1__ *  sin__a2__);
    (*this)(2,1) = ((( cos__a1__ *  cos__a2__) *  sin__a3__) + ( sin__a1__ *  cos__a3__));
    (*this)(2,2) = ((( cos__a1__ *  cos__a2__) *  cos__a3__) - ( sin__a1__ *  sin__a3__));
    return *this;
}
iit::eulerang::internal::RotationMatrices::Type_S_X_Exzx::Type_S_X_Exzx()
{
}
const iit::eulerang::internal::RotationMatrices::Type_S_X_Exzx& iit::eulerang::internal::RotationMatrices::Type_S_X_Exzx::update(const state_t& state) {
    static double sin__a2__;
    static double sin__a3__;
    static double sin__a1__;
    static double cos__a2__;
    static double cos__a3__;
    static double cos__a1__;

    sin__a2__ = std::sin( state.a2);
    sin__a3__ = std::sin( state.a3);
    sin__a1__ = std::sin( state.a1);
    cos__a2__ = std::cos( state.a2);
    cos__a3__ = std::cos( state.a3);
    cos__a1__ = std::cos( state.a1);

    (*this)(0,0) =  cos__a2__;
    (*this)(0,1) = (- sin__a2__ *  cos__a3__);
    (*this)(0,2) = ( sin__a2__ *  sin__a3__);
    (*this)(1,0) = ( cos__a1__ *  sin__a2__);
    (*this)(1,1) = ((( cos__a1__ *  cos__a2__) *  cos__a3__) - ( sin__a1__ *  sin__a3__));
    (*this)(1,2) = (((- cos__a1__ *  cos__a2__) *  sin__a3__) - ( sin__a1__ *  cos__a3__));
    (*this)(2,0) = ( sin__a1__ *  sin__a2__);
    (*this)(2,1) = (( cos__a1__ *  sin__a3__) + (( sin__a1__ *  cos__a2__) *  cos__a3__));
    (*this)(2,2) = (( cos__a1__ *  cos__a3__) - (( sin__a1__ *  cos__a2__) *  sin__a3__));
    return *this;
}
iit::eulerang::internal::RotationMatrices::Type_S_X_Eyxy::Type_S_X_Eyxy()
{
}
const iit::eulerang::internal::RotationMatrices::Type_S_X_Eyxy& iit::eulerang::internal::RotationMatrices::Type_S_X_Eyxy::update(const state_t& state) {
    static double sin__a1__;
    static double sin__a3__;
    static double sin__a2__;
    static double cos__a1__;
    static double cos__a3__;
    static double cos__a2__;

    sin__a1__ = std::sin( state.a1);
    sin__a3__ = std::sin( state.a3);
    sin__a2__ = std::sin( state.a2);
    cos__a1__ = std::cos( state.a1);
    cos__a3__ = std::cos( state.a3);
    cos__a2__ = std::cos( state.a2);

    (*this)(0,0) = (( cos__a1__ *  cos__a3__) - (( sin__a1__ *  cos__a2__) *  sin__a3__));
    (*this)(0,1) = ( sin__a1__ *  sin__a2__);
    (*this)(0,2) = (( cos__a1__ *  sin__a3__) + (( sin__a1__ *  cos__a2__) *  cos__a3__));
    (*this)(1,0) = ( sin__a2__ *  sin__a3__);
    (*this)(1,1) =  cos__a2__;
    (*this)(1,2) = (- sin__a2__ *  cos__a3__);
    (*this)(2,0) = (((- cos__a1__ *  cos__a2__) *  sin__a3__) - ( sin__a1__ *  cos__a3__));
    (*this)(2,1) = ( cos__a1__ *  sin__a2__);
    (*this)(2,2) = ((( cos__a1__ *  cos__a2__) *  cos__a3__) - ( sin__a1__ *  sin__a3__));
    return *this;
}
iit::eulerang::internal::RotationMatrices::Type_S_X_Eyzy::Type_S_X_Eyzy()
{
}
const iit::eulerang::internal::RotationMatrices::Type_S_X_Eyzy& iit::eulerang::internal::RotationMatrices::Type_S_X_Eyzy::update(const state_t& state) {
    static double sin__a1__;
    static double sin__a3__;
    static double sin__a2__;
    static double cos__a1__;
    static double cos__a2__;
    static double cos__a3__;

    sin__a1__ = std::sin( state.a1);
    sin__a3__ = std::sin( state.a3);
    sin__a2__ = std::sin( state.a2);
    cos__a1__ = std::cos( state.a1);
    cos__a2__ = std::cos( state.a2);
    cos__a3__ = std::cos( state.a3);

    (*this)(0,0) = ((( cos__a1__ *  cos__a2__) *  cos__a3__) - ( sin__a1__ *  sin__a3__));
    (*this)(0,1) = (- cos__a1__ *  sin__a2__);
    (*this)(0,2) = ((( cos__a1__ *  cos__a2__) *  sin__a3__) + ( sin__a1__ *  cos__a3__));
    (*this)(1,0) = ( sin__a2__ *  cos__a3__);
    (*this)(1,1) =  cos__a2__;
    (*this)(1,2) = ( sin__a2__ *  sin__a3__);
    (*this)(2,0) = ((- cos__a1__ *  sin__a3__) - (( sin__a1__ *  cos__a2__) *  cos__a3__));
    (*this)(2,1) = ( sin__a1__ *  sin__a2__);
    (*this)(2,2) = (( cos__a1__ *  cos__a3__) - (( sin__a1__ *  cos__a2__) *  sin__a3__));
    return *this;
}
iit::eulerang::internal::RotationMatrices::Type_S_X_Ezxz::Type_S_X_Ezxz()
{
}
const iit::eulerang::internal::RotationMatrices::Type_S_X_Ezxz& iit::eulerang::internal::RotationMatrices::Type_S_X_Ezxz::update(const state_t& state) {
    static double sin__a1__;
    static double sin__a3__;
    static double sin__a2__;
    static double cos__a1__;
    static double cos__a3__;
    static double cos__a2__;

    sin__a1__ = std::sin( state.a1);
    sin__a3__ = std::sin( state.a3);
    sin__a2__ = std::sin( state.a2);
    cos__a1__ = std::cos( state.a1);
    cos__a3__ = std::cos( state.a3);
    cos__a2__ = std::cos( state.a2);

    (*this)(0,0) = (( cos__a1__ *  cos__a3__) - (( sin__a1__ *  cos__a2__) *  sin__a3__));
    (*this)(0,1) = ((- cos__a1__ *  sin__a3__) - (( sin__a1__ *  cos__a2__) *  cos__a3__));
    (*this)(0,2) = ( sin__a1__ *  sin__a2__);
    (*this)(1,0) = ((( cos__a1__ *  cos__a2__) *  sin__a3__) + ( sin__a1__ *  cos__a3__));
    (*this)(1,1) = ((( cos__a1__ *  cos__a2__) *  cos__a3__) - ( sin__a1__ *  sin__a3__));
    (*this)(1,2) = (- cos__a1__ *  sin__a2__);
    (*this)(2,0) = ( sin__a2__ *  sin__a3__);
    (*this)(2,1) = ( sin__a2__ *  cos__a3__);
    (*this)(2,2) =  cos__a2__;
    return *this;
}
iit::eulerang::internal::RotationMatrices::Type_S_X_Ezyz::Type_S_X_Ezyz()
{
}
const iit::eulerang::internal::RotationMatrices::Type_S_X_Ezyz& iit::eulerang::internal::RotationMatrices::Type_S_X_Ezyz::update(const state_t& state) {
    static double sin__a1__;
    static double sin__a3__;
    static double sin__a2__;
    static double cos__a1__;
    static double cos__a2__;
    static double cos__a3__;

    sin__a1__ = std::sin( state.a1);
    sin__a3__ = std::sin( state.a3);
    sin__a2__ = std::sin( state.a2);
    cos__a1__ = std::cos( state.a1);
    cos__a2__ = std::cos( state.a2);
    cos__a3__ = std::cos( state.a3);

    (*this)(0,0) = ((( cos__a1__ *  cos__a2__) *  cos__a3__) - ( sin__a1__ *  sin__a3__));
    (*this)(0,1) = (((- cos__a1__ *  cos__a2__) *  sin__a3__) - ( sin__a1__ *  cos__a3__));
    (*this)(0,2) = ( cos__a1__ *  sin__a2__);
    (*this)(1,0) = (( cos__a1__ *  sin__a3__) + (( sin__a1__ *  cos__a2__) *  cos__a3__));
    (*this)(1,1) = (( cos__a1__ *  cos__a3__) - (( sin__a1__ *  cos__a2__) *  sin__a3__));
    (*this)(1,2) = ( sin__a1__ *  sin__a2__);
    (*this)(2,0) = (- sin__a2__ *  cos__a3__);
    (*this)(2,1) = ( sin__a2__ *  sin__a3__);
    (*this)(2,2) =  cos__a2__;
    return *this;
}

iit::eulerang::internal::RotationMatrices::Type_Exyz_X_S::Type_Exyz_X_S()
{
}
const iit::eulerang::internal::RotationMatrices::Type_Exyz_X_S& iit::eulerang::internal::RotationMatrices::Type_Exyz_X_S::update(const state_t& state) {
    static double sin__a3__;
    static double sin__a1__;
    static double sin__a2__;
    static double cos__a2__;
    static double cos__a3__;
    static double cos__a1__;

    sin__a3__ = std::sin( state.a3);
    sin__a1__ = std::sin( state.a1);
    sin__a2__ = std::sin( state.a2);
    cos__a2__ = std::cos( state.a2);
    cos__a3__ = std::cos( state.a3);
    cos__a1__ = std::cos( state.a1);

    (*this)(0,0) = ( cos__a2__ *  cos__a3__);
    (*this)(0,1) = (( cos__a1__ *  sin__a3__) + (( sin__a1__ *  sin__a2__) *  cos__a3__));
    (*this)(0,2) = (( sin__a1__ *  sin__a3__) - (( cos__a1__ *  sin__a2__) *  cos__a3__));
    (*this)(1,0) = (- cos__a2__ *  sin__a3__);
    (*this)(1,1) = (( cos__a1__ *  cos__a3__) - (( sin__a1__ *  sin__a2__) *  sin__a3__));
    (*this)(1,2) = ((( cos__a1__ *  sin__a2__) *  sin__a3__) + ( sin__a1__ *  cos__a3__));
    (*this)(2,0) =  sin__a2__;
    (*this)(2,1) = (- sin__a1__ *  cos__a2__);
    (*this)(2,2) = ( cos__a1__ *  cos__a2__);
    return *this;
}
iit::eulerang::internal::RotationMatrices::Type_Exzy_X_S::Type_Exzy_X_S()
{
}
const iit::eulerang::internal::RotationMatrices::Type_Exzy_X_S& iit::eulerang::internal::RotationMatrices::Type_Exzy_X_S::update(const state_t& state) {
    static double sin__a1__;
    static double sin__a3__;
    static double sin__a2__;
    static double cos__a2__;
    static double cos__a3__;
    static double cos__a1__;

    sin__a1__ = std::sin( state.a1);
    sin__a3__ = std::sin( state.a3);
    sin__a2__ = std::sin( state.a2);
    cos__a2__ = std::cos( state.a2);
    cos__a3__ = std::cos( state.a3);
    cos__a1__ = std::cos( state.a1);

    (*this)(0,0) = ( cos__a2__ *  cos__a3__);
    (*this)(0,1) = (( sin__a1__ *  sin__a3__) + (( cos__a1__ *  sin__a2__) *  cos__a3__));
    (*this)(0,2) = ((( sin__a1__ *  sin__a2__) *  cos__a3__) - ( cos__a1__ *  sin__a3__));
    (*this)(1,0) = - sin__a2__;
    (*this)(1,1) = ( cos__a1__ *  cos__a2__);
    (*this)(1,2) = ( sin__a1__ *  cos__a2__);
    (*this)(2,0) = ( cos__a2__ *  sin__a3__);
    (*this)(2,1) = ((( cos__a1__ *  sin__a2__) *  sin__a3__) - ( sin__a1__ *  cos__a3__));
    (*this)(2,2) = ((( sin__a1__ *  sin__a2__) *  sin__a3__) + ( cos__a1__ *  cos__a3__));
    return *this;
}
iit::eulerang::internal::RotationMatrices::Type_Eyxz_X_S::Type_Eyxz_X_S()
{
}
const iit::eulerang::internal::RotationMatrices::Type_Eyxz_X_S& iit::eulerang::internal::RotationMatrices::Type_Eyxz_X_S::update(const state_t& state) {
    static double sin__a1__;
    static double sin__a2__;
    static double sin__a3__;
    static double cos__a1__;
    static double cos__a3__;
    static double cos__a2__;

    sin__a1__ = std::sin( state.a1);
    sin__a2__ = std::sin( state.a2);
    sin__a3__ = std::sin( state.a3);
    cos__a1__ = std::cos( state.a1);
    cos__a3__ = std::cos( state.a3);
    cos__a2__ = std::cos( state.a2);

    (*this)(0,0) = ((( sin__a1__ *  sin__a2__) *  sin__a3__) + ( cos__a1__ *  cos__a3__));
    (*this)(0,1) = ( cos__a2__ *  sin__a3__);
    (*this)(0,2) = ((( cos__a1__ *  sin__a2__) *  sin__a3__) - ( sin__a1__ *  cos__a3__));
    (*this)(1,0) = ((( sin__a1__ *  sin__a2__) *  cos__a3__) - ( cos__a1__ *  sin__a3__));
    (*this)(1,1) = ( cos__a2__ *  cos__a3__);
    (*this)(1,2) = (( sin__a1__ *  sin__a3__) + (( cos__a1__ *  sin__a2__) *  cos__a3__));
    (*this)(2,0) = ( sin__a1__ *  cos__a2__);
    (*this)(2,1) = - sin__a2__;
    (*this)(2,2) = ( cos__a1__ *  cos__a2__);
    return *this;
}
iit::eulerang::internal::RotationMatrices::Type_Eyzx_X_S::Type_Eyzx_X_S()
{
}
const iit::eulerang::internal::RotationMatrices::Type_Eyzx_X_S& iit::eulerang::internal::RotationMatrices::Type_Eyzx_X_S::update(const state_t& state) {
    static double sin__a2__;
    static double sin__a1__;
    static double sin__a3__;
    static double cos__a1__;
    static double cos__a2__;
    static double cos__a3__;

    sin__a2__ = std::sin( state.a2);
    sin__a1__ = std::sin( state.a1);
    sin__a3__ = std::sin( state.a3);
    cos__a1__ = std::cos( state.a1);
    cos__a2__ = std::cos( state.a2);
    cos__a3__ = std::cos( state.a3);

    (*this)(0,0) = ( cos__a1__ *  cos__a2__);
    (*this)(0,1) =  sin__a2__;
    (*this)(0,2) = (- sin__a1__ *  cos__a2__);
    (*this)(1,0) = (( sin__a1__ *  sin__a3__) - (( cos__a1__ *  sin__a2__) *  cos__a3__));
    (*this)(1,1) = ( cos__a2__ *  cos__a3__);
    (*this)(1,2) = (( cos__a1__ *  sin__a3__) + (( sin__a1__ *  sin__a2__) *  cos__a3__));
    (*this)(2,0) = ((( cos__a1__ *  sin__a2__) *  sin__a3__) + ( sin__a1__ *  cos__a3__));
    (*this)(2,1) = (- cos__a2__ *  sin__a3__);
    (*this)(2,2) = (( cos__a1__ *  cos__a3__) - (( sin__a1__ *  sin__a2__) *  sin__a3__));
    return *this;
}
iit::eulerang::internal::RotationMatrices::Type_Ezxy_X_S::Type_Ezxy_X_S()
{
}
const iit::eulerang::internal::RotationMatrices::Type_Ezxy_X_S& iit::eulerang::internal::RotationMatrices::Type_Ezxy_X_S::update(const state_t& state) {
    static double sin__a1__;
    static double sin__a2__;
    static double sin__a3__;
    static double cos__a1__;
    static double cos__a3__;
    static double cos__a2__;

    sin__a1__ = std::sin( state.a1);
    sin__a2__ = std::sin( state.a2);
    sin__a3__ = std::sin( state.a3);
    cos__a1__ = std::cos( state.a1);
    cos__a3__ = std::cos( state.a3);
    cos__a2__ = std::cos( state.a2);

    (*this)(0,0) = (( cos__a1__ *  cos__a3__) - (( sin__a1__ *  sin__a2__) *  sin__a3__));
    (*this)(0,1) = ((( cos__a1__ *  sin__a2__) *  sin__a3__) + ( sin__a1__ *  cos__a3__));
    (*this)(0,2) = (- cos__a2__ *  sin__a3__);
    (*this)(1,0) = (- sin__a1__ *  cos__a2__);
    (*this)(1,1) = ( cos__a1__ *  cos__a2__);
    (*this)(1,2) =  sin__a2__;
    (*this)(2,0) = (( cos__a1__ *  sin__a3__) + (( sin__a1__ *  sin__a2__) *  cos__a3__));
    (*this)(2,1) = (( sin__a1__ *  sin__a3__) - (( cos__a1__ *  sin__a2__) *  cos__a3__));
    (*this)(2,2) = ( cos__a2__ *  cos__a3__);
    return *this;
}
iit::eulerang::internal::RotationMatrices::Type_Ezyx_X_S::Type_Ezyx_X_S()
{
}
const iit::eulerang::internal::RotationMatrices::Type_Ezyx_X_S& iit::eulerang::internal::RotationMatrices::Type_Ezyx_X_S::update(const state_t& state) {
    static double sin__a1__;
    static double sin__a2__;
    static double sin__a3__;
    static double cos__a1__;
    static double cos__a2__;
    static double cos__a3__;

    sin__a1__ = std::sin( state.a1);
    sin__a2__ = std::sin( state.a2);
    sin__a3__ = std::sin( state.a3);
    cos__a1__ = std::cos( state.a1);
    cos__a2__ = std::cos( state.a2);
    cos__a3__ = std::cos( state.a3);

    (*this)(0,0) = ( cos__a1__ *  cos__a2__);
    (*this)(0,1) = ( sin__a1__ *  cos__a2__);
    (*this)(0,2) = - sin__a2__;
    (*this)(1,0) = ((( cos__a1__ *  sin__a2__) *  sin__a3__) - ( sin__a1__ *  cos__a3__));
    (*this)(1,1) = ((( sin__a1__ *  sin__a2__) *  sin__a3__) + ( cos__a1__ *  cos__a3__));
    (*this)(1,2) = ( cos__a2__ *  sin__a3__);
    (*this)(2,0) = (( sin__a1__ *  sin__a3__) + (( cos__a1__ *  sin__a2__) *  cos__a3__));
    (*this)(2,1) = ((( sin__a1__ *  sin__a2__) *  cos__a3__) - ( cos__a1__ *  sin__a3__));
    (*this)(2,2) = ( cos__a2__ *  cos__a3__);
    return *this;
}
iit::eulerang::internal::RotationMatrices::Type_Exyx_X_S::Type_Exyx_X_S()
{
}
const iit::eulerang::internal::RotationMatrices::Type_Exyx_X_S& iit::eulerang::internal::RotationMatrices::Type_Exyx_X_S::update(const state_t& state) {
    static double sin__a1__;
    static double sin__a2__;
    static double sin__a3__;
    static double cos__a2__;
    static double cos__a1__;
    static double cos__a3__;

    sin__a1__ = std::sin( state.a1);
    sin__a2__ = std::sin( state.a2);
    sin__a3__ = std::sin( state.a3);
    cos__a2__ = std::cos( state.a2);
    cos__a1__ = std::cos( state.a1);
    cos__a3__ = std::cos( state.a3);

    (*this)(0,0) =  cos__a2__;
    (*this)(0,1) = ( sin__a1__ *  sin__a2__);
    (*this)(0,2) = (- cos__a1__ *  sin__a2__);
    (*this)(1,0) = ( sin__a2__ *  sin__a3__);
    (*this)(1,1) = (( cos__a1__ *  cos__a3__) - (( sin__a1__ *  cos__a2__) *  sin__a3__));
    (*this)(1,2) = ((( cos__a1__ *  cos__a2__) *  sin__a3__) + ( sin__a1__ *  cos__a3__));
    (*this)(2,0) = ( sin__a2__ *  cos__a3__);
    (*this)(2,1) = ((- cos__a1__ *  sin__a3__) - (( sin__a1__ *  cos__a2__) *  cos__a3__));
    (*this)(2,2) = ((( cos__a1__ *  cos__a2__) *  cos__a3__) - ( sin__a1__ *  sin__a3__));
    return *this;
}
iit::eulerang::internal::RotationMatrices::Type_Exzx_X_S::Type_Exzx_X_S()
{
}
const iit::eulerang::internal::RotationMatrices::Type_Exzx_X_S& iit::eulerang::internal::RotationMatrices::Type_Exzx_X_S::update(const state_t& state) {
    static double sin__a2__;
    static double sin__a1__;
    static double sin__a3__;
    static double cos__a2__;
    static double cos__a1__;
    static double cos__a3__;

    sin__a2__ = std::sin( state.a2);
    sin__a1__ = std::sin( state.a1);
    sin__a3__ = std::sin( state.a3);
    cos__a2__ = std::cos( state.a2);
    cos__a1__ = std::cos( state.a1);
    cos__a3__ = std::cos( state.a3);

    (*this)(0,0) =  cos__a2__;
    (*this)(0,1) = ( cos__a1__ *  sin__a2__);
    (*this)(0,2) = ( sin__a1__ *  sin__a2__);
    (*this)(1,0) = (- sin__a2__ *  cos__a3__);
    (*this)(1,1) = ((( cos__a1__ *  cos__a2__) *  cos__a3__) - ( sin__a1__ *  sin__a3__));
    (*this)(1,2) = (( cos__a1__ *  sin__a3__) + (( sin__a1__ *  cos__a2__) *  cos__a3__));
    (*this)(2,0) = ( sin__a2__ *  sin__a3__);
    (*this)(2,1) = (((- cos__a1__ *  cos__a2__) *  sin__a3__) - ( sin__a1__ *  cos__a3__));
    (*this)(2,2) = (( cos__a1__ *  cos__a3__) - (( sin__a1__ *  cos__a2__) *  sin__a3__));
    return *this;
}
iit::eulerang::internal::RotationMatrices::Type_Eyxy_X_S::Type_Eyxy_X_S()
{
}
const iit::eulerang::internal::RotationMatrices::Type_Eyxy_X_S& iit::eulerang::internal::RotationMatrices::Type_Eyxy_X_S::update(const state_t& state) {
    static double sin__a1__;
    static double sin__a3__;
    static double sin__a2__;
    static double cos__a1__;
    static double cos__a3__;
    static double cos__a2__;

    sin__a1__ = std::sin( state.a1);
    sin__a3__ = std::sin( state.a3);
    sin__a2__ = std::sin( state.a2);
    cos__a1__ = std::cos( state.a1);
    cos__a3__ = std::cos( state.a3);
    cos__a2__ = std::cos( state.a2);

    (*this)(0,0) = (( cos__a1__ *  cos__a3__) - (( sin__a1__ *  cos__a2__) *  sin__a3__));
    (*this)(0,1) = ( sin__a2__ *  sin__a3__);
    (*this)(0,2) = (((- cos__a1__ *  cos__a2__) *  sin__a3__) - ( sin__a1__ *  cos__a3__));
    (*this)(1,0) = ( sin__a1__ *  sin__a2__);
    (*this)(1,1) =  cos__a2__;
    (*this)(1,2) = ( cos__a1__ *  sin__a2__);
    (*this)(2,0) = (( cos__a1__ *  sin__a3__) + (( sin__a1__ *  cos__a2__) *  cos__a3__));
    (*this)(2,1) = (- sin__a2__ *  cos__a3__);
    (*this)(2,2) = ((( cos__a1__ *  cos__a2__) *  cos__a3__) - ( sin__a1__ *  sin__a3__));
    return *this;
}
iit::eulerang::internal::RotationMatrices::Type_Eyzy_X_S::Type_Eyzy_X_S()
{
}
const iit::eulerang::internal::RotationMatrices::Type_Eyzy_X_S& iit::eulerang::internal::RotationMatrices::Type_Eyzy_X_S::update(const state_t& state) {
    static double sin__a1__;
    static double sin__a3__;
    static double sin__a2__;
    static double cos__a1__;
    static double cos__a2__;
    static double cos__a3__;

    sin__a1__ = std::sin( state.a1);
    sin__a3__ = std::sin( state.a3);
    sin__a2__ = std::sin( state.a2);
    cos__a1__ = std::cos( state.a1);
    cos__a2__ = std::cos( state.a2);
    cos__a3__ = std::cos( state.a3);

    (*this)(0,0) = ((( cos__a1__ *  cos__a2__) *  cos__a3__) - ( sin__a1__ *  sin__a3__));
    (*this)(0,1) = ( sin__a2__ *  cos__a3__);
    (*this)(0,2) = ((- cos__a1__ *  sin__a3__) - (( sin__a1__ *  cos__a2__) *  cos__a3__));
    (*this)(1,0) = (- cos__a1__ *  sin__a2__);
    (*this)(1,1) =  cos__a2__;
    (*this)(1,2) = ( sin__a1__ *  sin__a2__);
    (*this)(2,0) = ((( cos__a1__ *  cos__a2__) *  sin__a3__) + ( sin__a1__ *  cos__a3__));
    (*this)(2,1) = ( sin__a2__ *  sin__a3__);
    (*this)(2,2) = (( cos__a1__ *  cos__a3__) - (( sin__a1__ *  cos__a2__) *  sin__a3__));
    return *this;
}
iit::eulerang::internal::RotationMatrices::Type_Ezxz_X_S::Type_Ezxz_X_S()
{
}
const iit::eulerang::internal::RotationMatrices::Type_Ezxz_X_S& iit::eulerang::internal::RotationMatrices::Type_Ezxz_X_S::update(const state_t& state) {
    static double sin__a1__;
    static double sin__a3__;
    static double sin__a2__;
    static double cos__a1__;
    static double cos__a3__;
    static double cos__a2__;

    sin__a1__ = std::sin( state.a1);
    sin__a3__ = std::sin( state.a3);
    sin__a2__ = std::sin( state.a2);
    cos__a1__ = std::cos( state.a1);
    cos__a3__ = std::cos( state.a3);
    cos__a2__ = std::cos( state.a2);

    (*this)(0,0) = (( cos__a1__ *  cos__a3__) - (( sin__a1__ *  cos__a2__) *  sin__a3__));
    (*this)(0,1) = ((( cos__a1__ *  cos__a2__) *  sin__a3__) + ( sin__a1__ *  cos__a3__));
    (*this)(0,2) = ( sin__a2__ *  sin__a3__);
    (*this)(1,0) = ((- cos__a1__ *  sin__a3__) - (( sin__a1__ *  cos__a2__) *  cos__a3__));
    (*this)(1,1) = ((( cos__a1__ *  cos__a2__) *  cos__a3__) - ( sin__a1__ *  sin__a3__));
    (*this)(1,2) = ( sin__a2__ *  cos__a3__);
    (*this)(2,0) = ( sin__a1__ *  sin__a2__);
    (*this)(2,1) = (- cos__a1__ *  sin__a2__);
    (*this)(2,2) =  cos__a2__;
    return *this;
}
iit::eulerang::internal::RotationMatrices::Type_Ezyz_X_S::Type_Ezyz_X_S()
{
}
const iit::eulerang::internal::RotationMatrices::Type_Ezyz_X_S& iit::eulerang::internal::RotationMatrices::Type_Ezyz_X_S::update(const state_t& state) {
    static double sin__a1__;
    static double sin__a3__;
    static double sin__a2__;
    static double cos__a1__;
    static double cos__a2__;
    static double cos__a3__;

    sin__a1__ = std::sin( state.a1);
    sin__a3__ = std::sin( state.a3);
    sin__a2__ = std::sin( state.a2);
    cos__a1__ = std::cos( state.a1);
    cos__a2__ = std::cos( state.a2);
    cos__a3__ = std::cos( state.a3);

    (*this)(0,0) = ((( cos__a1__ *  cos__a2__) *  cos__a3__) - ( sin__a1__ *  sin__a3__));
    (*this)(0,1) = (( cos__a1__ *  sin__a3__) + (( sin__a1__ *  cos__a2__) *  cos__a3__));
    (*this)(0,2) = (- sin__a2__ *  cos__a3__);
    (*this)(1,0) = (((- cos__a1__ *  cos__a2__) *  sin__a3__) - ( sin__a1__ *  cos__a3__));
    (*this)(1,1) = (( cos__a1__ *  cos__a3__) - (( sin__a1__ *  cos__a2__) *  sin__a3__));
    (*this)(1,2) = ( sin__a2__ *  sin__a3__);
    (*this)(2,0) = ( cos__a1__ *  sin__a2__);
    (*this)(2,1) = ( sin__a1__ *  sin__a2__);
    (*this)(2,2) =  cos__a2__;
    return *this;
}
