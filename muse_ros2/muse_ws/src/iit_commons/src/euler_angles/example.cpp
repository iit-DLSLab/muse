#include <iit/rbd/rbd.h>
#include <iit/commons/euler_angles/transform.h>
#include <iit/commons/euler_angles/transforms_internal.h>

#include <iostream>
#include <ctime>

using namespace iit;
using namespace std;


int main()
{
    std::srand(std::time(NULL));
    eulerang::angles_state state = {
            ((double)std::rand())/RAND_MAX,
            ((double)std::rand())/RAND_MAX,
            ((double)std::rand())/RAND_MAX};

    typedef eulerang::Convention convention;
    eulerang::Transform<convention::XYZ>::Base_R_Rotated Rxyz;
    eulerang::Transform<convention::XYZ>::Rotated_R_Base RIxyz;
    //eulerang::Transform<convention::XZY>::Base_R_Rotated Rxzy;
    //eulerang::Transform<convention::XZX>::Base_R_Rotated Rxzx;
    cout << Rxyz(state) << endl << endl;
    cout << RIxyz(state) << endl << endl;
    cout << Rxyz * RIxyz << endl << endl; // should be the identity

    static_assert(eulerang::getConvention(Rxyz) == eulerang::Convention::XYZ, "");
}



