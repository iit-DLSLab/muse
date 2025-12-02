#include "iit/commons/geometry/rotations.h"
#include <gtest/gtest.h>


using namespace std;
using namespace Eigen;

class RotationTest : public ::testing::Test {

protected:
    virtual void SetUp(){
    }
    virtual void TearDown(){}
};



TEST_F(RotationTest, test_twovecToRot){
    //line intersect
    Eigen::Vector3d V1, V2, V2_reconstructed;

    V1 = Vector3d(1.0,0.0,0.0);
    
    for (int i=0; i<100; i++) 
    {    
        V2 = Vector3d(cos(2.0*M_PI/100.0*(double)i),sin(2.0*M_PI/100.0 * (double)i),0);
           
        Matrix3d R = iit::commons::twovecToRot(V1, V2);
        //std::cout<<"iter: "<<i<< "    V1:   " << V1.head(2).transpose()<< "  V2:   "<<V2.head(2).transpose()<< std::endl;
        //std::cout<<V2 - R*V1<<std::endl;
        
        V2_reconstructed = R*V1;
        
        for(int i = 0; i < 3; i++){
                ASSERT_NEAR(V2(i), V2_reconstructed(i),1e-4);
        }
  
    }
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
