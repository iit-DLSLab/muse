#include "iit/commons/geometry/algebra.h"
#include <gtest/gtest.h>

using namespace std;
using namespace Eigen;

class AlgebraTest : public ::testing::Test {

protected:
    virtual void SetUp(){
        m1 << NAN, 10, NAN, 11, 12, NAN, 13, INFINITY;
        m1_expected << 0, 10, 0, 11, 12, 0, 13, 0;

        m2_expected << 1, 10, 3, 11, 12, 6, 13, 8;

        m3 << 1, 2, 3, 4, 5, 6, 7, 8;
    }
    virtual void TearDown(){}

    Matrix<double, 8, 1> m1;
    Matrix<double, 8, 1> m1_expected;
    Matrix<double, 8, 1> m2_expected;
    Matrix<double, 8, 1> m3;
};

TEST_F(AlgebraTest, SetFiniteSingle){
    iit::commons::setFinite(m1);
    for(int i = 0; i < m1.size(); i++){
        EXPECT_DOUBLE_EQ(m1_expected(i), m1(i));
    }
}

TEST_F(AlgebraTest, SetFiniteDual){
    iit::commons::setFinite(m1, m3);
    for(int i = 0; i < m1.size(); i++){
        EXPECT_DOUBLE_EQ(m2_expected(i), m1(i));
    }
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
