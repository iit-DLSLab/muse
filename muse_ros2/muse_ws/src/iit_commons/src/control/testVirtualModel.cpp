
#include <Eigen/Dense>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <iomanip>


#include <iit/commons/dog/declarations.h>
#include <iit/commons/dog/jsim.h>

#include <iit/commons/dog/leg_data_map.h>
#include <iit/commons/dog/leg_bool_map.h>
#include <iit/commons/control/VirtualModel.hpp>

#include <iit/commons/planning/planning.h>
#include <iit/commons/geometry/algebra.h>

#include <gtest/gtest.h>

using namespace std;
using namespace iit;
using namespace iit::control;
using namespace Eigen;

class VirtualModelTest : public ::testing::Test {
protected:
    virtual void SetUp(){
        actual_orient.x = Vector3d(0.1, -0.1,0.2);
        des_pos.x = Vector3d(2,2,2);
        des_orient.x = Vector3d(0,0,1.57);
        des_pos.xdd = Vector3d(10,20,30);
        des_orient.xdd = Vector3d(30,20,10);

        Ic<<   4.6455 , -0.0289564 ,    0.2777  ,        0 ,   2.53078 ,         0,
                -0.0289564 ,    13.3818 ,    0.0038  , -2.53078 ,         0 ,  -1.94249,
                0.2777 ,     0.0038 ,   13.2335  ,       -0 ,   1.94249 ,         0,
                0 ,   -2.53078 ,        -0  ,   97.554 ,         0 ,         0,
                2.53078 ,          0 ,   1.94249  ,        0 ,    97.554 ,         0,
                0 ,   -1.94249 ,         0  ,        0 ,         0 ,    97.554;


        vmodelWB = VirtualModel();
        vmodelWB.setGains(1000,1000,1000, 1000,1000,1000, 1000,1000,1000,1000,1000,1000);


        wrenchErrorTh << 400.0, 400.0,400.0,400.0,400.0,400.0;
        wrenchError << 0.0,600.0,0.0 ,200.0,0.0,0.0;


        vmodelWB.setWrenchErrorThreshold(wrenchErrorTh);

        testFeedbackWrench << -97.5102784,
                -134.083735,
                1109.99496,
                831.739743,
                1587.2676,
                2754.26749;

        testFeedforwardWrench <<-266.692493,
                141.775291,
                140.742,
                975.54,
                1951.08,
                2926.62;


        testTotalWrench <<-364.202771,
                7.69155651,
                1250.73696,
                1807.27974,
                3538.3476,
                5680.88749;

        feedbackWrench = vmodelWB.getFeedBackWrench(des_pos,
                                                    actual_pos,
                                                    des_orient,
                                                    actual_orient,
                                                    Ic.getMass(),
                                                    wrenchError);

        feedforwardWrench = vmodelWB.getFeedForwardWrench(des_pos.xdd,
                                                          des_orient.xdd,
                                                          des_orient.x,
                                                          des_orient.xd,
                                                          Ic.getMass(),
                                                          Ic);

        totalWrench = vmodelWB.getTotalWrench(des_pos,
                                              actual_pos,
                                              des_orient,
                                              actual_orient,
                                              Ic.getMass(),
                                              Ic,
                                              wrenchError);

        sumOfTerms = feedforwardWrench + feedbackWrench - totalWrench;

    }

    virtual void TearDown(){

    }
    iit::rbd::InertiaMatrixDense Ic;
    iit::planning::Point3d des_pos;
    iit::planning::Point3d actual_pos;
    iit::planning::Point3d des_orient;
    iit::planning::Point3d actual_orient;
    control::VirtualModel vmodelWB;
    rbd::Vector6D wrenchError;
    rbd::Vector6D wrenchErrorTh;

    rbd::Vector6D diffFeedbackWrench;
    rbd::Vector6D diffFeedforwardWrench;
    rbd::Vector6D diffTotalWrench;

    rbd::Vector6D testFeedbackWrench;
    rbd::Vector6D testFeedforwardWrench;
    rbd::Vector6D testTotalWrench;

    rbd::Vector6D feedbackWrench;
    rbd::Vector6D feedforwardWrench;
    rbd::Vector6D totalWrench;
    rbd::Vector6D sumOfTerms;

};

TEST_F(VirtualModelTest, FeedBackPlusForwardEqTotal){
    ASSERT_FLOAT_EQ(sumOfTerms(0),0);
    ASSERT_FLOAT_EQ(sumOfTerms(1),0);
    ASSERT_FLOAT_EQ(sumOfTerms(2),0);
    ASSERT_FLOAT_EQ(sumOfTerms(3),0);
    ASSERT_FLOAT_EQ(sumOfTerms(4),0);
    ASSERT_FLOAT_EQ(sumOfTerms(5),0);
}

TEST_F(VirtualModelTest, FeedBackWrench){
    ASSERT_FLOAT_EQ(diffFeedbackWrench(0),0);
    ASSERT_FLOAT_EQ(diffFeedbackWrench(1),0);
    ASSERT_FLOAT_EQ(diffFeedbackWrench(2),0);
    ASSERT_FLOAT_EQ(diffFeedbackWrench(3),0);
    ASSERT_FLOAT_EQ(diffFeedbackWrench(4),0);
    ASSERT_FLOAT_EQ(diffFeedbackWrench(5),0);
}

TEST_F(VirtualModelTest, FeedForwardWrench){
    ASSERT_FLOAT_EQ(diffFeedforwardWrench(0),0);
    ASSERT_FLOAT_EQ(diffFeedforwardWrench(1),0);
    ASSERT_FLOAT_EQ(diffFeedforwardWrench(2),0);
    ASSERT_FLOAT_EQ(diffFeedforwardWrench(3),0);
    ASSERT_FLOAT_EQ(diffFeedforwardWrench(4),0);
    ASSERT_FLOAT_EQ(diffFeedforwardWrench(5),0);
}

TEST_F(VirtualModelTest, TotalWrench){
    ASSERT_FLOAT_EQ(diffTotalWrench(0),0);
    ASSERT_FLOAT_EQ(diffTotalWrench(1),0);
    ASSERT_FLOAT_EQ(diffTotalWrench(2),0);
    ASSERT_FLOAT_EQ(diffTotalWrench(3),0);
    ASSERT_FLOAT_EQ(diffTotalWrench(4),0);
    ASSERT_FLOAT_EQ(diffTotalWrench(5),0);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}



