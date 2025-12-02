#include "iit/commons/planning/planning.h"
#include <gtest/gtest.h>


using namespace std;
using namespace Eigen;

class PlanningTest : public ::testing::Test {

protected:
    virtual void SetUp(){
    }
    virtual void TearDown(){}
};



TEST_F(PlanningTest, test_triangles){
    //line intersect
    Eigen::Vector3d P1, P2,P3,P4,intersection;

    P1 = Vector3d(0.5,0.2,0.1);
    P2 = Vector3d(-0.3,-0.2,0);
    P3 = Vector3d(-0.3,0.2,0);
    P4 = Vector3d(0.3,-0.2,0);
    double s;
    iit::planning::line_intersect_from_points(P1,P2,P3,P4,intersection, s);
    //std::cout<<"intersection: "<<intersection<<std::endl;
    //std::cout<<"s: "<<s<<std::endl;

    Vector3d expected_intersection;
    double expected_s = 0.425062;
    expected_intersection<< 0.0425062,-0.0299753,  0.0212531;
    for(int i = 0; i < 3; i++){
        ASSERT_NEAR(expected_intersection(i), intersection(i),1e-4);
    }
    ASSERT_NEAR(expected_s, s,1e-4);

}

TEST_F(PlanningTest, test_halfplane_description){
    //line intersect
    Eigen::Vector3d P1, P2,P3,P4;

    P1 = Vector3d(0.0, 0.0, 0.0);
    P2 = Vector3d(1.2, 1.0, 0.0);
    P3 = Vector3d(0.1,1.2, 0.0);
    P4 = Vector3d(1.0, 1.2, 0.0);

    Eigen::MatrixXd vertices, A;
    Eigen::VectorXd b;
    vertices.resize(3,4);
    vertices.col(0) =  P1;
    vertices.col(1) =  P2;
    vertices.col(2) =  P3;
    vertices.col(3) =  P4;

    //std::cout<<"vertices = "<<std::endl<< vertices <<std::endl;

    iit::planning::computeHalfPlaneDescription(vertices, A, b);
    //std::cout<<"A = "<< std::endl<< A <<std::endl;
    //std::cout<<"b = "<< std::endl<< b <<std::endl;

    Eigen::Vector3d inner_point_to_test;
    inner_point_to_test <<0.4,0.8, 0.0;
    //std::cout<<"margin = "<< std::endl << A*inner_point_to_test+b <<std::endl;
//    std::cout<<"point "<< inner_point_to_test.transpose()<< " is inside polygon?: "
//             << iit::planning::checkInsidePolygon(inner_point_to_test, A, b) <<std::endl;

      Eigen::MatrixXd expected_A;
      Eigen::VectorXd expected_b;
      expected_A.resize(4,3);
      expected_b.resize(4);

      expected_A<< -0.6402,    0.7682, 0.0,
                   -0.7071,   -0.7071, 0.0,
                        0 ,   -1.0000, 0.0,
                    0.9965,   -0.0830, 0.0;

      expected_b <<          0,
                        1.5556,
                        1.2000,
                             0;
     for(int i = 0; i < 4; i++)
     {
         for(int j = 0; j < 3; j++)
              ASSERT_NEAR(expected_A(i,j), A(i,j),1e-4);
     }
     for(int i = 0; i < 4; i++)
     {
         ASSERT_NEAR(expected_b(i), b(i),1e-4);
     }

     //test if point is inside polygon
     ASSERT_TRUE(iit::planning::checkInsidePolygon(inner_point_to_test, A, b));

     Eigen::MatrixXd shrinked_vertices, expected_shrinked_vertices;
     expected_shrinked_vertices.resize(3,4);
     iit::planning::shrinkPolygon(vertices, 0.5, shrinked_vertices);

     expected_shrinked_vertices <<0.3551, 0.9551, 0.4051, 0.8551,
                                  0.2282, 0.7282, 0.8282, 0.8282,
                                  0,      0,      0,      0;

     for(int i = 0; i < 3; i++)
     {
         for(int j = 0; j < 4; j++)
             ASSERT_NEAR(expected_shrinked_vertices(i,j), shrinked_vertices(i,j),1e-4);
     }
 }

TEST_F(PlanningTest, test_polygon_area){
    Eigen::Vector3d P1, P2,P3,P4;

    P1 = Vector3d(0.0, 0.0, 0.0);
    P2 = Vector3d(2.0, 0.0, 0.0);
    P3 = Vector3d(2.2,1.0, 0.0);
    P4 = Vector3d(0.2, 1.0, 0.0);
    Eigen::MatrixXd vertices;
    vertices.resize(3,4);
    vertices.col(0) =  P1;
    vertices.col(1) =  P2;
    vertices.col(2) =  P3;
    vertices.col(3) =  P4;
    double area = iit::planning::computePolygonArea(vertices);
    //std::cout<<"area" << area;
    ASSERT_NEAR(area, 2.0,1e-4);
}

TEST_F(PlanningTest, test_margin_from_poly){


    std::cout<<"Margin Test" << std::endl;
    iit::dog::LegDataMap<bool>  stance_legs = true;
    iit::dog::LegDataMap<Eigen::Vector3d>  actual_feetW;

    actual_feetW[iit::dog::LF] = Vector3d(0.3, 0.2, 0.0);
    actual_feetW[iit::dog::RF] = Vector3d(0.3, -0.2, 0.0);
    actual_feetW[iit::dog::LH] = Vector3d(-0.3,0.2, 0.0);
    actual_feetW[iit::dog::RH] = Vector3d(-0.3, -0.2, 0.0);

    Eigen::Vector3d point_to_test= Vector3d(0.25, 0.0, 0.44);
    double margin = iit::planning::margin_from_poly(point_to_test, stance_legs, actual_feetW);
    std::cout<<"margin from poly: " << margin << std::endl;
    ASSERT_NEAR(margin, 0.05,1e-4);
}


int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
