#include <iit/commons/dog/leg_bool_map.h>
#include <gtest/gtest.h>

using namespace iit;
using namespace iit::dog;

TEST (LegBoolMapTest,asserts){
    LegBoolMap test1;


    test1[LegID::LF] = false;
    test1[LegID::RF] = true;
    test1[LegID::LH] = false;
    test1[LegID::RH] = true;


    for (int i=0; i<16; i++){
        LegBoolMap test2;

        test2[LegID::LF] = i & 0b1000;
        test2[LegID::RF] = i & 0b0100;
        test2[LegID::LH] = i & 0b0010;
        test2[LegID::RH] = i & 0b0001;

        std::cout << std::endl;
        std::cout << "-------------------------------------------------" << std::endl;
        std::cout << "Test1: " << std::endl << test1 << std::endl;
        std::cout << "NOT Test1:" << std::endl << !test1 << std::endl << std::endl;
        //ASSERT_TRUE(test1);


        std::cout<< std::endl;
        std::cout << "Test" << i+2 <<": " << std::endl << test2 << std::endl;
        std::cout << "NOT Test" << i+2 <<": " << std::endl << !test2 << std::endl;
        //ASSERT_TRUE(test2);

        std::cout<< std::endl;
        std::cout << "Cwise OR Test1: " <<  test1.OR() << std::endl;
        std::cout << "Cwise OR Test1 on Test" << i+2 <<": " << test2.OR(test1) << std::endl;
        //ASSERT_TRUE(test1.OR());
        //ASSERT_TRUE(test2.OR(test1));

        std::cout<< std::endl;
        std::cout << "Cwise OR Test" << i+2 <<": " << test2.OR() << std::endl;
        std::cout << "Cwise OR Test" << i+2 <<" on Test1: " << test1.OR(test2) << std::endl;
        //ASSERT_TRUE(test2.OR());
        //ASSERT_TRUE(test1.OR(test2));

        std::cout<< std::endl;
        std::cout << "Cwise AND Test1: " << test1.AND() << std::endl;
        std::cout << "Cwise AND Test1 on Test" << i+2 <<": " << test2.AND(test1) << std::endl;
        //ASSERT_TRUE(test1.AND());
        //ASSERT_TRUE(test2.AND(test1));

        std::cout<< std::endl;
        std::cout << "Cwise AND Test" << i+2 <<": " << test2.AND() << std::endl;
        std::cout << "Cwise AND Test" << i+2 <<" on Test1: " << test1.AND(test2) << std::endl;
        //ASSERT_TRUE(test2.AND());
        //ASSERT_TRUE(test1.AND(test2));

        std::cout<< std::endl;
        std::cout << "Test1 || Test" << i+2 <<": " << std::endl << (test1 || test2) << std::endl << std::endl;
        std::cout << "Test1 && Test" << i+2 <<": " << std::endl << (test1 && test2) << std::endl;
        //ASSERT_TRUE((test1 || test2));
        //ASSERT_TRUE((test1 && test2));
    }






}


int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

