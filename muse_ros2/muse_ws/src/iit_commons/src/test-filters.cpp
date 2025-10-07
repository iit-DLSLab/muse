#include <iit/commons/filters.h>
#include <gtest/gtest.h>

#include <iostream>
#include <fstream>

using namespace iit::commons;
using namespace std;

TEST (TestFilters,test) {
    double a = 0.001/(0.001+0.005);

    AlphaFilter<float> af1(a, 10.0);
    AlphaFilter<float> af2(a, 10.0);

    FilterPipe< float > fpipe(af1,af2);
    SecondOrderAlphaFilter<float> af2nd(a, 10.0);

    const float demand = 18.99;
    for(int i=0; i<40; i++) {
        ASSERT_NEAR(fpipe.filter(demand), af2nd.filter(demand),1e-4);
        //std::cout << fpipe.filter(demand) << "   " << af2nd.filter(demand) << endl;
    }
}


/*
 * Acquire the results produced by this test with Octave, with:
 * > d1 = fread(fopen('fout1'), Inf, 'float32');
 * > d2 = fread(fopen('fout2'), Inf, 'float32');
 * > d3 = fread(fopen('fout3'), Inf, 'float32');
 */
/*TEST (TestFiltersFile,testFile) {
    double a = 0.001/(0.001+0.005);

    AlphaFilter<float> af1(a, 10.0);
    AlphaFilter<float> af2(a, 10.0);
    FilterPipe< float > fpipe(af1,af2);


    SecondOrderAlphaFilter<float> af2nd(a, 10.0);

    SecondOrderAlphaFilter<float> af2nd2(a, 10.0);
    AlphaFilter<float> af3(a, 10.0);
    auto af3rd = make_filtpipe(af2nd2, af3);


    auto fval1 = make_fvar(10.0f, make_limits(-100.0f,100.0f), fpipe );
    auto fval2 = make_fvar(10.0f, make_limits(-100.0f, 17.0f) , af2nd);
    auto fval3 = make_fvar(10.0f, make_limits(-100.0f,100.0f), af3rd );
    float v = 0.0;

    std::ofstream fout1("fout1", std::ios::binary);
    std::ofstream fout2("fout2", std::ios::binary);
    std::ofstream fout3("fout3", std::ios::binary);
    float demand = 18.99;
    for(int i=0; i<100; i++) {
        v = fval1.update(demand);
        fout1.write((const char*)(&v), sizeof(v));
        v = fval2.update(demand);
        fout2.write((const char*)(&v), sizeof(v));
        v = fval3.update(demand);
        fout3.write((const char*)(&v), sizeof(v));
    }

    demand = 5.67;
    for(int i=0; i<50; i++) {
        v = fval1.update(demand);
        fout1.write((const char*)(&v), sizeof(v));
        v = fval2.update(demand);
        fout2.write((const char*)(&v), sizeof(v));
        v = fval3.update(demand);
        fout3.write((const char*)(&v), sizeof(v));
    }
}*/


int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
