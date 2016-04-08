// Bring in my package's API, which is what I'm testing
#include "../allocator/lagrange_allocator.h"
// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(TestSuite, basicTest)
{
    EXPECT_TRUE(true);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
