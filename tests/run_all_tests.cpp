#include <gtest/gtest.h>
#include "test_tools.cpp"
#include "test_kalman_filter.cpp"
#include "test_fusion_ekf.cpp"

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}