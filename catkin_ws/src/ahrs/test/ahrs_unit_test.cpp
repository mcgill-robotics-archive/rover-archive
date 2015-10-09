#include <gtest/gtest.h>
#include "../libraries/include/ahrs/AhrsConfig.h"

TEST(AhrsTest, configTest)
{
    lineranger::ahrs::AhrsConfig ahrsConfig;

    ASSERT_EQ(115200, ahrsConfig.getBaudRate());
}

int main (int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
