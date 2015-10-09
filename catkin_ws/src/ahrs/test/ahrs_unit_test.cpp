#include <gtest/gtest.h>
#include "../libraries/include/ahrs/AhrsConfig.h"

TEST(AhrsTest, defaultConfigTest)
{
    lineranger::ahrs::AhrsConfig ahrsConfig;

    ASSERT_EQ(115200, ahrsConfig.getBaudRate());
    ASSERT_EQ("/dev/ahrs", ahrsConfig.getDeviceName());
    ASSERT_TRUE(ahrsConfig.isSimulation());
}

int main (int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
