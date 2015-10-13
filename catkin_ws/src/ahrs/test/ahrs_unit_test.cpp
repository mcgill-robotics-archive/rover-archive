#include <gtest/gtest.h>
#include "../libraries/include/ahrs/Ahrs.h"
#include <boost/smart_ptr/scoped_ptr.hpp>
#include <stdexcept>



class AhrsTest: public ::testing::Test
{
    protected:
        AhrsTest(){}
        virtual ~AhrsTest(){}

        virtual void SetUp(){}
        virtual void TearDown () {}
};

TEST(AhrsTest, defaultConfigTest)
{
    lineranger::ahrs::AhrsConfig ahrsConfig;

    ASSERT_EQ(115200, ahrsConfig.getBaudRate());
    ASSERT_EQ("/dev/ahrs", ahrsConfig.getDeviceName());
    ASSERT_TRUE(ahrsConfig.isSimulation());
}

TEST(AhrsTest, factoryTest)
{
    try{
        lineranger::ahrs::AhrsConfig ahrsConfig;
        ahrsConfig.setSimulation(true);

        boost::scoped_ptr<lineranger::ahrs::Ahrs> ahrs
            (lineranger::ahrs::Ahrs::createAhrs(ahrsConfig));
        lineranger::ahrs::AhrsStatus ahrsStatus = ahrs->getStatus();
    }
    catch (std::runtime_error& error)
    {
        ASSERT_TRUE(false) <<
            "an exception was thrown trying to create a virtual ahrs"; 
    }
}

int main (int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
