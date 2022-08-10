


#include "imu_complementary_filter1/complementary_filter_ros.h"


int main(int argc, char ** argv){
    ros::init(argc, argc, "ComplementaryFilterROS1");

    ros::NodeHandle nh;

    ros::NodeHandle nh_private("~");

    imu_tools::ComplementaryFilterROS1 filter(nh, nh_private);

    ros::spin();

    return 0;
}   