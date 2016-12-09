#ifndef SBGLOGPARSER_H
#define SBGLOGPARSER_H

#include "ros/ros.h"
//
#include <sbgEComLib.h>
#include <sbgEComIds.h>
//
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"


class SBGLogParser {
public:
    SBGLogParser(ros::NodeHandle _n=ros::NodeHandle(),
                 ros::NodeHandle _private_nh=ros::NodeHandle("~")):
        n(_n),
        private_nh(_private_nh)
    { }

    static SbgErrorCode onLogReceived(SbgEComHandle *pHandle,
                                      SbgEComClass msgClass,
                                      SbgEComMsgId msg,
                                      const SbgBinaryLogData *pLogData,
                                      void *pUserArg);

    void init(const std::string &_topic_for_imu="imu",
              const std::string &_topic_for_fix="fix",
              const std::string &_frame_id="map");

    void publish();

protected:

private:
    sensor_msgs::Imu imu_msg;
    sensor_msgs::NavSatFix nav_msg;

    bool new_imu_msg;
    bool new_nav_msg;
    bool new_twist_msg;

    ros::Publisher imu_pub;
    ros::Publisher gps_pub;

    ros::NodeHandle n;
    ros::NodeHandle private_nh;
};

typedef boost::shared_ptr< SBGLogParser > SBGLogParserPtr;
typedef boost::shared_ptr< SBGLogParser const> SBGLogParserConstPtr;

#endif // SBGLOGPARSER_H