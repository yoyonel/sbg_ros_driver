#ifndef SBGLOGPARSER_H
#define SBGLOGPARSER_H

#include "sbglogparser/isbglogparser.h"  // for ISBGLogParser (& ROS, SBG dependancies)
//
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"

namespace sbglogparser_simple {
class SBGLogParser : public ISBGLogParser {
public:
    SBGLogParser(ros::NodeHandle _n=ros::NodeHandle(),
                 ros::NodeHandle _private_nh=ros::NodeHandle("~")):
        ISBGLogParser(_n, _private_nh)
    { }

    // -------------------------------------------------------------------------
    // OVERRIDE
    // -------------------------------------------------------------------------
    SbgErrorCode onLogReceived(SbgEComHandle *pHandle,
                               SbgEComClass msgClass,
                               SbgEComMsgId msg,
                               const SbgBinaryLogData *pLogData) override;

    void init() override;

    void publish() override;
    // -------------------------------------------------------------------------

    // ---------------------------------------
    // Specific initialization
    // ---------------------------------------
    void init_for_ros_topics(const std::string &_topic_for_imu="imu",
                             const std::string &_topic_for_fix="fix",
                             const std::string &_frame_id="map");
    // ---------------------------------------

private:
    sensor_msgs::Imu imu_msg;
    sensor_msgs::NavSatFix nav_msg;

    bool new_imu_msg;
    bool new_nav_msg;
//    bool new_twist_msg;

    ros::Publisher imu_pub;
    ros::Publisher gps_pub;
};

//typedef boost::shared_ptr< SBGLogParser > SBGLogParserPtr;
//typedef boost::shared_ptr< SBGLogParser const> SBGLogParserConstPtr;

}

#endif // SBGLOGPARSER_H
