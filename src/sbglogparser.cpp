#include "sbglogparser.h"

void SBGLogParser::init(const std::string &_topic_for_imu,
                        const std::string &_topic_for_fix,
                        const std::string &_frame_id)
{
    //
    imu_pub = n.advertise<sensor_msgs::Imu>(_topic_for_imu, 10);
    gps_pub = n.advertise<sensor_msgs::NavSatFix>(_topic_for_fix, 10);
    //
    imu_msg.header.frame_id = _frame_id;
    nav_msg.header.frame_id = _frame_id;
}

void SBGLogParser::publish()
{
    // ROS
    if(new_nav_msg){
        nav_msg.header.stamp = ros::Time::now();
        gps_pub.publish(nav_msg);
        new_nav_msg = false;
    }
    if(new_imu_msg){
        imu_msg.header.stamp = ros::Time::now();
        imu_pub.publish(imu_msg);
        new_imu_msg = false;
    }
}

SbgErrorCode SBGLogParser::onLogReceived(SbgEComHandle *pHandle,
                                         SbgEComClass msgClass,
                                         SbgEComMsgId msg,
                                         const SbgBinaryLogData *pLogData,
                                         void *pUserArg)
{
    //    ROS_WARN_STREAM("pUserArg: " << pUserArg);
    SBGLogParser* ptr_sbgwrapper = static_cast<SBGLogParser*>(pUserArg);

    sensor_msgs::Imu &imu_msg = ptr_sbgwrapper->imu_msg;
    sensor_msgs::NavSatFix &nav_msg = ptr_sbgwrapper->nav_msg;

    bool &new_imu_msg = ptr_sbgwrapper->new_imu_msg;
    bool &new_nav_msg = ptr_sbgwrapper->new_nav_msg;

    // float time_of_week;
    switch (msg){
    case SBG_ECOM_LOG_EKF_QUAT:
        imu_msg.orientation.x = pLogData->ekfEulerData.euler[1];
        imu_msg.orientation.y = pLogData->ekfEulerData.euler[2];
        imu_msg.orientation.z = pLogData->ekfEulerData.euler[3];
        imu_msg.orientation.w = pLogData->ekfEulerData.euler[0];
        new_imu_msg = true;
        break;
    case SBG_ECOM_LOG_EKF_NAV:
        nav_msg.latitude  = pLogData->ekfNavData.position[0];
        nav_msg.longitude = pLogData->ekfNavData.position[1];
        nav_msg.altitude  = pLogData->ekfNavData.position[2];
        new_nav_msg = true;
        break;
    case SBG_ECOM_LOG_SHIP_MOTION:
        imu_msg.linear_acceleration.x = pLogData->shipMotionData.shipVel[0];
        imu_msg.linear_acceleration.y = pLogData->shipMotionData.shipVel[1];
        imu_msg.linear_acceleration.z = pLogData->shipMotionData.shipVel[2];
        new_imu_msg = true;
        break;

        // case SBG_ECOM_LOG_UTC_TIME:
        //   pInsSBG->new_time = true;
        //   pInsSBG->year          = pLogData->utcData.year;
        //   pInsSBG->month         = pLogData->utcData.month;
        //   pInsSBG->day           = pLogData->utcData.day;
        //   pInsSBG->hour          = pLogData->utcData.hour;
        //   pInsSBG->minute        = pLogData->utcData.minute;
        //   pInsSBG->second        = pLogData->utcData.second;
        //   pInsSBG->nanoSecond    = pLogData->utcData.nanoSecond;
        //   pInsSBG->gpsTimeOfWeek = pLogData->utcData.gpsTimeOfWeek;
        // break;
    default:
        break;
    }
    return SBG_NO_ERROR;
}
