#include "sbglogparser2.h"
#include <boost/exception/all.hpp>

// url: http://stackoverflow.com/questions/16344444/how-to-supplement-boostexception-with-a-proper-what-function
SbgErrorCode SBGLogParser2::onLogReceived(SbgEComHandle *pHandle,
                                          SbgEComClass msgClass,
                                          SbgEComMsgId msg,
                                          const SbgBinaryLogData *pLogData)
{
    SbgErrorCode errCode = SBG_NO_ERROR;
    try {
        f[static_cast<SbgEComLog>(msg)](this, pLogData);
    }
    catch(const boost::exception& e) {
        ROS_ERROR_STREAM("Bad function call: " << boost::diagnostic_information(e));
        ROS_ERROR_STREAM("msg: " << int(msg));
        errCode = SBG_ERROR;
    }

    return errCode;
}

void SBGLogParser2::parser_for_SBG_ECOM_LOG_EKF_QUAT(const SbgBinaryLogData*)
{
    ROS_INFO_STREAM("parser_for_SBG_ECOM_LOG_EKF_QUAT");
    return;
}

void SBGLogParser2::parser_for_SBG_ECOM_LOG_EKF_NAV(const SbgBinaryLogData*)
{
    ROS_INFO_STREAM("parser_for_SBG_ECOM_LOG_EKF_NAV");
    return;
}

void SBGLogParser2::parser_for_SBG_ECOM_LOG_SHIP_MOTION(const SbgBinaryLogData*)
{
    ROS_INFO_STREAM("parser_SBG_ECOM_LOG_SHIP_MOTION");
    return;
}
