#include "SBGLogtoROSMsg.h"
//
#include <boost/exception/all.hpp>

// url: http://stackoverflow.com/questions/16344444/how-to-supplement-boostexception-with-a-proper-what-function
SbgErrorCode SBGLogtoROSMsg::onLogReceived(SbgEComHandle *pHandle,
                                           SbgEComClass msgClass,
                                           SbgEComMsgId msg,
                                           const SbgBinaryLogData *pLogData)
{
    SbgErrorCode errCode = SBG_NO_ERROR;
    bv_sbglog_data msg_ros;
    try {
        msg_ros = f[static_cast<SbgEComLog>(msg)](this, pLogData);
    }
    catch(const boost::exception& e) {
        ROS_ERROR_STREAM("Bad function call: " << boost::diagnostic_information(e));
        ROS_ERROR_STREAM("msg: " << int(msg));
        errCode = SBG_ERROR;
    }

    sbglog_rospublisher->publish(msg_ros);

    return errCode;
}

#define _DEFINE_FUNC_SBGLOG_2_ROSMSG(sbg_log_type, sbg_log_var)        SBGLogtoROSMsg::bv_sbglog_data SBGLogtoROSMsg::parser_for_ ## sbg_log_type(const SbgBinaryLogData* pLogData) const {     sbg_driver::sbg_log_type ros_msg;       if (pLogData) std::memcpy(&ros_msg, &pLogData->sbg_log_var, sizeof(sbg_log_type));       return ros_msg;     }
//
_DEFINE_FUNC_SBGLOG_2_ROSMSG(SbgLogStatusData,	statusData)
_DEFINE_FUNC_SBGLOG_2_ROSMSG(SbgLogImuData,	imuData)
_DEFINE_FUNC_SBGLOG_2_ROSMSG(SbgLogEkfEulerData,	ekfEulerData)
_DEFINE_FUNC_SBGLOG_2_ROSMSG(SbgLogEkfQuatData,	ekfQuatData)
_DEFINE_FUNC_SBGLOG_2_ROSMSG(SbgLogEkfNavData,	ekfNavData)
_DEFINE_FUNC_SBGLOG_2_ROSMSG(SbgLogShipMotionData,	shipMotionData)
_DEFINE_FUNC_SBGLOG_2_ROSMSG(SbgLogOdometerData,	odometerData)
_DEFINE_FUNC_SBGLOG_2_ROSMSG(SbgLogUtcData,	utcData)
_DEFINE_FUNC_SBGLOG_2_ROSMSG(SbgLogMag,	magData)
_DEFINE_FUNC_SBGLOG_2_ROSMSG(SbgLogMagCalib,	magCalibData)
_DEFINE_FUNC_SBGLOG_2_ROSMSG(SbgLogDvlData,	dvlData)
_DEFINE_FUNC_SBGLOG_2_ROSMSG(SbgLogPressureData,	pressureData)
_DEFINE_FUNC_SBGLOG_2_ROSMSG(SbgLogUsblData,	usblData)
_DEFINE_FUNC_SBGLOG_2_ROSMSG(SbgLogDebug0Data,	debug0Data)
_DEFINE_FUNC_SBGLOG_2_ROSMSG(SbgLogFastImuData,	fastImuData)
