#include "sbglogparser2.h"
#include <boost/exception/all.hpp>

// url: http://stackoverflow.com/questions/16344444/how-to-supplement-boostexception-with-a-proper-what-function
SbgErrorCode SBGLogParser2::onLogReceived(SbgEComHandle *pHandle,
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

    return errCode;
}

bv_sbglog_data SBGLogParser2::parser_for_SBG_ECOM_LOG_EKF_QUAT(const SbgBinaryLogData* pLogData)
{
    ROS_INFO_STREAM("parser_for_SBG_ECOM_LOG_EKF_QUAT");

    // Bridge messages
    // SBG (Logs) -> ROS (Messages)
    sbg_driver::SbgLogEkfEulerData msg_efk_euler;
    //
    if (pLogData) {
        // fonctionne car le message ROS suit la définition de type
        // de SBG_Log data type (EulerData dans ce cas).
        std::memcpy(&msg_efk_euler, &pLogData->ekfEulerData, sizeof(SbgLogEkfEulerData));
    }
    return msg_efk_euler;
}

bv_sbglog_data SBGLogParser2::parser_for_SBG_ECOM_LOG_EKF_NAV(const SbgBinaryLogData* pLogData)
{
    ROS_INFO_STREAM("parser_for_SBG_ECOM_LOG_EKF_NAV");

    // PS: Bug QtCreator ! Quand il y a plusieurs types (SbgLogEkfEulerData, SbgLogEkfNavData)
    // dans un même namespace (sbg_driver) définit dans plusieurs fichiers sources/headers
    // QtCreator ne reconnait pas (AutoComplete pas) tous les types (en l'occurrence
    // il semble ne reconnaitre (gérer) que le 1er ...).

    sbg_driver::SbgLogEkfNavData msg_efk_nav;
    if (pLogData) std::memcpy(&msg_efk_nav, &pLogData->ekfNavData, sizeof(SbgLogEkfNavData));
    return msg_efk_nav;
}

bv_sbglog_data SBGLogParser2::parser_for_SBG_ECOM_LOG_SHIP_MOTION(const SbgBinaryLogData* pLogData)
{
    ROS_INFO_STREAM("parser_SBG_ECOM_LOG_SHIP_MOTION");

    sbg_driver::SbgLogShipMotionData msg_ship_motion;
    if (pLogData) std::memcpy(&msg_ship_motion, &pLogData->shipMotionData, sizeof(SbgLogShipMotionData));
    return msg_ship_motion;
}
