#ifndef WRAPPER_SPECIALIZATION_H
#define WRAPPER_SPECIALIZATION_H

#include "sbgECom.h"    // for SbgLog*Data
#include <unordered_map>    // for std::unordered_map

// include des headers des messages ROS
#include "sbg_driver/SbgLogImuData.h"  // for sbg_driver::SbgLogImuData
#include "sbg_driver/SbgLogEvent.h"  // for sbg_driver::SbgLogEvent
#include "sbg_driver/SbgLogMagCalib.h"  // for sbg_driver::SbgLogMagCalib
#include "sbg_driver/SbgLogEkfQuatData.h"  // for sbg_driver::SbgLogEkfQuatData
#include "sbg_driver/SbgLogDvlData.h"  // for sbg_driver::SbgLogDvlData
#include "sbg_driver/SbgLogUsblData.h"  // for sbg_driver::SbgLogUsblData
#include "sbg_driver/SbgLogPressureData.h"  // for sbg_driver::SbgLogPressureData
#include "sbg_driver/SbgLogFastImuData.h"  // for sbg_driver::SbgLogFastImuData
#include "sbg_driver/SbgLogOdometerData.h"  // for sbg_driver::SbgLogOdometerData
#include "sbg_driver/SbgLogStatusData.h"  // for sbg_driver::SbgLogStatusData
#include "sbg_driver/SbgLogEkfEulerData.h"  // for sbg_driver::SbgLogEkfEulerData
#include "sbg_driver/SbgLogGpsHdt.h"  // for sbg_driver::SbgLogGpsHdt
#include "sbg_driver/SbgLogGpsVel.h"  // for sbg_driver::SbgLogGpsVel
#include "sbg_driver/SbgLogShipMotionData.h"  // for sbg_driver::SbgLogShipMotionData
#include "sbg_driver/SbgLogEkfNavData.h"  // for sbg_driver::SbgLogEkfNavData
#include "sbg_driver/SbgLogUtcData.h"  // for sbg_driver::SbgLogUtcData
#include "sbg_driver/SbgLogMag.h"  // for sbg_driver::SbgLogMag
#include "sbg_driver/SbgLogGpsRaw.h"  // for sbg_driver::SbgLogGpsRaw
#include "sbg_driver/SbgLogGpsPos.h"  // for sbg_driver::SbgLogGpsPos
#include "sbg_driver/SbgLogDebug0Data.h"  // for sbg_driver::SbgLogDebug0Data

//----------------------------------------------------------------------------------
// Association des enums log de sbg aux ros msg type par spécialisation de templates
//----------------------------------------------------------------------------------

const std::unordered_map<SbgEComMsgId, SbgEComLog> map_int_to_sbgecomlog = {
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_STATUS), SBG_ECOM_LOG_STATUS},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_IMU_DATA), SBG_ECOM_LOG_IMU_DATA},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_EKF_EULER), SBG_ECOM_LOG_EKF_EULER},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_EKF_QUAT), SBG_ECOM_LOG_EKF_QUAT},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_EKF_NAV), SBG_ECOM_LOG_EKF_NAV},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_SHIP_MOTION), SBG_ECOM_LOG_SHIP_MOTION},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_SHIP_MOTION_HP), SBG_ECOM_LOG_SHIP_MOTION_HP},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_ODO_VEL), SBG_ECOM_LOG_ODO_VEL},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_UTC_TIME), SBG_ECOM_LOG_UTC_TIME},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_MAG), SBG_ECOM_LOG_MAG},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_MAG_CALIB), SBG_ECOM_LOG_MAG_CALIB},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_DVL_BOTTOM_TRACK), SBG_ECOM_LOG_DVL_BOTTOM_TRACK},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_PRESSURE), SBG_ECOM_LOG_PRESSURE},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_USBL), SBG_ECOM_LOG_USBL},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_GPS2_POS), SBG_ECOM_LOG_GPS2_POS},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_GPS1_POS), SBG_ECOM_LOG_GPS1_POS},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_GPS2_VEL), SBG_ECOM_LOG_GPS2_VEL},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_GPS1_VEL), SBG_ECOM_LOG_GPS1_VEL},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_GPS1_HDT), SBG_ECOM_LOG_GPS1_HDT},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_GPS2_HDT), SBG_ECOM_LOG_GPS2_HDT},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_GPS2_RAW), SBG_ECOM_LOG_GPS2_RAW},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_GPS1_RAW), SBG_ECOM_LOG_GPS1_RAW},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_EVENT_B), SBG_ECOM_LOG_EVENT_B},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_EVENT_C), SBG_ECOM_LOG_EVENT_C},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_EVENT_A), SBG_ECOM_LOG_EVENT_A},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_EVENT_D), SBG_ECOM_LOG_EVENT_D},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_EVENT_E), SBG_ECOM_LOG_EVENT_E},
};

const std::unordered_map<SbgEComMsgId, const char*> map_msg_to_string = {
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_STATUS), "SBG_ECOM_LOG_STATUS"},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_IMU_DATA), "SBG_ECOM_LOG_IMU_DATA"},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_EKF_EULER), "SBG_ECOM_LOG_EKF_EULER"},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_EKF_QUAT), "SBG_ECOM_LOG_EKF_QUAT"},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_EKF_NAV), "SBG_ECOM_LOG_EKF_NAV"},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_SHIP_MOTION), "SBG_ECOM_LOG_SHIP_MOTION"},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_SHIP_MOTION_HP), "SBG_ECOM_LOG_SHIP_MOTION_HP"},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_ODO_VEL), "SBG_ECOM_LOG_ODO_VEL"},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_UTC_TIME), "SBG_ECOM_LOG_UTC_TIME"},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_MAG), "SBG_ECOM_LOG_MAG"},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_MAG_CALIB), "SBG_ECOM_LOG_MAG_CALIB"},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_DVL_BOTTOM_TRACK), "SBG_ECOM_LOG_DVL_BOTTOM_TRACK"},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_PRESSURE), "SBG_ECOM_LOG_PRESSURE"},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_USBL), "SBG_ECOM_LOG_USBL"},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_GPS2_POS), "SBG_ECOM_LOG_GPS2_POS"},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_GPS1_POS), "SBG_ECOM_LOG_GPS1_POS"},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_GPS2_VEL), "SBG_ECOM_LOG_GPS2_VEL"},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_GPS1_VEL), "SBG_ECOM_LOG_GPS1_VEL"},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_GPS1_HDT), "SBG_ECOM_LOG_GPS1_HDT"},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_GPS2_HDT), "SBG_ECOM_LOG_GPS2_HDT"},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_GPS2_RAW), "SBG_ECOM_LOG_GPS2_RAW"},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_GPS1_RAW), "SBG_ECOM_LOG_GPS1_RAW"},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_EVENT_B), "SBG_ECOM_LOG_EVENT_B"},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_EVENT_C), "SBG_ECOM_LOG_EVENT_C"},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_EVENT_A), "SBG_ECOM_LOG_EVENT_A"},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_EVENT_D), "SBG_ECOM_LOG_EVENT_D"},
    {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_EVENT_E), "SBG_ECOM_LOG_EVENT_E"},
};

template <SbgEComLog enum_log>
struct SBGLogEnum_Types {
};

template <>
struct SBGLogEnum_Types<SBG_ECOM_LOG_STATUS> {
    typedef SbgLogStatusData t_sbglog;
    typedef sbg_driver::SbgLogStatusData t_rosmsg;
};

template <>
struct SBGLogEnum_Types<SBG_ECOM_LOG_IMU_DATA> {
    typedef SbgLogImuData t_sbglog;
    typedef sbg_driver::SbgLogImuData t_rosmsg;
};

template <>
struct SBGLogEnum_Types<SBG_ECOM_LOG_EKF_EULER> {
    typedef SbgLogEkfEulerData t_sbglog;
    typedef sbg_driver::SbgLogEkfEulerData t_rosmsg;
};

template <>
struct SBGLogEnum_Types<SBG_ECOM_LOG_EKF_QUAT> {
    typedef SbgLogEkfQuatData t_sbglog;
    typedef sbg_driver::SbgLogEkfQuatData t_rosmsg;
};

template <>
struct SBGLogEnum_Types<SBG_ECOM_LOG_EKF_NAV> {
    typedef SbgLogEkfNavData t_sbglog;
    typedef sbg_driver::SbgLogEkfNavData t_rosmsg;
};

template <>
struct SBGLogEnum_Types<SBG_ECOM_LOG_SHIP_MOTION> {
    typedef SbgLogShipMotionData t_sbglog;
    typedef sbg_driver::SbgLogShipMotionData t_rosmsg;
};

template <>
struct SBGLogEnum_Types<SBG_ECOM_LOG_SHIP_MOTION_HP> {
    typedef SbgLogShipMotionData t_sbglog;
    typedef sbg_driver::SbgLogShipMotionData t_rosmsg;
};

template <>
struct SBGLogEnum_Types<SBG_ECOM_LOG_ODO_VEL> {
    typedef SbgLogOdometerData t_sbglog;
    typedef sbg_driver::SbgLogOdometerData t_rosmsg;
};

template <>
struct SBGLogEnum_Types<SBG_ECOM_LOG_UTC_TIME> {
    typedef SbgLogUtcData t_sbglog;
    typedef sbg_driver::SbgLogUtcData t_rosmsg;
};

template <>
struct SBGLogEnum_Types<SBG_ECOM_LOG_MAG> {
    typedef SbgLogMag t_sbglog;
    typedef sbg_driver::SbgLogMag t_rosmsg;
};

template <>
struct SBGLogEnum_Types<SBG_ECOM_LOG_MAG_CALIB> {
    typedef SbgLogMagCalib t_sbglog;
    typedef sbg_driver::SbgLogMagCalib t_rosmsg;
};

template <>
struct SBGLogEnum_Types<SBG_ECOM_LOG_DVL_BOTTOM_TRACK> {
    typedef SbgLogDvlData t_sbglog;
    typedef sbg_driver::SbgLogDvlData t_rosmsg;
};

template <>
struct SBGLogEnum_Types<SBG_ECOM_LOG_PRESSURE> {
    typedef SbgLogPressureData t_sbglog;
    typedef sbg_driver::SbgLogPressureData t_rosmsg;
};

template <>
struct SBGLogEnum_Types<SBG_ECOM_LOG_USBL> {
    typedef SbgLogUsblData t_sbglog;
    typedef sbg_driver::SbgLogUsblData t_rosmsg;
};

template <>
struct SBGLogEnum_Types<SBG_ECOM_LOG_GPS2_POS> {
    typedef SbgLogGpsPos t_sbglog;
    typedef sbg_driver::SbgLogGpsPos t_rosmsg;
};

template <>
struct SBGLogEnum_Types<SBG_ECOM_LOG_GPS1_POS> {
    typedef SbgLogGpsPos t_sbglog;
    typedef sbg_driver::SbgLogGpsPos t_rosmsg;
};

template <>
struct SBGLogEnum_Types<SBG_ECOM_LOG_GPS2_VEL> {
    typedef SbgLogGpsVel t_sbglog;
    typedef sbg_driver::SbgLogGpsVel t_rosmsg;
};

template <>
struct SBGLogEnum_Types<SBG_ECOM_LOG_GPS1_VEL> {
    typedef SbgLogGpsVel t_sbglog;
    typedef sbg_driver::SbgLogGpsVel t_rosmsg;
};

template <>
struct SBGLogEnum_Types<SBG_ECOM_LOG_GPS1_HDT> {
    typedef SbgLogGpsHdt t_sbglog;
    typedef sbg_driver::SbgLogGpsHdt t_rosmsg;
};

template <>
struct SBGLogEnum_Types<SBG_ECOM_LOG_GPS2_HDT> {
    typedef SbgLogGpsHdt t_sbglog;
    typedef sbg_driver::SbgLogGpsHdt t_rosmsg;
};

template <>
struct SBGLogEnum_Types<SBG_ECOM_LOG_GPS2_RAW> {
    typedef SbgLogGpsRaw t_sbglog;
    typedef sbg_driver::SbgLogGpsRaw t_rosmsg;
};

template <>
struct SBGLogEnum_Types<SBG_ECOM_LOG_GPS1_RAW> {
    typedef SbgLogGpsRaw t_sbglog;
    typedef sbg_driver::SbgLogGpsRaw t_rosmsg;
};

template <>
struct SBGLogEnum_Types<SBG_ECOM_LOG_EVENT_B> {
    typedef SbgLogEvent t_sbglog;
    typedef sbg_driver::SbgLogEvent t_rosmsg;
};

template <>
struct SBGLogEnum_Types<SBG_ECOM_LOG_EVENT_C> {
    typedef SbgLogEvent t_sbglog;
    typedef sbg_driver::SbgLogEvent t_rosmsg;
};

template <>
struct SBGLogEnum_Types<SBG_ECOM_LOG_EVENT_A> {
    typedef SbgLogEvent t_sbglog;
    typedef sbg_driver::SbgLogEvent t_rosmsg;
};

template <>
struct SBGLogEnum_Types<SBG_ECOM_LOG_EVENT_D> {
    typedef SbgLogEvent t_sbglog;
    typedef sbg_driver::SbgLogEvent t_rosmsg;
};

template <>
struct SBGLogEnum_Types<SBG_ECOM_LOG_EVENT_E> {
    typedef SbgLogEvent t_sbglog;
    typedef sbg_driver::SbgLogEvent t_rosmsg;
};

#endif
