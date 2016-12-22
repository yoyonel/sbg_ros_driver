#ifndef SBGLOGPARSER_STD_H
#define SBGLOGPARSER_STD_H

#include "sbglogparser/isbglogparser.h"  // for ISBGLogParser (& ROS, SBG dependancies)
#include "sbglogparser_specialization.h"
#include "sbglogparser_exception.h"
#include <map>      // for std::map
#include <memory>   // for std::shared_ptr
#include <string>   // for std::string
#include <unordered_map>
#include <bits/shared_ptr.h>

namespace ros {
typedef std::shared_ptr<Publisher> PublisherPtr;
}

namespace sbglogparser_std {
struct SBGLogParser : public ISBGLogParser {
    SBGLogParser(ros::NodeHandle _n=ros::NodeHandle(),
                   ros::NodeHandle _private_nh=ros::NodeHandle("~"))
        : ISBGLogParser(_n, _private_nh)
    {}

    void publish(SbgEComMsgId msg, const SbgBinaryLogData *pLogData);

    // -------------------------------------------------------------------------
    // OVERRIDE: des méthodes virtuelles de la classe ISBGLogParser
    // -------------------------------------------------------------------------
    // On ne fait rien (actuellement) dans publish car on publit directement
    // à la réception des messages (par callback).
    // Faudrait voir, si on réintroduit le mécanisme
    // de publication déréférencé (asynchrone).
    void publish() override {}

    void init() override {}

    SbgErrorCode onLogReceived(SbgEComHandle *pHandle,
                               SbgEComClass msgClass,
                               SbgEComMsgId msg,
                               const SbgBinaryLogData *pLogData) override;
    // -------------------------------------------------------------------------

protected:
    template<SbgEComLog enum_log,
             typename TSbgLog=typename SBGLogEnum_Types<enum_log>::t_sbglog,
             typename TRosMsg=typename SBGLogEnum_Types<enum_log>::t_rosmsg>
    void publish(const SbgBinaryLogData *pLogData, ros::NodeHandle& _n);

    template<SbgEComLog enum_log,
             typename TSbgLog=typename SBGLogEnum_Types<enum_log>::t_sbglog,
             typename TRosMsg=typename SBGLogEnum_Types<enum_log>::t_rosmsg>
    inline void publish(const SbgBinaryLogData *pLogData) {
        publish<enum_log, TSbgLog, TRosMsg>(pLogData, n);
    }

    template<typename TSbgLog, typename TRosMsg>
    TRosMsg build_rosmsg(const SbgBinaryLogData *pLogData);

    template<typename TRosMsg>
    ros::Publisher get_rospub(const SbgEComLog& enum_log, ros::NodeHandle& n);

    template<typename TRosMsg>
    inline ros::Publisher get_rospub(const SbgEComLog& enum_log) {
        return get_rospub<TRosMsg>(enum_log, n);
    }

    template<typename TRosMsg>
    std::string build_topic_name(
            const std::string& _prefix_pattern="SbgLog",
            const std::string& _suffix_pattern="Data",
            const std::string& _suffix_for_rostopic_name="");

private:
    std::map<SbgEComLog, ros::PublisherPtr> map_enum_rospub;
    ros::NodeHandle n;

    // urls:
    // - http://en.cppreference.com/w/cpp/container/unordered_map
    // -> http://supercomputingblog.com/windows/ordered-map-vs-unordered-map-a-performance-study/
    // - http://en.cppreference.com/w/cpp/language/lambda
    // - http://www.cprogramming.com/c++11/c++11-lambda-closures.html
    typedef std::function<void(const SbgBinaryLogData *)> fun_t;

    const std::unordered_map<SbgEComMsgId, fun_t> map_msg_to_publish = {
        {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_STATUS), [&](const SbgBinaryLogData *pLogData) { publish<SBG_ECOM_LOG_STATUS>(pLogData); }},
        {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_IMU_DATA), [&](const SbgBinaryLogData *pLogData) { publish<SBG_ECOM_LOG_IMU_DATA>(pLogData); }},
        {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_EKF_EULER), [&](const SbgBinaryLogData *pLogData) { publish<SBG_ECOM_LOG_EKF_EULER>(pLogData); }},
        {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_EKF_QUAT), [&](const SbgBinaryLogData *pLogData) { publish<SBG_ECOM_LOG_EKF_QUAT>(pLogData); }},
        {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_EKF_NAV), [&](const SbgBinaryLogData *pLogData) { publish<SBG_ECOM_LOG_EKF_NAV>(pLogData); }},
        {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_SHIP_MOTION), [&](const SbgBinaryLogData *pLogData) { publish<SBG_ECOM_LOG_SHIP_MOTION>(pLogData); }},
        {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_SHIP_MOTION_HP), [&](const SbgBinaryLogData *pLogData) { publish<SBG_ECOM_LOG_SHIP_MOTION_HP>(pLogData); }},
        {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_ODO_VEL), [&](const SbgBinaryLogData *pLogData) { publish<SBG_ECOM_LOG_ODO_VEL>(pLogData); }},
        {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_UTC_TIME), [&](const SbgBinaryLogData *pLogData) { publish<SBG_ECOM_LOG_UTC_TIME>(pLogData); }},
        {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_MAG), [&](const SbgBinaryLogData *pLogData) { publish<SBG_ECOM_LOG_MAG>(pLogData); }},
        {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_MAG_CALIB), [&](const SbgBinaryLogData *pLogData) { publish<SBG_ECOM_LOG_MAG_CALIB>(pLogData); }},
        {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_DVL_BOTTOM_TRACK), [&](const SbgBinaryLogData *pLogData) { publish<SBG_ECOM_LOG_DVL_BOTTOM_TRACK>(pLogData); }},
        {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_PRESSURE), [&](const SbgBinaryLogData *pLogData) { publish<SBG_ECOM_LOG_PRESSURE>(pLogData); }},
        {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_USBL), [&](const SbgBinaryLogData *pLogData) { publish<SBG_ECOM_LOG_USBL>(pLogData); }},
        {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_GPS2_POS), [&](const SbgBinaryLogData *pLogData) { publish<SBG_ECOM_LOG_GPS2_POS>(pLogData); }},
        {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_GPS1_POS), [&](const SbgBinaryLogData *pLogData) { publish<SBG_ECOM_LOG_GPS1_POS>(pLogData); }},
        {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_GPS2_VEL), [&](const SbgBinaryLogData *pLogData) { publish<SBG_ECOM_LOG_GPS2_VEL>(pLogData); }},
        {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_GPS1_VEL), [&](const SbgBinaryLogData *pLogData) { publish<SBG_ECOM_LOG_GPS1_VEL>(pLogData); }},
        {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_GPS1_HDT), [&](const SbgBinaryLogData *pLogData) { publish<SBG_ECOM_LOG_GPS1_HDT>(pLogData); }},
        {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_GPS2_HDT), [&](const SbgBinaryLogData *pLogData) { publish<SBG_ECOM_LOG_GPS2_HDT>(pLogData); }},
        {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_GPS2_RAW), [&](const SbgBinaryLogData *pLogData) { publish<SBG_ECOM_LOG_GPS2_RAW>(pLogData); }},
        {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_GPS1_RAW), [&](const SbgBinaryLogData *pLogData) { publish<SBG_ECOM_LOG_GPS1_RAW>(pLogData); }},
        {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_EVENT_B), [&](const SbgBinaryLogData *pLogData) { publish<SBG_ECOM_LOG_EVENT_B>(pLogData); }},
        {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_EVENT_C), [&](const SbgBinaryLogData *pLogData) { publish<SBG_ECOM_LOG_EVENT_C>(pLogData); }},
        {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_EVENT_A), [&](const SbgBinaryLogData *pLogData) { publish<SBG_ECOM_LOG_EVENT_A>(pLogData); }},
        {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_EVENT_D), [&](const SbgBinaryLogData *pLogData) { publish<SBG_ECOM_LOG_EVENT_D>(pLogData); }},
        {static_cast<SbgEComMsgId>(SBG_ECOM_LOG_EVENT_E), [&](const SbgBinaryLogData *pLogData) { publish<SBG_ECOM_LOG_EVENT_E>(pLogData); }},
    };

};
// Templates implementations
#include "sbglogparser.inl"
}   // namespace sbglogparser_std

#endif // SBGLOGPARSER_STD_H
