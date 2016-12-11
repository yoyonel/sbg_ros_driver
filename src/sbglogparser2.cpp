#include "sbglogparser2.h"
#include "sbglogparser2_visitor.h"
#include <boost/exception/all.hpp>

// PS: probleme (warning) pour rendre 'const' onLogReceived ! :-/
// url: http://stackoverflow.com/questions/16344444/how-to-supplement-boostexception-with-a-proper-what-function
SbgErrorCode SBGLogParserImp::onLogReceived(SbgEComHandle *pHandle,
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

    // apply visitor on boost::variant to deal with polymorphism operations
    // (publication of ROS messages)
    auto bound_visitor = visitor_sbglog_data(this);
    boost::apply_visitor(bound_visitor, msg_ros);

//    // FAILED
//    // car 'SBGLogParserImp2' n'hérite pas de 'ISBGLogParser' et le check à la
//    // compile dans 'visitor_sbglog_data' ne passe pas ! (=> c'est le comportement
//    // qu'on souhait :-))
//    class SBGLogParserImp2 {};
//    auto log_parser2 = new SBGLogParserImp2();
//    auto bound_visitor = visitor_sbglog_data(log_parser2);

//    // OK
//    typedef boost::variant<sbg_driver::SbgLogEkfEulerData> bv_sbglog_data2;
//    bv_sbglog_data2 i = sbg_driver::SbgLogEkfEulerData();
//    boost::apply_visitor(bound_visitor, i);

//    // FAILED: car 'sbg_driver::SbgLogUtcData' n'est pas (encore) un type
//    // possible pour boost::variant 'bv_sbglog_data'
//    // => on a un check à la compilation sur les types ou boost::variant sur types
//    // possibles !
//    typedef boost::variant<sbg_driver::SbgLogUtcData> bv_sbglog_data2;
//    bv_sbglog_data2 i = sbg_driver::SbgLogUtcData();
//    boost::apply_visitor(bound_visitor, i);

    return errCode;
}

// url: http://en.cppreference.com/w/cpp/string/byte/memcpy
// Les messages ROS possèdent les mêmes noms (modulo le namespace 'sbh_driver::')
// que les types SBG associés.
// exemple: SbgLogEkfEulerData -> SbgLogEkfEulerData.msg -> sbg_driver::SbgLogEkfEulerData
#define _DEFINE_FUNC_SBGLOG_2_ROSMSG(SUFFIX_NAME, sbg_log_type, sbg_log_var)    \
    SBGLogParserImp::bv_sbglog_data SBGLogParserImp::parser_for_SBG_ECOM_LOG_ ## SUFFIX_NAME(const SbgBinaryLogData* pLogData) const { \
        sbg_driver::sbg_log_type ros_msg;   \
        if (pLogData) std::memcpy(&ros_msg, &pLogData->sbg_log_var, sizeof(sbg_log_type));   \
        return ros_msg; \
    }
//
_DEFINE_FUNC_SBGLOG_2_ROSMSG(EKF_QUAT,      SbgLogEkfEulerData,     ekfEulerData)
_DEFINE_FUNC_SBGLOG_2_ROSMSG(EKF_NAV,       SbgLogEkfNavData,       ekfNavData)
_DEFINE_FUNC_SBGLOG_2_ROSMSG(SHIP_MOTION,   SbgLogShipMotionData,   shipMotionData)
_DEFINE_FUNC_SBGLOG_2_ROSMSG(UTC_DATA,      SbgLogUtcData,          utcData)

// TODO: séparer les publishers ROS de la construction des messages ROS (from SBG API).
void SBGLogParserImp::init_ros_publishers()
{
    map_sbglog_pub[typeid(sbg_driver::SbgLogEkfEulerData)] = n.advertise<sbg_driver::SbgLogEkfEulerData>("SbgLogEkfEulerData", 10);
    map_sbglog_pub[typeid(sbg_driver::SbgLogEkfNavData)] = n.advertise<sbg_driver::SbgLogEkfNavData>("SbgLogEkfNavData", 10);
    map_sbglog_pub[typeid(sbg_driver::SbgLogShipMotionData)] = n.advertise<sbg_driver::SbgLogShipMotionData>("SbgLogShipMotionData", 10);
    map_sbglog_pub[typeid(sbg_driver::SbgLogUtcData)] = n.advertise<sbg_driver::SbgLogUtcData>("SbgLogShipUtcData", 10);
}
