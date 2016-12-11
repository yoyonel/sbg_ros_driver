#include "sbglogparser2.h"
#include <boost/exception/all.hpp>

class visitor_sbglog_data : public boost::static_visitor<>
{
public:
    visitor_sbglog_data(const SBGLogParser2* _log_parser):
        boost::static_visitor<>(), log_parser(_log_parser)
    {}

    // utilisation de la forme templatisée de l'operator de visitor
    // car l'opération (modulo le type instancé dans boost::variant) reste
    // la même (récupération d'un publisher et publication du message). Le seul
    // paramètre dynamique est le type du ROS message (type du message lié au
    // type du log récupéré via l'API SBG).
    template <typename T>
    inline
    void operator()(T& ros_msg_with_SBGData) const
    {
        try {
            const ros::Publisher& pub = log_parser->get_pub<T>();
            pub.publish(ros_msg_with_SBGData);
        }
        catch(std::out_of_range) {
        }
    }

private:
    const SBGLogParser2* log_parser;
};

// PS: probleme (warning) pour rendre 'const' onLogReceived ! :-/
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

    // apply visitor on boost::variant to deal with polymorphism operations
    // (publication of ROS messages)
    const visitor_sbglog_data& bound_visitor = visitor_sbglog_data(this);
    boost::apply_visitor(bound_visitor, msg_ros);

    return errCode;
}

bv_sbglog_data SBGLogParser2::parser_for_SBG_ECOM_LOG_EKF_QUAT(const SbgBinaryLogData* pLogData) const
{
//    ROS_INFO_STREAM("parser_for_SBG_ECOM_LOG_EKF_QUAT");

    // Bridge messages
    // SBG (Logs) -> ROS (Messages)
    sbg_driver::SbgLogEkfEulerData msg_efk_euler;
    //
    if (pLogData) {
        // fonctionne car le message ROS suit la définition de type
        // de SBG_Log data type (EulerData dans ce cas).
        // url: http://en.cppreference.com/w/cpp/string/byte/memcpy
        std::memcpy(&msg_efk_euler, &pLogData->ekfEulerData, sizeof(SbgLogEkfEulerData));
    }
    return msg_efk_euler;
}

bv_sbglog_data SBGLogParser2::parser_for_SBG_ECOM_LOG_EKF_NAV(const SbgBinaryLogData* pLogData) const
{
//    ROS_INFO_STREAM("parser_for_SBG_ECOM_LOG_EKF_NAV");
    sbg_driver::SbgLogEkfNavData msg_efk_nav;
    if (pLogData) std::memcpy(&msg_efk_nav, &pLogData->ekfNavData, sizeof(SbgLogEkfNavData));
    return msg_efk_nav;
}

bv_sbglog_data SBGLogParser2::parser_for_SBG_ECOM_LOG_SHIP_MOTION(const SbgBinaryLogData* pLogData) const
{
//    ROS_INFO_STREAM("parser_SBG_ECOM_LOG_SHIP_MOTION");
    sbg_driver::SbgLogShipMotionData msg_ship_motion;
    if (pLogData) std::memcpy(&msg_ship_motion, &pLogData->shipMotionData, sizeof(SbgLogShipMotionData));
    return msg_ship_motion;
}

// TODO: séparer les publishers ROS de la construction des messages ROS (from SBG API).
void SBGLogParser2::init_ros_publishers()
{
    map_sbglog_pub[typeid(sbg_driver::SbgLogEkfEulerData)] = n.advertise<sbg_driver::SbgLogEkfEulerData>("SbgLogEkfEulerData", 10);
    map_sbglog_pub[typeid(sbg_driver::SbgLogEkfNavData)] = n.advertise<sbg_driver::SbgLogEkfNavData>("SbgLogEkfNavData", 10);
    map_sbglog_pub[typeid(sbg_driver::SbgLogShipMotionData)] = n.advertise<sbg_driver::SbgLogShipMotionData>("SbgLogShipMotionData", 10);
}
